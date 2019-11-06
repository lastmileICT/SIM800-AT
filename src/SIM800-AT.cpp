/* SIM800_AT firmware
 * Copyright (c) 2016-2019 Connected Energy Technologies Ltd
 * (www.connectedenergy.net)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "SIM800-AT.h"
#include <inttypes.h>

int GPRS::request_data()
{
    send_cmd("AT+CIPRXGET=2,100");
    return 0;
}

int GPRS::read_resp(char *rx_buffer, int size)
{
    Timer t;
    t.start();
    int pos = 0;
    while (t.read() < DEFAULT_TIMEOUT) {
        if (pos < size) {
            if (gprsSerial.readable()) {
                rx_buffer[pos] = gprsSerial.getc();
                pos++;
            }
        }
        else
            return pos;
    }
    return pos;
}


void GPRS::send_cmd(const char *cmd)
{
    gprsSerial.puts(cmd);
    gprsSerial.puts("\r\n");
}

int GPRS::check_resp(const char *resp, int timeout)
{
    Timer t;
    t.start();
    const int len = strlen(resp);
    int sum = 0;    // sum of equal characters found
    while (t.read() < DEFAULT_TIMEOUT) {
        if (gprsSerial.readable()) {
            if (gprsSerial.getc() == resp[sum]) {
                sum++;
            }
            else {
                sum = 0;    // reset counter
            }

            if (sum == len) {
                return 0;
            }
        }
    }
    return -1;
}


int GPRS::init()
{
    for(int i = 0; i < 3; i++) {
        send_cmd("AT");
        if(0 != check_resp("OK", DEFAULT_TIMEOUT))
            return -1;
    }
    send_cmd("ATE0");
    if(0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    return 0;
}

int GPRS::wakeup()
{
    send_cmd("AT");
    send_cmd("AT+CSCLK=0");
    if (0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    return 0;
}

int GPRS::check_pin(void)
{
    send_cmd("AT+CPIN?");
    if (0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    return 0;
}

int GPRS::set_pin(const char* pin)
{
    char cpin[20];
    sprintf(cpin, "AT+CPIN=\"%s\"", pin);
    send_cmd(cpin);
    if(0 != check_resp("OK",DEFAULT_TIMEOUT))
        return -1;
    return 0;
}

int GPRS::enable_SSL(const char *filename)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+SSLSETCERT=%s,ABC", filename);
    send_cmd(cmd);
    if(0 != check_resp("+SSLSETCERT: 0", DEFAULT_TIMEOUT)){
        return -1;
    }
    send_cmd("AT+CIPSSL=1");
    if (0 != check_resp("OK", DEFAULT_TIMEOUT)) {
        return -1;
    }
    return 0;
}

int GPRS::setup_clock()
{
    send_cmd("AT+CNTPCID=1");
    if (0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    send_cmd("AT+CNTP=time1.google.com,0");
    if (0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    send_cmd("AT+CNTP");
    if (0 != check_resp("+CNTP: 1", DEFAULT_TIMEOUT))
        return -1;

    return 0;
}

int GPRS::enable_bearer(const char* apn, const char* user, const char* pass)
{
    char cmd[64];
    send_cmd("AT+SAPBR=3,1,Contype,GPRS");
    if(0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    snprintf(cmd, sizeof(cmd), "AT+SAPBR=3,1,APN,\"%s\"", apn);
    send_cmd(cmd);
    if(0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    snprintf(cmd, sizeof(cmd), "AT+SAPBR=3,1,USER,\"%s\"", user);
    send_cmd(cmd);
    if(0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    snprintf(cmd, sizeof(cmd), "AT+SAPBR=3,1,PWD,\"%s\"", pass);
    send_cmd(cmd);
    if(0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    send_cmd("AT+SAPBR=1,1");
    if(0 != check_resp("OK", DEFAULT_TIMEOUT)) {
        disable_bearer();
        return -1;
    }
    return 0;
}

int GPRS::network_availablity()
{
    send_cmd("AT+CREG?");
    if(0 == check_resp("+CREG: 0,1", DEFAULT_TIMEOUT))
        return 0;
    send_cmd("AT+CREG?");
    if(0 == check_resp("+CREG: 0,5", DEFAULT_TIMEOUT))
        return 0;
    return -1;
}

int GPRS::check_signal_strength(void)
{
    char gprsBuffer[16];
    int value;
    send_cmd("AT+CSQ");
    read_resp(gprsBuffer,16);
    sscanf(gprsBuffer, " +CSQ: %d,", &value);
    int rssi = (2*value - 113);
    if ((rssi >= 0) || (rssi < -155))
        return -1;
    return rssi;
}

uint32_t GPRS::get_time()
{
    char rx_data[64];
    send_cmd("AT+CCLK?");
    read_resp(rx_data, sizeof(rx_data));
    rx_data[sizeof(rx_data)-1] = '\0';  // make sure we have a null-terminated string for sscanf

    struct tm timeinfo;
    int timezone;
    int items = sscanf(rx_data, " +CCLK: \"%2d/%2d/%2d,%2d:%2d:%2d%3d\" ",
        &timeinfo.tm_year, &timeinfo.tm_mon, &timeinfo.tm_mday,
        &timeinfo.tm_hour, &timeinfo.tm_min, &timeinfo.tm_sec,
        &timezone);

    if (items == 7) {                   // all information found
        timeinfo.tm_year += 100;        // struct tm starts counting years from 1900
        timeinfo.tm_mon -= 1;           // struct tm starts counting months from 0 for January
        uint32_t timestamp = mktime(&timeinfo); // - timezone * 15 * 60;     // SIM800 provides timezone as multiple of 15 mins but if using NTP, time will always be the specified zone
        return timestamp;
    }
    else
        return -1;
}

int GPRS::attach_gprs()
{
    send_cmd("AT+CGATT=1");
    if(0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    send_cmd("AT+CIPMUX=0");
    if(0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    send_cmd("AT+CIPRXGET=1");
    if(0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    return 0;
}

int GPRS::set_apn(const char* apn, const char* user, const char* pass)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CSTT=\"%s\",\"%s\",\"%s\"", apn, user, pass);
    send_cmd(cmd);
    if(0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    return 0;
}

int GPRS::activate_gprs()
{
    send_cmd("AT+CIICR");
    if(0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    return 0;
}

int GPRS::get_ip()
{
    char ip[16];
    send_cmd("AT+CIFSR");
    read_resp(ip, 16);
    return 0;
}

int GPRS::connect_tcp(const char *ip, const char *port)
{
    char cmd[100];
    sprintf(cmd, "AT+CIPSTART=TCP,%s,%s", ip, port);
    send_cmd(cmd);
    if(0 != check_resp("CONNECT OK", DEFAULT_TIMEOUT)){
        send_cmd(cmd);
        if(0 != check_resp("ALREADY CONNECT", DEFAULT_TIMEOUT))
            return -1;
    }
    return 0;
}

int GPRS::close_tcp()
{
    char resp[10];
    send_cmd("AT+CIPCLOSE");
    read_resp(resp, 10);
    send_cmd("AT+CIPSHUT");
    return 0;
}

int GPRS::detach_gprs()
{
    send_cmd("AT+CGATT=0");
    if(0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    return 0;
}

int GPRS::disable_bearer()
{
    send_cmd("AT+SAPBR=0,1");
    if(0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    return 0;
}

void GPRS::sleep()
{
    send_cmd("AT+CSCLK=2");
}

int GPRS::send_tcp_data(unsigned char *data, int len)
{
    char cmd[64];
    snprintf(cmd,sizeof(cmd),"AT+CIPSEND=%d",len);
    clear_buffer();
    send_cmd(cmd);
    if (0 != check_resp(">", DEFAULT_TIMEOUT))
        return -1;
    for (int i = 0; i <= len; i++)
    {
        gprsSerial.putc(data[i]);
    }
    if (0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    return 0;
}

void GPRS::clear_buffer()
{
    // For non-buffered serial, even one getc would be enough...
    // Using for-loop to prevent accidental indefinite loop
    for (int i = 0; i < 1000 && gprsSerial.readable(); i++) {
        gprsSerial.getc();
    }
}

int GPRS::check_ssl_cert(const char *filename, int filesize)
{
    char cmd[64];
    char resp[64];
    snprintf(cmd, sizeof(cmd), "AT+FSFLSIZE=%s", filename);
    send_cmd(cmd);
    snprintf(resp, sizeof(resp), "FSFLSIZE: %d", filesize);
    if(0 != check_resp(resp, DEFAULT_TIMEOUT)){
        return -1;
    }
    return 0;
}

int GPRS::load_ssl(const char *filename, const char *cert, int filesize)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+FSCREATE=%s", filename);
    send_cmd(cmd);
    if(0 != check_resp("OK", DEFAULT_TIMEOUT)){
        return -1;
    }
    snprintf(cmd, sizeof(cmd), "AT+FSWRITE=%s,0,%d,5", filename, filesize);
    send_cmd(cmd);
    if(0 != check_resp(">", DEFAULT_TIMEOUT)){
        return -1;
    }
    send_cmd(cert);
    if(0 != check_resp("OK", DEFAULT_TIMEOUT)){
        return -1;
    }
    return 0;
 }

int GPRS::reset(void)
{
    send_cmd("AT+CFUN=0");
    if (0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    send_cmd("AT+CFUN=1,1");
    if (0 != check_resp("OK", 5))
        return -1;
    return 0;
}

 int GPRS::init_SMS(void)
{
    send_cmd("AT+CMGF=1");
    if (0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    send_cmd("AT+CNMI=2,1,0,0,0");
    if (0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    return 0;
}

int GPRS::check_new_SMS(void)
{
    char buf[20];
    char *s;
    int sms_location = 0;
    read_resp(buf, sizeof(buf));
    if(NULL == (s = strstr(buf,"+CMTI")))
        return -1;
    sscanf(buf, "+CMTI: %s,%d", s, &sms_location);
    return sms_location;
}

int GPRS::get_SMS(int index, char* message)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CMGR=%d", index);
    send_cmd(cmd);
    read_resp(message, sizeof(message));
    return -1;
}

int GPRS::send_get_request(char* url)
{
    char cmd[64];
    send_cmd("AT+HTTPINIT");
    if (0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    send_cmd("AT+HTTPPARA=\"CID\",1");
    if (0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    send_cmd("AT+HTTPSSL=1");
    if (0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    send_cmd("AT+HTTPPARA=\"REDIR\",1");
    if (0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"%s\"", url);
    send_cmd(cmd);
    if (0 != check_resp("OK", DEFAULT_TIMEOUT))
        return -1;
    send_cmd("AT+HTTPACTION=0");
    if (0 != check_resp("+HTTPACTION", 5))
        return -1;
    send_cmd("AT+HTTPREAD");
}

//////// THIS IS THE END OF FUNCTIONS CURRENTLY IN USE ////////////


int GPRS::sendSMS(char *number, char *data)
{
    char cmd[64];
    while(gprsSerial.readable()) {
        gprsSerial.getc();
    }
    snprintf(cmd, sizeof(cmd),"AT+CMGS=\"%s\"",number);
    send_cmd(cmd);
    if(0 != check_resp(">",DEFAULT_TIMEOUT))
        return -1;
    gprsSerial.puts(data);
    gprsSerial.putc((char)0x1a);
    return 0;
}

int GPRS::deleteSMS(int index)
{
    char cmd[32];
    snprintf(cmd,sizeof(cmd),"AT+CMGD=%d",index);
    send_cmd(cmd);
    return 0;
}

int GPRS::answer(void)
{
    gprsSerial.printf("ATA");
    return 0;
}

int GPRS::callUp(char *number)
{
    send_cmd("AT+COLP=1");
    if(0 != check_resp("OK",5))
        return -1;
    gprsSerial.printf("\r\nATD+ %s;", number);
    return 0;
}

bool GPRS::get_location(float *latitude, float *longitude)
{
    char *location[10];
    const char s[2] = ",";
    char *token;
    char gprsBuffer[100];
    int i = 0;
    send_cmd("AT+CIPGSMLOC=1,1");
    read_resp(gprsBuffer,sizeof(gprsBuffer));
    if (strstr(gprsBuffer, "$$+CIPGSMLOC: ") != 0) {
        token = strtok(gprsBuffer,s);
        while (token != NULL) {
            location[i] = token;
            token = strtok(NULL,s);
            i++;
        }
        sscanf(location[1],"%f",longitude);
        sscanf(location[2],"%f",latitude);
        return true;
    }
    return false;
}