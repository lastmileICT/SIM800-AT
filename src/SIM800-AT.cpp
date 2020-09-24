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

#ifndef UNIT_TEST

#include "SIM800-AT.h"
#include <inttypes.h>

#ifdef __MBED__

int GPRS::request_data(void)
{
    send_cmd("AT+CIPRXGET=2,100");
    return MODEM_RESPONSE_OK;
}

int GPRS::read_resp(char *resp_buf, int size_buf, int time_out, const char *ack_message)
{
    Timer t;
    t.start();

    int index_buf = 0;
    int exp_ack_num_bytes = strlen(ack_message);
    int actual_ack_num_bytes = 0;

    while (t.read() < time_out) {
        if (index_buf < size_buf-1) {
            if (gprsSerial.readable()) {
                resp_buf[index_buf] = gprsSerial.getc();
                if (ack_message != NULL) { //Decide if an acknowledgement check required
                    // Checks if the acknowledgement string is received correctly
                    if (resp_buf[index_buf] == ack_message[actual_ack_num_bytes]) {
                        actual_ack_num_bytes++;
                    }
                    else {
                        actual_ack_num_bytes = 0; // Reset the counter
                    }

                    if (actual_ack_num_bytes == exp_ack_num_bytes) { // Exit condition
                        resp_buf[0] = '\0'; // Null char to clear the buffer
                        return MODEM_RESPONSE_OK;
                    }
                }
                index_buf++; // Only increments if a byte received
            }
        }
        else { // If the received number of bytes reached the maximum expected size.
            resp_buf[index_buf] = '\0'; // Add a null char at the end of the received data
            return MODEM_RESPONSE_ERROR; // This is actually not an invalid response
        }
    }
    // If timed-out
    resp_buf[index_buf] = '\0'; // Add a null char at the end of the received data
    return MODEM_RESPONSE_ERROR;
}

void GPRS::send_cmd(const char *cmd)
{
    gprsSerial.puts(cmd);
    gprsSerial.puts("\r\n");
}

int GPRS::init(char *resp_buf, int size_buf)
{
    for (int i = 0; i < 3; i++) {
        send_cmd("AT");
        if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
            return MODEM_RESPONSE_ERROR;
        }
    }

    // Disable modem echo
    send_cmd("ATE0");
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::wakeup(char *resp_buf, int size_buf)
{
    send_cmd("AT");
    send_cmd("AT+CSCLK=0");
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::check_pin(char *resp_buf, int size_buf)
{
    send_cmd("AT+CPIN?");
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "READY")) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::set_pin(const char* pin, char *resp_buf, int size_buf)
{
    char cpin[20];
    sprintf(cpin, "AT+CPIN=\"%s\"", pin);
    send_cmd(cpin);
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::ssl_set_cert(const char *filename, char *resp_buf, int size_buf)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+SSLSETCERT=%s,ABC", filename);
    send_cmd(cmd);
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "+SSLSETCERT: 0")) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::enable_ssl(char *resp_buf, int size_buf)
{
    send_cmd("AT+CIPSSL=1");
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::setup_clock(char *resp_buf, int size_buf)
{
    send_cmd("AT+CNTPCID=1");
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+CNTP=time1.google.com,0");
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+CNTP");
    if (0 != read_resp(resp_buf, size_buf, 6, "+CNTP: 1")) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::setup_bearer(const char* apn, const char* user, const char* pass,
                        char *resp_buf, int size_buf)
{
    // Set the type of Internet connection as GPRS
    char cmd[64];
    send_cmd("AT+SAPBR=3,1,Contype,GPRS");
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }

    // Set the access point name string
    snprintf(cmd, sizeof(cmd), "AT+SAPBR=3,1,APN,\"%s\"", apn);
    send_cmd(cmd);
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }

    // Set the user name for APN
    snprintf(cmd, sizeof(cmd), "AT+SAPBR=3,1,USER,\"%s\"", user);
    send_cmd(cmd);
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }

    // Set the password for APN
    snprintf(cmd, sizeof(cmd), "AT+SAPBR=3,1,PWD,\"%s\"", pass);
    send_cmd(cmd);
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the responses are as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::enable_bearer(char *resp_buf, int size_buf)
{
    send_cmd("AT+SAPBR=1,1"); // Response time can be upto 85 seconds
    read_resp(resp_buf, size_buf, 6, NULL);

    if (NULL != strstr(resp_buf, "OK")) {
        return MODEM_RESPONSE_OK;
    }
    // Returns with ERROR if it's already open or if actually an error
    else if (NULL != strstr(resp_buf, "ERROR")) {
        if (1 == check_bearer_status()) {
            // Bearer already opened. So this is actually not an error.
            return MODEM_RESPONSE_OK;
        }
    }
    // An actual error response received or no response yet
    return MODEM_RESPONSE_ERROR;
}

int GPRS::check_bearer_status(void)
{
    const char delimiters[2] = ","; // Multiple delimiters to separate the string
    char *token = NULL;
    int status = -1;
    char bearer_info[40];

    send_cmd("AT+SAPBR=2,1");
    read_resp(bearer_info, sizeof(bearer_info), DEFAULT_TIMEOUT, NULL);

    if (0 == strlen(bearer_info)) {
        return MODEM_RESPONSE_ERROR; // No response
    }
    else { // Response received
        // Response will be in the format +SAPBR: 1,1,"10.136.76.225"
        // Split the string using the set of delimiters
        token = strtok(bearer_info, delimiters);
        while (token != NULL) {
            for (int i = 0; i < 3; i++) {
                if (i == 1) {
                    sscanf(token, "%d", &status);
                }
                token = strtok(NULL, delimiters);
                if (token == NULL) { // If the end of the list
                    break;
                }
            }
            return status;
        }
    }
    return MODEM_RESPONSE_ERROR; // Invalid response
}

int GPRS::get_active_network(char *resp_buf, int size_buf)
{
    send_cmd("AT+COPS?");
    read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, NULL);
    if ((NULL != strstr(resp_buf, "OK"))) {
        return MODEM_RESPONSE_OK;
    }
    // Invalid or no response
    return MODEM_RESPONSE_ERROR;
}

void GPRS::search_networks(void)
{
    send_cmd("AT+COPS=?");
}

int GPRS::select_network(const char *network, char *resp_buf, int size_buf)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+COPS=4,1,\"%s\"", network);
    send_cmd(cmd);
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the responses are as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::network_registration_gsm(char *resp_buf, int size_buf)
{
    send_cmd("AT+CREG?");
    read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, NULL);
    if ((NULL != strstr(resp_buf, "+CREG: 0,1")) ||
        (NULL != strstr(resp_buf, "+CREG: 0,5"))) {
        return MODEM_RESPONSE_OK;
    }
    // Not registered yet
    return MODEM_RESPONSE_ERROR;
}

int GPRS::network_registration_gprs(char *resp_buf, int size_buf)
{
    send_cmd("AT+CGREG?");
    read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, NULL);
    if ((NULL != strstr(resp_buf, "+CGREG: 0,1")) ||
        (NULL != strstr(resp_buf, "+CGREG: 0,5"))) {
        return MODEM_RESPONSE_OK;
    }
    // Not registered yet
    return MODEM_RESPONSE_ERROR;
}

int GPRS::check_signal_strength(char *resp_buf, int size_buf)
{
    int value = 0;
    send_cmd("AT+CSQ");
    read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, NULL);
    if (0 != strlen(resp_buf)) {
        // Extract the integer value from the received string.
        sscanf(resp_buf, " +CSQ: %d,", &value);
        int rssi = (2*value - 113);
        if ((rssi >= 0) || (rssi < -155)) {
            return MODEM_RESPONSE_ERROR;
        }
        return rssi;
    }
    return MODEM_RESPONSE_ERROR; // If no response
}

uint32_t GPRS::get_time(char *resp_buf, int size_buf)
{
    send_cmd("AT+CCLK?");
    read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, NULL);

    if (0 != strlen(resp_buf)) {
        struct tm timeinfo;
        int timezone;

        // Response format is +CCLK: "20/05/08,11:05:20+00"
        int items = sscanf(resp_buf, " +CCLK: \"%2d/%2d/%2d,%2d:%2d:%2d%3d\" ",
            &timeinfo.tm_year, &timeinfo.tm_mon, &timeinfo.tm_mday,
            &timeinfo.tm_hour, &timeinfo.tm_min, &timeinfo.tm_sec,
            &timezone);

        if (items == 7) {                   // all information found
            timeinfo.tm_year += 100;        // struct tm starts counting years from 1900
            timeinfo.tm_mon -= 1;           // struct tm starts counting months from 0 for January
            // SIM800 provides timezone as multiple of 15 mins but if using NTP, time will
            // always be the specified zone
            uint32_t timestamp = mktime(&timeinfo); // - timezone * 15 * 60;
            return timestamp;
        }
        else {
            return MODEM_RESPONSE_ERROR;
        }
    }
    return MODEM_RESPONSE_ERROR; // If no response
}

int GPRS::attach_gprs(char *resp_buf, int size_buf)
{
    // Attach GPRS
    send_cmd("AT+CGATT=1");
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the responses are as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::enable_get_data_manually(char *resp_buf, int size_buf)
{
    // Startup single IP connection
    send_cmd("AT+CIPMUX=0");
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }

    //Enable getting data from network manually.
    send_cmd("AT+CIPRXGET=1");
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the responses are as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::set_apn(const char* apn, const char* user, const char* pass, char *resp_buf, int size_buf)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CSTT=\"%s\",\"%s\",\"%s\"", apn, user, pass);
    send_cmd(cmd);
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }
    return MODEM_RESPONSE_OK;
}

int GPRS::activate_gprs(char *resp_buf, int size_buf)
{
    send_cmd("AT+CIICR"); // Upto 85 seconds
    read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, NULL);
    if ((NULL != strstr(resp_buf, "OK")) ||
        (NULL != strstr(resp_buf, "ERROR"))) {
        // If the responses are as expected - OK or ERROR received
        return MODEM_RESPONSE_OK;
    }
    // If no response or invalid
    return MODEM_RESPONSE_ERROR;
}

int GPRS::get_ip(char *resp_buf, int size_buf)
{
    send_cmd("AT+CIFSR");
    if (0 == read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "ERROR")) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response gives a valid IP address
    return MODEM_RESPONSE_OK;
}

int GPRS::connect_tcp(const char *domain, const char *port, char *resp_buf, int size_buf)
{
    char cmd[100];
    sprintf(cmd, "AT+CIPSTART=TCP,%s,%s", domain, port);
    send_cmd(cmd);

    read_resp(resp_buf, size_buf, 6, NULL);
    if ((NULL != strstr(resp_buf, "CONNECT OK")) ||
        (NULL != strstr(resp_buf, "ALREADY CONNECT"))) {
        return MODEM_RESPONSE_OK;
    }

    return MODEM_RESPONSE_ERROR; // Invalid
}

int GPRS::close_pdp_context(void)
{
    char buf[30];
    // Close the GPRS PDP context.
    send_cmd("AT+CIPSHUT"); // Can take up to 65 seconds
    read_resp(buf, sizeof(buf), 6, NULL);

    if (NULL != strstr(buf, "SHUT OK")) {
        return MODEM_RESPONSE_OK;
    }
    return MODEM_RESPONSE_ERROR;
}

int GPRS::close_tcp(char *resp_buf, int size_buf)
{
    // closes the TCP connection
    send_cmd("AT+CIPCLOSE=0");
    read_resp(resp_buf, size_buf, 6, NULL);
    if ((NULL != strstr(resp_buf, "CLOSE OK")) ||
        (NULL != strstr(resp_buf, "ERROR"))) { // If TCP not opened previously
        return MODEM_RESPONSE_OK;
    }
    return MODEM_RESPONSE_ERROR; // Invalid
}

int GPRS::close_tcp_quick(char *resp_buf, int size_buf)
{
    // closes the TCP connection quickly
    send_cmd("AT+CIPCLOSE=1");
    read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, NULL);
    if ((NULL != strstr(resp_buf, "CLOSE OK")) ||
        (NULL != strstr(resp_buf, "ERROR"))) { // If TCP not opened previously
        return MODEM_RESPONSE_OK;
    }
    return MODEM_RESPONSE_ERROR; // Invalid
}

int GPRS::detach_gprs(char *resp_buf, int size_buf)
{
    send_cmd("AT+CGATT=0");
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::disable_bearer(char *resp_buf, int size_buf)
{
    send_cmd("AT+SAPBR=0,1"); // Up to 65 seconds
    if (0 != read_resp(resp_buf, size_buf, 6, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

void GPRS::sleep(void)
{
    send_cmd("AT+CSCLK=2");
}

int GPRS::send_tcp_data(unsigned char *data, int len, char *tcp_buf, int size_buf, uint8_t timeout)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d", len);
    send_cmd(cmd);
    if (0 != read_resp(tcp_buf, size_buf, DEFAULT_TIMEOUT, ">")) {
        return MODEM_RESPONSE_ERROR;
    }

    // Send data
    for (int i = 0; i < len; i++) {
        gprsSerial.putc(data[i]);
    }
    // The response could take longer than 7 seconds, it depends on the connection to the server
    read_resp(tcp_buf, size_buf, timeout, NULL);
    if (NULL == strstr(tcp_buf, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

void GPRS::clear_buffer(void)
{
    // For non-buffered serial, even one getc would be enough...
    // Using for-loop to prevent accidental indefinite loop
    for (int i = 0; i < 1000 && gprsSerial.readable(); i++) {
        gprsSerial.getc();
    }
}

int GPRS::check_ssl_cert(const char *filename, int filesize, char *resp_buf, int size_buf)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+FSFLSIZE=%s", filename);
    send_cmd(cmd);

    char resp[64];
    snprintf(resp, sizeof(resp), "FSFLSIZE: %d", filesize);

    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, resp)) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::load_ssl(const char *filename, const char *cert, int filesize, char *resp_buf, int size_buf)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+FSCREATE=%s", filename);
    send_cmd(cmd);

    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }

    snprintf(cmd, sizeof(cmd), "AT+FSWRITE=%s,0,%d,5", filename, filesize);
    send_cmd(cmd);
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, ">")) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd(cert);
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the responses are as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::reset(char *resp_buf, int size_buf)
{
    send_cmd("AT+CFUN=0");
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+CFUN=1,1");
    if (0 != read_resp(resp_buf, size_buf, 5, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the responses are as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::init_sms(char *resp_buf, int size_buf)
{
    send_cmd("AT+CMGF=1");
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+CNMI=2,1,0,0,0");
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the responses are as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::check_new_sms(char *resp_buf, int size_buf)
{
    char *s;
    int sms_location = 0;
    read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, NULL);
    if (0 != strlen(resp_buf)) {
        s = strstr(resp_buf, "+CMTI");
        if (NULL == s) {
            return MODEM_RESPONSE_ERROR; // Invalid response
        }
        sscanf(resp_buf, "+CMTI: %s,%d", s, &sms_location);
        return sms_location;
    }
    return MODEM_RESPONSE_ERROR; // No response
}

int GPRS::get_sms(int index, char* message, int length_message)
{
    // Read a text message from the message storage area at the specified index
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CMGR=%d", index);
    send_cmd(cmd);
    read_resp(message, length_message, DEFAULT_TIMEOUT, NULL);
    return MODEM_RESPONSE_ERROR; // To be changed in future
}

int GPRS::send_get_request(char* url, char *resp_buf, int size_buf)
{
    char cmd[64];
    send_cmd("AT+HTTPINIT");
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+HTTPPARA=\"CID\",1");
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+HTTPSSL=1");
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+HTTPPARA=\"REDIR\",1");
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }

    snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"%s\"", url);
    send_cmd(cmd);
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+HTTPACTION=0");
    if (0 != read_resp(resp_buf, size_buf, 5, "+HTTPACTION")) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+HTTPREAD");

    // If the responses are as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::bt_power_on(char *resp_buf, int size_buf)
{
    send_cmd("AT+BTPOWER=1");
    read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, NULL);
    if ((NULL != strstr(resp_buf, "OK")) ||
        (NULL != strstr(resp_buf, "ERROR"))) {
        return MODEM_RESPONSE_OK; // Connection success
    }
    return MODEM_RESPONSE_ERROR;
}

int GPRS::accept_bt(char *resp_buf, int size_buf)
{
    send_cmd("AT+BTACPT=1");
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }
    return MODEM_RESPONSE_OK;
}

int GPRS::accept_bt_pair(char *resp_buf, int size_buf)
{
    send_cmd("AT+BTPAIR=1,1");
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }
    return MODEM_RESPONSE_OK;
}

int GPRS::send_bt_data(unsigned char *data, int len, char *resp_buf, int size_buf)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+BTSPPSEND=%d", len);
    send_cmd(cmd);

    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, ">")) {
        return MODEM_RESPONSE_ERROR;
    }

    // Send data
    for (int i = 0; i < len; i++) {
        gprsSerial.putc(data[i]);
    }

    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::check_bt_host(const char *host, char *resp_buf, int size_buf)
{
    send_cmd("AT+BTHOST?");
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, host)) {
        return MODEM_RESPONSE_ERROR;
    }
    return MODEM_RESPONSE_OK;
}

int GPRS::change_bt_host(const char *host, char *resp_buf, int size_buf)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+BTHOST=%s", host);
    send_cmd(cmd);
    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }
    return MODEM_RESPONSE_OK;
}

//////// THIS IS THE END OF FUNCTIONS CURRENTLY IN USE ////////////


int GPRS::send_sms(char *number, char *data, char *resp_buf, int size_buf)
{
    char cmd[64];

    while (gprsSerial.readable()) {
        gprsSerial.getc();
    }

    snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"", number);
    send_cmd(cmd);

    if (0 != read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, ">")) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the responses are as expected
    gprsSerial.puts(data);
    gprsSerial.putc((char)0x1a);

    return MODEM_RESPONSE_OK;
}

int GPRS::delete_sms(int index)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+CMGD=%d", index);
    send_cmd(cmd);
    return MODEM_RESPONSE_OK;
}

int GPRS::answer(void)
{
    gprsSerial.printf("ATA");
    return MODEM_RESPONSE_OK;
}

int GPRS::call_up(char *number, char *resp_buf, int size_buf)
{
    send_cmd("AT+COLP=1");

    if (0 != read_resp(resp_buf, size_buf, 5, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the responses are as expected
    gprsSerial.printf("\r\nATD+ %s;", number);
    return MODEM_RESPONSE_OK;
}

bool GPRS::get_location(float *latitude, float *longitude, char *resp_buf, int size_buf)
{
    char *location[10];
    const char s[2] = ",";
    char *token;
    int i = 0;
    send_cmd("AT+CIPGSMLOC=1,1");
    if (0 == read_resp(resp_buf, size_buf, DEFAULT_TIMEOUT, "$$+CIPGSMLOC: ")) {
        token = strtok(resp_buf, s);
        while (token != NULL) {
            location[i] = token;
            token = strtok(NULL, s);
            i++;
        }
        sscanf(location[1], "%f", longitude);
        sscanf(location[2], "%f", latitude);
        return true;
    }
    return false;
}

#endif // MBED

#endif /* UNIT_TEST */
