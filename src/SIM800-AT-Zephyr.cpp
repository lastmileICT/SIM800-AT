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

#ifdef __ZEPHYR__
#include <zephyr.h>
#include <drivers/uart.h>
#include <string.h>
#include <time.h>

#define UART_GSM DT_LABEL(DT_ALIAS(uart_uext))
struct device *gsm_dev = device_get_binding(UART_GSM);

int time_out = DEFAULT_TIMEOUT;
char ack_message[32];
uint32_t time_initial;
uint16_t actual_ack_num_bytes;
volatile size_t current_index = 0;
static volatile bool ack_received = false;
char resp_buf[BUF_SIZE_DEFAULT]; // The size need to be tuned

int GPRS::request_data(void)
{
    send_cmd("AT+CIPRXGET=2,100", DEFAULT_TIMEOUT, NULL);
    k_sleep(K_SECONDS(DEFAULT_TIMEOUT));
    return MODEM_RESPONSE_OK;
}

void read_resp(void* user_data)
{
    uint8_t c;
    bool exit_flag = false;

    if (!uart_irq_update(gsm_dev)) {
        return;
    }

    if (((k_uptime_get() / 1000) - time_initial) <= time_out) { // timer not expired yet
        while (uart_irq_rx_ready(gsm_dev)) {
            uart_fifo_read(gsm_dev, &c, 1);
            // Fill the buffer up to all but 1 character (the last character is reserved for '\0')
            // Characters beyond the size of the buffer are dropped.
            if (current_index < (BUF_SIZE_DEFAULT - 1)) {
                // There is always one last character left for the '\0'
                resp_buf[current_index] = c;

                if (strlen(ack_message) != 0) { //Decide if an acknowledgement check required
                    // Checks if the acknowledgement string is received correctly
                    if (resp_buf[current_index] == ack_message[actual_ack_num_bytes]) {
                        actual_ack_num_bytes++;
                    }
                    else {
                        actual_ack_num_bytes = 0; // Reset the counter
                    }

                    if (actual_ack_num_bytes == strlen(ack_message)) { // Exit condition
                        // Acknowledgement string received
                        current_index++;
                        ack_received = true;
                        exit_flag = true;
                        goto exit;
                    }
                }
                current_index++;
            }
            else { // Buffer exceeded
                exit_flag = true;
                goto exit;
            }
        } // While data receive
    }
    else { // If timed out
        exit_flag = true;
    }
exit:
    if (exit_flag) {
        uart_irq_rx_disable(gsm_dev);
        resp_buf[current_index] = '\0';
    }
    return;
}

void init_modem(void)
{
    uart_irq_callback_user_data_set(gsm_dev, read_resp, NULL);
}

void GPRS::send_cmd(const char *cmd, int timeout, const char *ack)
{
    for (int i = 0; i < (int)strlen(cmd); i++) {
        uart_poll_out(gsm_dev, cmd[i]);
    }
    uart_poll_out(gsm_dev, '\r');
    uart_poll_out(gsm_dev, '\n');

    // Prepare for processing in the receive interrupt
    prepare_for_rx(timeout, ack);
}

// Call this only after sending a command to the SIM800.
// Resets the necessary variables and enables the Rx interrupt
void GPRS::prepare_for_rx(int timeout, const char *ack)
{
    ack_message[0] = '\0';
    // Make the ack_message configurable
    if (strlen(ack) > sizeof(ack_message) - 1) {
        printf("Invalid string length for ACK \n");
    }
    else {
        if (ack != NULL) {
            strncpy(ack_message, ack, sizeof(ack_message));
        }
        // Else, the ack_message will be empty anyway.
    }

    // To handle a special case of early exit from the ISR while using the NULL argument,
    // as described in send_cmd().
    if (ack == NULL) {
        // We are reusing the buffer, so clear it before the reuse
        memset(resp_buf, 0, sizeof(resp_buf));
    }

    ack_received = false;
    time_out = timeout;
    time_initial = k_uptime_get() / 1000; // Set the initial value for timeout check
    current_index = 0;  // Do this here instead of doing in the ISR. This to handle a special case,
                        // when the modem response comes faster and NULL is used as an Ack string.
    resp_buf[0] = '\0';
    actual_ack_num_bytes = 0;
    uart_irq_rx_enable(gsm_dev);
}

int GPRS::init(void)
{
    for (int i = 0; i < 3; i++) {
        send_cmd("AT", DEFAULT_TIMEOUT, "OK");
        k_sleep(K_SECONDS(1));
        if (ack_received == false) {
            printf("\r\nFailed init_resp\r\n");
            return MODEM_RESPONSE_ERROR;
        }
    }

    // Disable modem echo.
    send_cmd("ATE0", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::wakeup(void)
{
    send_cmd("AT", 1, NULL);
    k_sleep(K_SECONDS(1));
    send_cmd("AT+CSCLK=0", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        printf("\r\nFailed wakeup\r\n");
        return MODEM_RESPONSE_ERROR;
    }

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::check_pin(void)
{
    send_cmd("AT+CPIN?", DEFAULT_TIMEOUT, "READY");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        printf("\r\nFailed check_pin\r\n");
        return MODEM_RESPONSE_ERROR;
    }
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::set_pin(const char* pin)
{
    char cpin[20];
    sprintf(cpin, "AT+CPIN=\"%s\"", pin);
    send_cmd(cpin, DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::check_ssl_cert(const char *filename, int filesize)
{
    char cmd[64];
    char resp[64];
    snprintf(cmd, sizeof(cmd), "AT+FSFLSIZE=%s", filename);
    snprintf(resp, sizeof(resp), "FSFLSIZE: %d", filesize);
    send_cmd(cmd, DEFAULT_TIMEOUT, resp);
    k_sleep(K_SECONDS(1));

    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::load_ssl(const char *filename, const char *cert, int filesize)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+FSCREATE=%s", filename);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    snprintf(cmd, sizeof(cmd), "AT+FSWRITE=%s,0,%d,5", filename, filesize);
    send_cmd(cmd, DEFAULT_TIMEOUT, ">");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd(cert, DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the responses are as expected
    return MODEM_RESPONSE_OK;
}


int GPRS::ssl_set_cert(const char *filename)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+SSLSETCERT=%s,ABC", filename);
    send_cmd(cmd, DEFAULT_TIMEOUT, "+SSLSETCERT: 0");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::enable_ssl(void)
{
    send_cmd("AT+CIPSSL=1", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::setup_clock(void)
{
    send_cmd("AT+CNTPCID=1", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+CNTP=time1.google.com,0", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+CNTP", 6, "+CNTP: 1");
    k_sleep(K_SECONDS(3));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::setup_bearer(const char* apn, const char* user, const char* pass)
{
    // Set the type of Internet connection as GPRS
    char cmd[64];
    send_cmd("AT+SAPBR=3,1,Contype,GPRS", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // Set the access point name string
    snprintf(cmd, sizeof(cmd), "AT+SAPBR=3,1,APN,\"%s\"", apn);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // Set the user name for APN
    snprintf(cmd, sizeof(cmd), "AT+SAPBR=3,1,USER,\"%s\"", user);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // Set the password for APN
    snprintf(cmd, sizeof(cmd), "AT+SAPBR=3,1,PWD,\"%s\"", pass);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the responses are as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::enable_bearer(void)
{
    send_cmd("AT+SAPBR=1,1", 85, NULL); // Response time can be upto 85 seconds
    k_sleep(K_SECONDS(2));
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

    send_cmd("AT+SAPBR=2,1", DEFAULT_TIMEOUT, NULL);
    k_sleep(K_SECONDS(DEFAULT_TIMEOUT));
    if (0 == strlen(resp_buf)) {
        return MODEM_RESPONSE_ERROR; // No response
    }
    else { // Response received
        // Response will be in the format +SAPBR: 1,1,"10.136.76.225"
        // Split the string using the set of delimiters
        token = strtok(resp_buf, delimiters);
        while (token != NULL) {
            for (int i = 0; i < 3; i++) {
                if (i == 1) {
                    sscanf(token, "%d", &status);
                    printf("Bearer status: %d\n", status);
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

int GPRS::get_active_network(void)
{
    send_cmd("AT+COPS?", DEFAULT_TIMEOUT, NULL);
    k_sleep(K_SECONDS(1));
    if ((NULL != strstr(resp_buf, "OK"))) {
        return MODEM_RESPONSE_OK;
    }
    // Invalid or no response
    return MODEM_RESPONSE_ERROR;
}

void GPRS::search_networks(void)
{
    send_cmd("AT+COPS=?", 120, NULL);
}

int GPRS::select_network(const char *network)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+COPS=4,1,\"%s\"", network);
    send_cmd(cmd, 120, "OK");
    k_sleep(K_SECONDS(DEFAULT_TIMEOUT));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the responses are as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::network_registration_gsm(void)
{
    send_cmd("AT+CREG?", DEFAULT_TIMEOUT, NULL);
    k_sleep(K_SECONDS(DEFAULT_TIMEOUT));
    if ((NULL != strstr(resp_buf, "+CREG: 0,1")) ||
        (NULL != strstr(resp_buf, "+CREG: 0,5"))) {
        return MODEM_RESPONSE_OK;
    }
    // Not registered yet
    return MODEM_RESPONSE_ERROR;
}

int GPRS::network_registration_gprs(void)
{
    send_cmd("AT+CGREG?", DEFAULT_TIMEOUT, NULL);
    k_sleep(K_SECONDS(DEFAULT_TIMEOUT));
    if ((NULL != strstr(resp_buf, "+CGREG: 0,1")) ||
        (NULL != strstr(resp_buf, "+CGREG: 0,5"))) {
        return MODEM_RESPONSE_OK;
    }
    // Not registered yet
    return MODEM_RESPONSE_ERROR;
}

int GPRS::check_signal_strength(void)
{
    int value = 0;
    send_cmd("AT+CSQ", DEFAULT_TIMEOUT, NULL);
    k_sleep(K_SECONDS(DEFAULT_TIMEOUT));
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

uint32_t GPRS::get_time(void)
{
    send_cmd("AT+CCLK?", DEFAULT_TIMEOUT, NULL);
    k_sleep(K_SECONDS(DEFAULT_TIMEOUT));
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

int GPRS::attach_gprs(void)
{
    // Attach GPRS
    send_cmd("AT+CGATT=1", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the responses are as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::enable_get_data_manually(void)
{
    // Startup single IP connection
    send_cmd("AT+CIPMUX=0", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    //Enable getting data from network manually.
    send_cmd("AT+CIPRXGET=1", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the responses are as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::set_apn(const char* apn, const char* user, const char* pass)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CSTT=\"%s\",\"%s\",\"%s\"", apn, user, pass);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    return MODEM_RESPONSE_OK;
}

int GPRS::activate_gprs(void)
{
    send_cmd("AT+CIICR", DEFAULT_TIMEOUT, NULL); // Upto 85 seconds
    k_sleep(K_SECONDS(DEFAULT_TIMEOUT));
    if ((NULL != strstr(resp_buf, "OK")) ||
        (NULL != strstr(resp_buf, "ERROR"))) {
        // If the responses are as expected - OK or ERROR received
        return MODEM_RESPONSE_OK;
    }
    // If no response or invalid
    return MODEM_RESPONSE_ERROR;
}

int GPRS::get_ip(void)
{
    send_cmd("AT+CIFSR", DEFAULT_TIMEOUT, "ERROR");
    k_sleep(K_SECONDS(1));
    if (ack_received == true) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response gives a valid IP address
    return MODEM_RESPONSE_OK;
}

int GPRS::connect_tcp(const char *domain, const char *port)
{
    char cmd[100];
    sprintf(cmd, "AT+CIPSTART=TCP,%s,%s", domain, port);
    send_cmd(cmd, 6, NULL);
    k_sleep(K_SECONDS(DEFAULT_TIMEOUT));
    if ((NULL != strstr(resp_buf, "CONNECT OK")) ||
        (NULL != strstr(resp_buf, "ALREADY CONNECT"))) {
        return MODEM_RESPONSE_OK;
    }

    return MODEM_RESPONSE_ERROR; // Invalid
}

int GPRS::close_pdp_context(void)
{
    // Close the GPRS PDP context.
    send_cmd("AT+CIPSHUT", 6, NULL);
    k_sleep(K_SECONDS(DEFAULT_TIMEOUT));
    if (NULL != strstr(resp_buf, "SHUT OK")) {
        return MODEM_RESPONSE_OK;
    }
    return MODEM_RESPONSE_ERROR;
}

int GPRS::close_tcp(void)
{
    // closes the TCP connection
    send_cmd("AT+CIPCLOSE=0", 6, NULL);
    k_sleep(K_SECONDS(DEFAULT_TIMEOUT));
    if ((NULL != strstr(resp_buf, "CLOSE OK")) ||
        (NULL != strstr(resp_buf, "ERROR"))) { // If TCP not opened previously
        return MODEM_RESPONSE_OK;
    }
    return MODEM_RESPONSE_ERROR; // Invalid
}

// int GPRS::close_tcp_quick(char *resp_buf, int size_buf)
int GPRS::close_tcp_quick(void)
{
    // closes the TCP connection quickly
    send_cmd("AT+CIPCLOSE=1", DEFAULT_TIMEOUT, NULL);
    k_sleep(K_SECONDS(DEFAULT_TIMEOUT));
    if ((NULL != strstr(resp_buf, "CLOSE OK")) ||
        (NULL != strstr(resp_buf, "ERROR"))) { // If TCP not opened previously
        return MODEM_RESPONSE_OK;
    }
    return MODEM_RESPONSE_ERROR; // Invalid
}

int GPRS::detach_gprs(void)
{
    send_cmd("AT+CGATT=0", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::disable_bearer(void)
{
    send_cmd("AT+SAPBR=0,1", 6, "OK"); // Up to 65 seconds
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

void GPRS::sleep(void)
{
    send_cmd("AT+CSCLK=2", DEFAULT_TIMEOUT, "OK");
    // k_sleep(K_SECONDS(1));
}

int GPRS::send_tcp_data(unsigned char *data, int len, uint8_t timeout)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d", len);
    send_cmd(cmd, DEFAULT_TIMEOUT, ">");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // Send data
    for (int i = 0; i < len; i++) {
        uart_poll_out(gsm_dev, data[i]);
    }

    prepare_for_rx(timeout, NULL);
    k_sleep(K_SECONDS(timeout));
    // The response could take longer than 7 seconds, it depends on the connection to the server
    if (NULL == strstr(resp_buf, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

void GPRS::clear_buffer(void)
{
    // For non-buffered serial, even one getc would be enough...
    // Using for-loop to prevent accidental indefinite loop
    unsigned char rec_char;
    for (int i = 0; i < 1000 && (uart_poll_in(gsm_dev, &rec_char) == 0); i++) {;}
}

int GPRS::reset(void)
{
    send_cmd("AT+CFUN=0", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+CFUN=1,1", 5, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the responses are as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::init_sms(void)
{
    send_cmd("AT+CMGF=1", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+CNMI=2,1,0,0,0", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the responses are as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::check_new_sms(void)
{
    char *s;
    int sms_location = 0;

    // Prepare for reception
    prepare_for_rx(DEFAULT_TIMEOUT, NULL);
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

int GPRS::get_sms(int index)
{
    // Read a text message from the message storage area at the specified index
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CMGR=%d", index);
    send_cmd(cmd, DEFAULT_TIMEOUT, NULL);
    k_sleep(K_SECONDS(1));
    resp_buf[current_index] = '\0';
    return MODEM_RESPONSE_ERROR; // To be changed in future
}

int GPRS::send_get_request(char* url)
{
    char cmd[64];
    send_cmd("AT+HTTPINIT", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+HTTPPARA=\"CID\",1", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+HTTPSSL=1", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+HTTPPARA=\"REDIR\",1", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"%s\"", url);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+HTTPACTION=0", 5, "+HTTPACTION");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+HTTPREAD", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));

    // If the responses are as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::bt_power_on(void)
{
    send_cmd("AT+BTPOWER=1", DEFAULT_TIMEOUT, NULL);
    k_sleep(K_SECONDS(1));
    if ((NULL != strstr(resp_buf, "OK")) ||
        (NULL != strstr(resp_buf, "ERROR"))) {
        return MODEM_RESPONSE_OK; // Connection success
    }
    return MODEM_RESPONSE_ERROR;
}

int GPRS::accept_bt(void)
{
    send_cmd("AT+BTACPT=1", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    return MODEM_RESPONSE_OK;
}

int GPRS::accept_bt_pair(void)
{
    send_cmd("AT+BTPAIR=1,1", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    return MODEM_RESPONSE_OK;
}

int GPRS::send_bt_data(unsigned char *data, int len)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+BTSPPSEND=%d", len);
    send_cmd(cmd, DEFAULT_TIMEOUT, ">");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // Send data
    for (int i = 0; i < len; i++) {
        uart_poll_out(gsm_dev, data[i]);
    }

    prepare_for_rx(DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::check_bt_host(const char *host)
{
    send_cmd("AT+BTHOST?", DEFAULT_TIMEOUT, host);
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    return MODEM_RESPONSE_OK;
}

int GPRS::change_bt_host(const char *host)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+BTHOST=%s", host);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    return MODEM_RESPONSE_OK;
}

//////// THIS IS THE END OF FUNCTIONS CURRENTLY IN USE ////////////

int GPRS::send_sms(char *number, char *data)
{
    char cmd[64];

    unsigned char rec_char;
    if (uart_poll_in(gsm_dev, &rec_char) == 0) {
        printf("%c", rec_char);
    }

    snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"", number);
    send_cmd(cmd, DEFAULT_TIMEOUT, ">");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the responses are as expected
    for (int i = 0; i < (int)strlen(data); i++) {
        uart_poll_out(gsm_dev, data[i]);
    }
    uart_poll_out(gsm_dev, (char)0x1a);

    return MODEM_RESPONSE_OK;
}

int GPRS::delete_sms(int index)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+CMGD=%d", index);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    k_sleep(K_SECONDS(1));
    return MODEM_RESPONSE_OK;
}

int GPRS::answer(void)
{
    return MODEM_RESPONSE_OK;
}

int GPRS::call_up(char *number)
{
    send_cmd("AT+COLP=1", 5, "OK");
    k_sleep(K_SECONDS(1));

    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the responses are as expected
    return MODEM_RESPONSE_OK;
}

bool GPRS::get_location(float *latitude, float *longitude)
{
    char *location[10];
    const char s[2] = ",";
    char *token;
    int i = 0;
    send_cmd("AT+CIPGSMLOC=1,1", DEFAULT_TIMEOUT, "$$+CIPGSMLOC: ");
    k_sleep(K_SECONDS(1));
    if (ack_received == false) {
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

#endif // ZEPHYR

#endif /* UNIT_TEST */
