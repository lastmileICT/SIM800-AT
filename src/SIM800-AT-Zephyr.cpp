/* SIM800_AT firmware
 * Copyright (c) 2016-2021 Connected Energy Technologies Ltd
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
#ifdef CONFIG_SOC_SERIES_STM32L0X
#include "stm32l072xx.h"
#endif

#ifdef __ZEPHYR__
#include <zephyr.h>
#include <drivers/uart.h>
#include <string.h>
#include <time.h>

#define XSTR(x) STR(x)
#define STR(x) #x

#define UART_GSM DT_LABEL(DT_ALIAS(uart_gsm))
#define UART_GSM_BASE_ADDR DT_REG_ADDR(DT_ALIAS(uart_gsm))
const struct device *gsm_dev = device_get_binding(UART_GSM);
static USART_TypeDef *UARTGSM = (USART_TypeDef *)(UART_GSM_BASE_ADDR);

int time_out = DEFAULT_TIMEOUT;
char ack_message[32];
size_t len_ack;
uint32_t time_initial;
uint16_t actual_ack_num_bytes;
volatile size_t current_index = 0;
static volatile bool ack_received = false;

static char *resp_buf = NULL;
static uint32_t sizeof_resp_buf;

int GPRS::ip_rx_data(void)
{
    // The length of output data can not exceed 1460 bytes at a time,
    // based on SIM800 Series_AT Command Manual.
    send_cmd("AT+CIPRXGET=2,1460", DEFAULT_TIMEOUT, NULL);
    k_sleep(K_SECONDS(DEFAULT_TIMEOUT));
    uint16_t tcp_packet_len;
    sscanf((char *)resp_buf, " +CIPRXGET: 2,%hu,0", &tcp_packet_len);
    return tcp_packet_len;
}

void irq_handler(const struct device *dev, void* user_data)
{
    char c;
    bool done = false;
    UARTGSM->CR1 &= ~(USART_CR1_RXNEIE); // disable receive interrupts

    // Timer expired or buffer length reached ? There is always one
    // byte reserved for '\0'.
    if ((current_index == sizeof_resp_buf - 1) \
        || ((k_uptime_get() / 1000) - time_initial) > time_out) {
        // RX interrupts will stay disabled !
        resp_buf[current_index] = '\0';
        return;
    }

    c = resp_buf[current_index] = (char)UARTGSM->RDR;
    //USART1->TDR = c; // Debug aid: print incoming data to the console
    current_index++;

    if (len_ack != 0) { // ACK check required ?
        if (c == ack_message[actual_ack_num_bytes]) {
            actual_ack_num_bytes++;
        } else {
            actual_ack_num_bytes = 0;
        }

        if (actual_ack_num_bytes == len_ack) {
            // Acknowledgement string received
            ack_received = true;
            resp_buf[current_index] = '\0';
            done = true;
        }
    }

    UARTGSM->RQR |= USART_RQR_RXFRQ; // clear RXNE again
    // clear overrun, framing, parity, noise errors
    UARTGSM->ICR |= (USART_ICR_ORECF | USART_ICR_PECF
                    | USART_ICR_FECF | USART_ICR_NCF);
    if (!done) {
        UARTGSM->CR1 |= USART_CR1_RXNEIE; // enable receive interrupts
    }
}

void init_modem(char *buf, uint32_t size_buf)
{
    uart_irq_callback_user_data_set(gsm_dev, irq_handler, NULL);

    // Passing the external buffer handle to SIM800 library
    resp_buf = buf;
    sizeof_resp_buf = size_buf;
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
    len_ack = 0;
    if (ack != NULL) {
        len_ack = strlen(ack);
    }
    if (len_ack && len_ack < sizeof(ack_message) - 1) {
        strncpy(ack_message, ack, len_ack + 1);
    }

    // To handle a special case of early exit from the ISR while using the NULL argument,
    // as described in send_cmd().
    if (ack == NULL) {
        // We are reusing the buffer, so clear it before the reuse
        memset(resp_buf, 0, sizeof_resp_buf);
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

// simple AT -> OK serial comm test
int GPRS::test_uart()
{
    for (int i = 0; i < 2; i++) {
        send_cmd("AT", 1, "OK");
        k_sleep(K_MSEC(200));
        if (ack_received) {
            return MODEM_RESPONSE_OK;
        }
    }
    return MODEM_RESPONSE_ERROR;
}

int GPRS::init(void)
{
    send_cmd("AT", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_MSEC(400));
    send_cmd("AT", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_MSEC(400));
    if (!ack_received) {
        return MODEM_RESPONSE_ERROR;
    }
    // Disable modem echo.
    send_cmd("ATE0", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_MSEC(200));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // Disable clock down scaling
    send_cmd("AT+CSCLK=0", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_MSEC(700));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::wakeup(void)
{
    send_cmd("AT", 1, NULL);
    k_sleep(K_MSEC(200));

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::check_pin(void)
{
    send_cmd("AT+CPIN?", DEFAULT_TIMEOUT, "READY");
    k_sleep(K_MSEC(500));
    if (ack_received == false) {
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
    k_sleep(K_MSEC(200));
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
    k_sleep(K_MSEC(500));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // Set the access point name string
    snprintf(cmd, sizeof(cmd), "AT+SAPBR=3,1,APN,\"%s\"", apn);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    k_sleep(K_MSEC(500));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // Set the user name for APN
    snprintf(cmd, sizeof(cmd), "AT+SAPBR=3,1,USER,\"%s\"", user);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    k_sleep(K_MSEC(500));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // Set the password for APN
    snprintf(cmd, sizeof(cmd), "AT+SAPBR=3,1,PWD,\"%s\"", pass);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    k_sleep(K_MSEC(500));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the responses are as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::enable_bearer(void)
{
    send_cmd("AT+SAPBR=1,1", 85, NULL); // Response time can be upto 85 seconds
    k_sleep(K_SECONDS(3));
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
    k_sleep(K_MSEC(500));
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
    k_sleep(K_MSEC(500));
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
    k_sleep(K_MSEC(300));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    //Enable getting data from network manually.
    send_cmd("AT+CIPRXGET=1", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_MSEC(300));
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
    k_sleep(K_MSEC(200));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    return MODEM_RESPONSE_OK;
}

int GPRS::activate_gprs(void)
{
    send_cmd("AT+CIICR", DEFAULT_TIMEOUT, NULL); // Upto 85 seconds
    k_sleep(K_MSEC(1000));
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
    k_sleep(K_SECONDS(3));
    if ((NULL != strstr(resp_buf, "CONNECT OK")) ||
        (NULL != strstr(resp_buf, "ALREADY CONNECT"))) {
        snprintf(cmd, sizeof(cmd), "AT+CIPQSEND=1");
        send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
        k_sleep(K_MSEC(500));
        return MODEM_RESPONSE_OK;
    }

    return MODEM_RESPONSE_ERROR; // Invalid
}

int GPRS::close_pdp_context(void)
{
    // Close the GPRS PDP context.
    send_cmd("AT+CIPSHUT", 6, NULL);
    k_sleep(K_MSEC(600));
    if (NULL != strstr(resp_buf, "SHUT OK")) {
        return MODEM_RESPONSE_OK;
    }
    return MODEM_RESPONSE_ERROR;
}

int GPRS::close_tcp(void)
{
    // closes the TCP connection
    send_cmd("AT+CIPCLOSE=0", 6, NULL);
    k_sleep(K_MSEC(800));
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
    k_sleep(K_MSEC(400));
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
    k_sleep(K_MSEC(600));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

void GPRS::sleep(void)
{
    send_cmd("AT+CSCLK=2", DEFAULT_TIMEOUT, "OK");
}

void GPRS::powerdown(void)
{
    send_cmd("AT+CPOWD=0", DEFAULT_TIMEOUT, NULL);
}


int GPRS::send_tcp_data(void *data, int len, uint8_t timeout)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d", len);
    send_cmd(cmd, DEFAULT_TIMEOUT, ">");
    k_sleep(K_MSEC(300));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // Send data
    for (int i = 0; i < len; i++) {
        uart_poll_out(gsm_dev, ((char *)data)[i]);
    }

    prepare_for_rx(timeout, NULL);
    k_sleep(K_SECONDS(timeout));

    if (strstr(resp_buf, "SEND OK") == NULL && strstr(resp_buf, "DATA ACCEPT") == NULL) {
        if (strstr(resp_buf, "+CME ERROR") != NULL) {
            // An error related to mobile equipment or network
            return MODEM_CME_ERROR;
        }
        else {
            return MODEM_RESPONSE_ERROR;
        }
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
        // printf("%c", rec_char);
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

int GPRS::delete_file(const char *file_name)
{
    // Delete the existing file in the SIM800 memory
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+FSDEL=C:\\User\\FTP\\%s", file_name);

    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    k_sleep(K_MSEC(400));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::ftp_init(const char *server, const char *user, const char *pw, const char *file_name,
                    const char *file_path)
{
    char cmd[64];

    // Set parameters for FTP session
    send_cmd("AT+FTPCID=1", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_MSEC(100));

    // Server name
    snprintf(cmd, sizeof(cmd), "AT+FTPSERV=\"%s\"", server);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    k_sleep(K_MSEC(100));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // User name
    snprintf(cmd, sizeof(cmd), "AT+FTPUN=\"%s\"", user);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    k_sleep(K_MSEC(100));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    snprintf(cmd, sizeof(cmd), "AT+FTPPW=\"%s\"", pw);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    k_sleep(K_MSEC(100));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    snprintf(cmd, sizeof(cmd), "AT+FTPGETNAME=\"%s\"", file_name);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    k_sleep(K_MSEC(100));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    snprintf(cmd, sizeof(cmd), "AT+FTPGETPATH=\"%s\"", file_path);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    k_sleep(K_MSEC(100));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::ftp_get(const char *server, const char *user, const char *pw, const char *file_name,
                    const char *file_path)
{
    char cmd[64];

    if (ftp_init(server, user, pw, file_name, file_path) != MODEM_RESPONSE_OK) {
        return MODEM_RESPONSE_ERROR;
    }

    snprintf(cmd, sizeof(cmd), "AT+FTPGETTOFS=0,\"%s\"", file_name);
    send_cmd(cmd, 90, NULL); // More than 1 minute for response?
    k_sleep(K_MSEC(400));

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::read_ftp_file(const char *file_name, int length, int offset)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+FSREAD=C:\\User\\FTP\\%s,1,%d,%d", file_name, length, offset);
    send_cmd(cmd, 2, NULL);
    k_sleep(K_SECONDS(1));

    return MODEM_RESPONSE_OK;
}

int GPRS::ftp_get_to_ram(const char *server, const char *user, const char *pw, const char *file_name,
                    const char *file_path)
{
    if (ftp_init(server, user, pw, file_name, file_path) != MODEM_RESPONSE_OK) {
        return MODEM_RESPONSE_ERROR;
    }

    // Open the FTP session
    send_cmd("AT+FTPEXTGET=1", 90, NULL); //Takes more than 1 minute to download
    k_sleep(K_MSEC(500));
    if (NULL == strstr(resp_buf, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }
    // After this, the caller needs to wait until the response "+FTPEXTGET: 1,0" is received,
    // which indicates that the downloading is successful.

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::ftp_get_ram_filesize(void)
{
    int file_size = 0;
    send_cmd("AT+FTPEXTGET?", DEFAULT_TIMEOUT, NULL);
    k_sleep(K_MSEC(400));

    // The response should ideally be like, +FTPEXTGET: 1,108944
    char *ptr = strstr(resp_buf, "+FTPEXTGET: 1,");
    if (ptr == NULL) {
        return MODEM_RESPONSE_ERROR;
    }
    // Success
    sscanf(ptr, "+FTPEXTGET: 1,%d,", &file_size);
    return file_size;
}

int GPRS::ftp_read_from_ram(int length, int offset)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+FTPEXTGET=3,%d,%d", offset, length);
    send_cmd(cmd, 2, NULL);
    k_sleep(K_MSEC(CONFIG_FTP_READ_WAIT));

    return MODEM_RESPONSE_OK;
}

int GPRS::ftp_end_session(void)
{
    send_cmd("AT+FTPEXTGET=0", DEFAULT_TIMEOUT, "OK");
    k_sleep(K_MSEC(400));

    return MODEM_RESPONSE_OK;
}

#endif // ZEPHYR

#endif /* UNIT_TEST */
