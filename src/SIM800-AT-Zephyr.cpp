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

#include <zephyr.h>
#include <drivers/uart.h>
#include <string.h>
#include <time.h>
#include <inttypes.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(sim800, CONFIG_GSM_LOG_LEVEL);

#define UART_GSM DT_LABEL(DT_ALIAS(uart_gsm))
#define UART_GSM_BASE_ADDR DT_REG_ADDR(DT_ALIAS(uart_gsm))
#define UART_GSM_SPEED DT_PROP(DT_ALIAS(uart_gsm), current_speed)
#ifndef GSM_MAX_RX_BUF
#define GSM_MAX_RX_BUF 150
#endif
// The delay below has to be scaled with the flash page size in the
// MCU, which in general is the largest RX buffer we're going to
// get. General formula for wait times in milliseconds would be:
// `delay_per_buffer = 1.44 * (sizeof(buffer) * 8000 / serial_bps)`,
// where the 1.44 = 36/25 term is a safety offset ratio.
#define FTP_READ_WAIT int((36 * GSM_MAX_RX_BUF * 8000 / (31 * UART_GSM_SPEED)) + 30)

int GPRS::ip_rx_data(void)
{
    char cmd[19];
    uint16_t tcp_packet_len = 0;
    uint16_t tcp_avail = 65535;

    /* Test for TCP data length still increasing */
    snprintf(cmd, sizeof(cmd), "AT+CIPRXGET=4");

    uint8_t retries = 0;
    while((tcp_packet_len != tcp_avail) && (retries++ < 10)) {
        send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
        // ACK string position varies between 2 and three characters
        // with mixed whitespace so we can't sscanf for it properly.
        char *ack_pos = strstr(resp_buf, "+CIPRXGET:");
        if (NULL != ack_pos) {
            tcp_packet_len = tcp_avail;
            sscanf(ack_pos, "+CIPRXGET: 4,%hu", &tcp_avail);
        }
        k_sleep(K_MSEC(150));
    }
    if (tcp_packet_len == 0) {
        return 0;
    }

    // The length of output data can not exceed 1460 bytes at a time,
    // based on SIM800 Series_AT Command Manual.
    // We assume the worst case transfer speed of 9.6kbps, equivalent
    // to ~1byte/ms. We also append a margin of 55ms.
    if (tcp_packet_len > 1460) {
        tcp_packet_len = 1460;
    }
    snprintf(cmd, sizeof(cmd), "AT+CIPRXGET=2,%d", tcp_packet_len);
    int timeout = tcp_packet_len + 55;
    send_cmd(cmd, timeout, NULL);

    size_t wasted = strip_modem_response(tcp_packet_len);
    if (tcp_packet_len > (resp_buf_len - wasted)) {
        tcp_packet_len = resp_buf_len - wasted;
    }
    return tcp_packet_len;
}

// Strips the modem response and moves the following count bytes to the
// start of the response buffer. Returns count bytes stripped.
size_t GPRS::strip_modem_response(size_t count)
{
    // The modem response is enclosed between two CR-LF
    // sequences. Look for the second occurence of CR LF starting from
    // position 3 in the buffer.
    for (size_t pos = 2; pos <= resp_buf_len - 2; pos++) {
        if (resp_buf[pos] == 0x0D && resp_buf[pos + 1] == 0x0A) {
            if ((count + pos) > resp_buf_len) {
                count = resp_buf_len - pos;
            }
            memmove(resp_buf, resp_buf + pos + 2, count);
            return pos + 2;
        }
    }
    return 0;
}

void irq_handler(const struct device *dev, void* user_data)
{
    char c;
    bool done = false;
    GPRS *modem = (GPRS *)user_data;

    modem->UARTGSM->CR1 &= ~(USART_CR1_RXNEIE); // disable receive interrupts

    // Timer expired or buffer length reached ? There is always one
    // byte reserved for '\0'.
    if ((modem->current_index == modem->resp_buf_len - 1) \
        || (k_uptime_get() - modem->time_initial) > modem->time_out) {
        // RX interrupts will stay disabled !
        modem->resp_buf[modem->current_index] = '\0';
        return;
    }

    c = modem->resp_buf[modem->current_index] = (char)(modem->UARTGSM->RDR);
    //USART1->TDR = c; // Debug aid: print incoming data to the console
    modem->current_index++;

    if (modem->len_ack != 0) { // ACK check required ?
        if (c == modem->ack_message[modem->actual_ack_num_bytes]) {
            modem->actual_ack_num_bytes++;
        } else {
            modem->actual_ack_num_bytes = 0;
        }

        if (modem->actual_ack_num_bytes == modem->len_ack) {
            // Acknowledgement string received
            modem->ack_received = true;
            modem->resp_buf[modem->current_index] = '\0';
            done = true;
        }
    }

    modem->UARTGSM->RQR |= USART_RQR_RXFRQ; // clear RXNE again
    // clear overrun, framing, parity, noise errors
    modem->UARTGSM->ICR |= (USART_ICR_ORECF | USART_ICR_PECF
                    | USART_ICR_FECF | USART_ICR_NCF);
    if (!done) {
        modem->UARTGSM->CR1 |= USART_CR1_RXNEIE; // enable receive interrupts
    }
}

GPRS::GPRS(uint8_t *rx_buf, size_t rx_buf_size,
            void (*_feed_watchdog)(int), int* _wdt_channel)
            : feed_watchdog{_feed_watchdog}, wdt_channel{_wdt_channel}

{
    UARTGSM = (USART_TypeDef *)(UART_GSM_BASE_ADDR);
    gsm_dev = device_get_binding(UART_GSM);
    uart_irq_callback_user_data_set(gsm_dev, irq_handler, this);
    this->set_rx_buf(rx_buf, rx_buf_size);
}

void GPRS::set_rx_buf(uint8_t *buf, size_t len)
{
    // TODO: check no RX in progress
    resp_buf = (char *)buf;
    resp_buf_len = len;
}

void GPRS::send_cmd(const char *cmd, size_t timeout, const char *ack,
                    bool no_wait /*=false*/)
{
    for (int i = 0; i < (int)strlen(cmd); i++) {
        uart_poll_out(gsm_dev, cmd[i]);
    }
    uart_poll_out(gsm_dev, '\r');
    uart_poll_out(gsm_dev, '\n');
    // Prepare for processing in the receive interrupt
    prepare_for_rx(timeout, ack);

    // Some commands can specify a timeout but don't want to wait for
    // the ack
    if (!no_wait) {
        // Sleep in 20ms slices until we get the ack back.
        int wait_count = timeout / 20;
        while (wait_count--) {
            if(wdt_channel != NULL) {
                feed_watchdog(*wdt_channel);
            }
            k_sleep(K_MSEC(20));
            if (ack_received){
                break;
            }
        }
    }
}

// Call this only after sending a command to the SIM800.
// Resets the necessary variables and enables the Rx interrupt
void GPRS::prepare_for_rx(size_t timeout, const char *ack)
{
    uart_irq_rx_disable(gsm_dev);
    ack_message[0] = '\0';
    // Make the ack_message configurable
    len_ack = 0;
    if (ack != NULL) {
        len_ack = strlen(ack);
    }
    if (len_ack && len_ack < sizeof(ack_message) - 1) {
        strncpy(ack_message, ack, len_ack + 1);
    }
    memset(resp_buf, 0, resp_buf_len);

    ack_received = false;
    time_out = timeout;
    time_initial = k_uptime_get(); // Set the initial value for timeout check
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
        send_cmd("AT", DEFAULT_TIMEOUT, "OK");
        if (ack_received) {
            return MODEM_RESPONSE_OK;
        }
    }
    return MODEM_RESPONSE_ERROR;
}

int GPRS::init(void)
{
    // On higher baud rates, we might need to bang AT a number of
    // times, to train the modem's autobad feature.
    int i = 6;
    while(i--) {
        send_cmd("AT", 300, "OK");
        if (ack_received) {
            break;
        }
    }
    if (!ack_received) {
        return MODEM_RESPONSE_ERROR;
    }
    // Disable modem echo.
    send_cmd("ATE0", DEFAULT_TIMEOUT, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // Disable clock down scaling. Might not work if downscaling not
    // set, so we don't consider missing ACK an error.
    send_cmd("AT+CSCLK=0", DEFAULT_TIMEOUT, "OK");

    // Try to use the maximum GPRS class (12) for SIM800, in which 4 TX
    // slots and 4 RX slots are being used. We don't mind errors here.
    send_cmd("AT+CGMSCLASS=12", DEFAULT_TIMEOUT, "OK");

    return MODEM_RESPONSE_OK;
}

int GPRS::wakeup(void)
{
    send_cmd("AT", DEFAULT_TIMEOUT, NULL);
    return MODEM_RESPONSE_OK;
}

int GPRS::check_pin(void)
{
    send_cmd("AT+CPIN?", 3000, "PIN: READY");
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
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    snprintf(cmd, sizeof(cmd), "AT+FSWRITE=%s,0,%d,5", filename, filesize);
    send_cmd(cmd, DEFAULT_TIMEOUT, ">");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd(cert, DEFAULT_TIMEOUT, "OK");
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
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::enable_ssl(void)
{
    send_cmd("AT+CIPSSL=1", DEFAULT_TIMEOUT, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::disable_ssl(void)
{
    send_cmd("AT+CIPSSL=0", DEFAULT_TIMEOUT, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::setup_clock(void)
{
    send_cmd("AT+CNTPCID=1", DEFAULT_TIMEOUT, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+CNTP=time1.google.com,0", DEFAULT_TIMEOUT, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+CNTP", 1500, "+CNTP: 1");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::setup_bearer(const char* apn, const char* user, const char* pass)
{
    // Set the type of Internet connection as GPRS
    char cmd[50];
    send_cmd("AT+SAPBR=3,1,Contype,GPRS", DEFAULT_TIMEOUT, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    // Set the access point name string
    snprintf(cmd, sizeof(cmd), "AT+SAPBR=3,1,APN,\"%s\"", apn);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // Set the user name for APN
    snprintf(cmd, sizeof(cmd), "AT+SAPBR=3,1,USER,\"%s\"", user);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // Set the password for APN
    snprintf(cmd, sizeof(cmd), "AT+SAPBR=3,1,PWD,\"%s\"", pass);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the responses are as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::enable_bearer(void)
{
    send_cmd("AT+SAPBR=1,1", 85000, "OK"); // Response time can be upto 85 seconds
    if (ack_received) {
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
    if ((NULL != strstr(resp_buf, "OK"))) {
        return MODEM_RESPONSE_OK;
    }
    // Invalid or no response
    return MODEM_RESPONSE_ERROR;
}

int GPRS::search_networks(void)
{
    send_cmd("AT+COPS=?", 45000, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    return MODEM_RESPONSE_OK;
}

int GPRS::select_network(const char *network)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+COPS=4,0,\"%s\"", network);
    send_cmd(cmd, 120000, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the responses are as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::network_registration_gsm(void)
{
    send_cmd("AT+CREG?", DEFAULT_TIMEOUT, NULL);
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
    send_cmd("AT+CGATT=1", 1000, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the responses are as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::enable_get_data_manually(void)
{
    // CIPMUX=0 -> set single IP connection
    // CIPRXGET=1 -> poll for RX data.
    send_cmd("AT+CIPMUX=0;+CIPRXGET=1", DEFAULT_TIMEOUT * 10, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    return MODEM_RESPONSE_OK;
}

int GPRS::set_apn(const char* apn, const char* user, const char* pass)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CSTT=\"%s\",\"%s\",\"%s\"", apn, user, pass);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    return MODEM_RESPONSE_OK;
}

int GPRS::activate_gprs(void)
{
    send_cmd("AT+CIICR", 1000, NULL); // Upto 85 seconds
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
    // The AT command below returns either the IP address as a string
    // or ERROR if IP is not set. It's easier to search for ERROR than
    // to sscanf for the ip addresses components.
    send_cmd("AT+CIFSR", DEFAULT_TIMEOUT, "ERROR");
    if (ack_received == true) {
        return MODEM_RESPONSE_ERROR;
    }
    return MODEM_RESPONSE_OK;
    // Alternative implementation for actually fetching the IP addres.
    /*
    uint16_t ip[4];
    int c = sscanf(resp_buf, "%3hu.%3hu.%3hu.%3hu", &ip[0], &ip[1], &ip[2], &ip[3]);
    // Newlib does not support hhu
    //uint8_t ip[4];
    //int c = sscanf(resp_buf, "%hhu.%hhu.%hhu.%hhu", &ip[0], &ip[1], &ip[2], &ip[3]);
    if (c == 4) {
        return MODEM_RESPONSE_OK;
    }
    return MODEM_RESPONSE_ERROR;
    */
}

int GPRS::connect_tcp(const char *domain, const char *port)
{
    char cmd[100];
    this->tcp_send_len = 0;
    sprintf(cmd, "AT+CIPSTART=TCP,%s,%s", domain, port);
    send_cmd(cmd, 5000, "CONNECT OK");
    if ((NULL != strstr(resp_buf, "CONNECT OK"))) {
#ifdef CONFIG_SIM800_TCP_QUICKSEND
        // If configured, set the SIM800 to support TCP quicksend - in
        // which the data stream length is not declared upfront, but
        // ends with Ctrl-Z.
        snprintf(cmd, sizeof(cmd), "AT+CIPQSEND=1");
        send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
#endif
        return MODEM_RESPONSE_OK;
    }
    return MODEM_RESPONSE_ERROR; // Invalid
}

int GPRS::close_pdp_context(void)
{
    // Close the GPRS PDP context.
    send_cmd("AT+CIPSHUT", 65000, "SHUT OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::close_tcp(void)
{
    this->tcp_send_len = 0;
    send_cmd("AT+CIPCLOSE=0", DEFAULT_TIMEOUT, NULL);
    if ((NULL != strstr(resp_buf, "CLOSE OK")) ||
        (NULL != strstr(resp_buf, "ERROR"))) { // If TCP not opened previously
        return MODEM_RESPONSE_OK;
    }
    return MODEM_RESPONSE_ERROR; // Invalid
}

int GPRS::close_tcp_quick(void)
{
    // closes the TCP connection quickly
    this->tcp_send_len = 0;
    send_cmd("AT+CIPCLOSE=1", DEFAULT_TIMEOUT, NULL);
    if ((NULL != strstr(resp_buf, "CLOSE OK")) ||
        (NULL != strstr(resp_buf, "ERROR"))) { // If TCP not opened previously
        return MODEM_RESPONSE_OK;
    }
    return MODEM_RESPONSE_ERROR; // Invalid
}

int GPRS::detach_gprs(void)
{
    send_cmd("AT+CGATT=0", DEFAULT_TIMEOUT, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::disable_bearer(void)
{
    send_cmd("AT+SAPBR=0,1", 600, "OK"); // Up to 65 seconds
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

void GPRS::sleep(void)
{
    send_cmd("AT+CSCLK=2", DEFAULT_TIMEOUT, "OK", true);
}

void GPRS::powerdown(void)
{
    send_cmd("AT+CPOWD=0", DEFAULT_TIMEOUT, NULL, true);
}

int GPRS::send_tcp_data(const void *data, size_t len)
{
    char cmd[64];
    this->tcp_send_len += len;

#ifdef CONFIG_SIM800_TCP_QUICKSEND
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND");
#else
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d", len);
#endif
    send_cmd(cmd, 900, ">");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // Send data
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(gsm_dev, ((char *)data)[i]);
    }
#ifdef CONFIG_SIM800_TCP_QUICKSEND
    // Send Ctrl-Z, if using quicksend
    k_sleep(K_MSEC(10));
    uart_poll_out(gsm_dev, '\x1A');
    prepare_for_rx(3000, "DATA ACCEPT");
#else
    prepare_for_rx(3000, "SEND OK");
#endif

    // Sleep in 20ms slices until we get the ack back.
    int wait_count = 3000 / 20;
    while (wait_count--) {
        k_sleep(K_MSEC(20));
        if (ack_received){
            break;
        }
    }

    if (!ack_received) {
        if (strstr(resp_buf, "+CME ERROR") != NULL) {
            // An error related to mobile equipment or network
            return MODEM_CME_ERROR;
        }
        else {
            return MODEM_RESPONSE_ERROR;
        }
    }

#ifndef CONFIG_SIM800_ACK_TCP_TX
    return MODEM_RESPONSE_OK;
#else
    snprintf(cmd, sizeof(cmd), "AT+CIPACK");
    uint8_t iter = 2;
    size_t ack_len = 0, reported_total_len = 0;
    bool all_ack = false;
    char *ack_pos = NULL;
    while(iter--) {
        send_cmd(cmd, DEFAULT_TIMEOUT, NULL);
        // ACK string position varies between 2 and three characters
        // with mixed whitespace so we can't sscanf for it properly.
        ack_pos = strstr(resp_buf, "+CIPACK:");
        if (NULL != ack_pos) {
            sscanf(ack_pos, "+CIPACK: %d,%d ", &ack_len, &reported_total_len);
        }
#ifndef CONFIG_SIM800_TCP_QUICKSEND
        if (ack_len >= tcp_send_len) {
#else
        if (ack_len >= reported_total_len) {
#endif
            all_ack = true;
            break;
        }
        if(iter >= 1) {
            k_sleep(K_MSEC(1000));
        }
    }
    return all_ack ? MODEM_RESPONSE_OK : MODEM_RESPONSE_ERROR;
#endif
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
    send_cmd("AT+CFUN=0", 1000, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+CFUN=1,1", 1000, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the responses are as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::init_sms(void)
{
    send_cmd("AT+CMGF=1", DEFAULT_TIMEOUT, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+CNMI=2,1,0,0,0", DEFAULT_TIMEOUT, "OK");
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
    resp_buf[current_index] = '\0';
    return MODEM_RESPONSE_ERROR; // To be changed in future
}

int GPRS::send_get_request(char* url)
{
    char cmd[64];
    send_cmd("AT+HTTPINIT", 1000, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+HTTPPARA=\"CID\",1", DEFAULT_TIMEOUT, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+HTTPSSL=1", 1000, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+HTTPPARA=\"REDIR\",1", 1000, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"%s\"", url);
    send_cmd(cmd, 1000, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+HTTPACTION=0", 5000, "+HTTPACTION");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+HTTPREAD", 1000, "OK");

    // If the responses are as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::bt_power_on(void)
{
    send_cmd("AT+BTPOWER=1", 1000, NULL);
    if ((NULL != strstr(resp_buf, "OK")) ||
        (NULL != strstr(resp_buf, "ERROR"))) {
        return MODEM_RESPONSE_OK; // Connection success
    }
    return MODEM_RESPONSE_ERROR;
}

int GPRS::accept_bt(void)
{
    send_cmd("AT+BTACPT=1", 1000, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    return MODEM_RESPONSE_OK;
}

int GPRS::accept_bt_pair(void)
{
    send_cmd("AT+BTPAIR=1,1", 1000, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    return MODEM_RESPONSE_OK;
}

int GPRS::send_bt_data(unsigned char *data, int len)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+BTSPPSEND=%d", len);
    send_cmd(cmd, 1000, ">");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // Send data
    for (int i = 0; i < len; i++) {
        uart_poll_out(gsm_dev, data[i]);
    }

    prepare_for_rx(DEFAULT_TIMEOUT, "OK");
    k_sleep(K_MSEC(DEFAULT_TIMEOUT));
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::check_bt_host(const char *host)
{
    send_cmd("AT+BTHOST?", 1000, host);
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
    send_cmd(cmd, 1000, ">");
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
    send_cmd(cmd, 1000, "OK");
    return MODEM_RESPONSE_OK;
}

int GPRS::answer(void)
{
    return MODEM_RESPONSE_OK;
}

int GPRS::call_up(char *number)
{
    send_cmd("AT+COLP=1", 1000, "OK");

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
    send_cmd("AT+CIPGSMLOC=1,1", 1000, "$$+CIPGSMLOC: ");
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

    send_cmd(cmd, 400, "OK");
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

    // Server name
    snprintf(cmd, sizeof(cmd), "AT+FTPSERV=\"%s\"", server);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // User name
    snprintf(cmd, sizeof(cmd), "AT+FTPUN=\"%s\"", user);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    snprintf(cmd, sizeof(cmd), "AT+FTPPW=\"%s\"", pw);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    snprintf(cmd, sizeof(cmd), "AT+FTPGETNAME=\"%s\"", file_name);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    snprintf(cmd, sizeof(cmd), "AT+FTPGETPATH=\"%s\"", file_path);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
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
    send_cmd(cmd, 400, NULL); // More than 1 minute for response?

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int GPRS::read_ftp_file(const char *file_name, int length, int offset)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+FSREAD=C:\\User\\FTP\\%s,1,%d,%d", file_name, length, offset);
    send_cmd(cmd, 500, NULL);

    return MODEM_RESPONSE_OK;
}

int GPRS::ftp_get_to_ram(const char *server, const char *user, const char *pw,
                         const char *file_name, const char *file_path)
{
    if (ftp_init(server, user, pw, file_name, file_path) != MODEM_RESPONSE_OK) {
        return MODEM_RESPONSE_ERROR;
    }

    // Open the FTP session
    send_cmd("AT+FTPEXTGET=1", DEFAULT_TIMEOUT, NULL);
    if (NULL == strstr(resp_buf, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }
    // The caller needs to wait until the response "+FTPEXTGET: 1,0"
    // is received, which indicates that the downloading is
    // successful.
    prepare_for_rx(90000, "EXTGET: 1,0");

    return MODEM_RESPONSE_OK;
}

int GPRS::ftp_get_ram_filesize(void)
{
    int file_size = 0;
    send_cmd("AT+FTPEXTGET?", DEFAULT_TIMEOUT, NULL);

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
    send_cmd(cmd, FTP_READ_WAIT, NULL);
    if (strip_modem_response(length) > 0) {
        return MODEM_RESPONSE_OK;
    }
    else {
        return MODEM_RESPONSE_ERROR;
    }
}

int GPRS::ftp_end_session(void)
{
    send_cmd("AT+FTPEXTGET=0", DEFAULT_TIMEOUT, "OK");

    return MODEM_RESPONSE_OK;
}

#endif /* UNIT_TEST */
