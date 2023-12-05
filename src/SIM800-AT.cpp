/* SIM800_AT firmware
 * Copyright (c) 2016-2023 Inclusive Energy Ltd
 * (www.inclusive.energy)
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

#include "uart_dma_driver.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(sim800, CONFIG_GSM_LOG_LEVEL);

#define UART_GSM DT_LABEL(DT_ALIAS(uart_gsm))
#define UART_GSM_BASE_ADDR DT_REG_ADDR(DT_ALIAS(uart_gsm))
#define UART_GSM_SPEED DT_PROP(DT_ALIAS(uart_gsm), current_speed)
#define UART_GSM_DMA_BASE_ADDR DT_REG_ADDR(DT_PHANDLE_BY_NAME(DT_ALIAS(uart_gsm), dmas, rx))
#define UART_GSM_DMA_CHANNEL DT_PHA_BY_NAME(DT_ALIAS(uart_gsm), dmas, rx, channel)
#define UART_GSM_DMA_SLOT DT_PHA_BY_NAME(DT_ALIAS(uart_gsm), dmas, rx, slot)

// Base constructor sets up basic requirements for UART
UARTmodem::UARTmodem(uint8_t *rx_buf, size_t rx_buf_size)
{
    UART_PERIPH = (USART_TypeDef *)(UART_GSM_BASE_ADDR);
    modem_dev = device_get_binding(UART_GSM);
    DMA_PERIPH = (DMA_TypeDef *)(UART_GSM_DMA_BASE_ADDR);
    this->set_rx_buf(rx_buf, rx_buf_size);
    uart_dma_init_rx(DMA_PERIPH, UART_PERIPH, resp_buf,
                    UART_GSM_DMA_CHANNEL, UART_GSM_DMA_SLOT);
}

// Inherited constructor calls base constructor
SIM800::SIM800(uint8_t *rx_buf, size_t rx_buf_size)
            : UARTmodem(rx_buf, rx_buf_size)

{ // no special construction
}

// Inherited constructor calls base constructor
A7672::A7672(uint8_t *rx_buf, size_t rx_buf_size)
            : UARTmodem(rx_buf, rx_buf_size)

{ // no special construction
}

// Strips the modem response and moves the following count bytes to the
// start of the response buffer. Returns count bytes stripped.
size_t SIM800::strip_modem_response(size_t count, const char* resp)
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

// Strips the modem response up to the end of the line containing resp.
// Returns count bytes stripped.
size_t A7672::strip_modem_response(size_t count, const char* resp)
{
    char *ack_pos = strstr(resp_buf, resp);
    if (ack_pos == NULL) { return 0; }
    size_t pos = (size_t)(ack_pos - resp_buf);

    // Search for a newline and move the memory after it to
    // the start of the buffer
    for (; pos <= resp_buf_len - 2; pos++) {
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

void UARTmodem::set_rx_buf(uint8_t *buf, size_t len)
{
    // TODO: check no RX in progress
    resp_buf = (char *)buf;
    resp_buf_len = len;
}

// See if the ack has been put in the buffer by the DMA
void UARTmodem::ack_check() {
    if (len_ack != 0) {
        if (strstr(resp_buf, ack_message) != NULL) {
            ack_received = true;
        }
    }
}

void UARTmodem::send_cmd(const char *cmd, size_t timeout, const char *ack,
                    bool no_wait /*=false*/)
{
    // LOG_DBG("send: %s",cmd);

    prepare_for_rx(timeout, ack);
    for (int i = 0; i < (int)strlen(cmd); i++) {
        uart_poll_out(modem_dev, cmd[i]);
    }
    uart_poll_out(modem_dev, '\r');
    uart_poll_out(modem_dev, '\n');

    // Some commands can specify a timeout but don't want to wait for
    // the ack
    if (!no_wait) {
        // Sleep in 20ms slices until we get the ack back.
        int wait_count = timeout / 20;
        while (wait_count--) {
            k_sleep(K_MSEC(20));
            ack_check();
            if (ack_received){
                break;
            }
        }
        // LOG_DBG("recv: %s",resp_buf);
    }
}

// Clear the rx buffer and reconfigure the DMA. Use just before
// sending a command to the modem.
void UARTmodem::prepare_for_rx(size_t timeout, const char *ack)
{
    uart_dma_rx_stop(DMA_PERIPH, UART_PERIPH, UART_GSM_DMA_CHANNEL);

    // Clear the old ack info
    ack_message[0] = '\0';
    len_ack = 0;
    ack_received = false;

    // Set up the new ack
    if (ack != NULL) {
        len_ack = strlen(ack);
    }
    if (len_ack && len_ack < sizeof(ack_message) - 1) {
        strncpy(ack_message, ack, len_ack + 1);
    }
    memset(resp_buf, 0, resp_buf_len);

    // No longer used. Could be used to stop the DMA RX
    // but there is no real need since there is no performance hit.
    time_out = timeout;

    uart_dma_rx_start(DMA_PERIPH, UART_PERIPH,
                        resp_buf, resp_buf_len, UART_GSM_DMA_CHANNEL);
}

// simple AT -> OK serial comm test
int UARTmodem::test_uart()
{
    for (int i = 0; i < 2; i++) {
        send_cmd("AT", DEFAULT_TIMEOUT, "OK");
        if (ack_received) {
            return MODEM_RESPONSE_OK;
        }
    }
    return MODEM_RESPONSE_ERROR;
}

int UARTmodem::init_common() {
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

    return MODEM_RESPONSE_OK;
}

int SIM800::init(void)
{

    if (init_common() == MODEM_RESPONSE_ERROR) {
        return MODEM_RESPONSE_ERROR;
    }

    // Try to use the maximum GPRS class (12) for SIM800, in which 4 TX
    // slots and 4 RX slots are being used. We don't mind errors here.
    send_cmd("AT+CGMSCLASS=12", DEFAULT_TIMEOUT, "OK");

    return MODEM_RESPONSE_OK;
}

int A7672::init(void)
{
    if (init_common() == MODEM_RESPONSE_ERROR) {
        return MODEM_RESPONSE_ERROR;
    }

    // Turn off unsolicited return codes we dont care
    // about
    send_cmd("AT+CGEREP=0", 9000, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    send_cmd("AT+CNMI=0", 9000, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int UARTmodem::wakeup(void)
{
    send_cmd("AT", DEFAULT_TIMEOUT, NULL);
    return MODEM_RESPONSE_OK;
}

int UARTmodem::check_pin(void)
{
    send_cmd("AT+CPIN?", 9000, "PIN: READY");
    if (ack_received) {
        return MODEM_RESPONSE_OK;
    }
    else if (NULL != strstr(resp_buf, "not insert")) {
        return MODEM_CME_ERROR;
    }
    else {
        // Likely the SIM is locked
        return MODEM_RESPONSE_ERROR;
    }
}

int SIM800::set_pin(const char* pin)
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

int A7672::set_pin(const char* pin)
{
    char cpin[20];
    sprintf(cpin, "AT+CPIN=%s", pin);
    send_cmd(cpin, DEFAULT_TIMEOUT, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int SIM800::get_iccid(char* buf)
{
    send_cmd("AT+CCID", 2000, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    sscanf(resp_buf, "%s", buf);
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int A7672::get_iccid(char* buf)
{
    send_cmd("AT+CICCID", 9000, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    sscanf(resp_buf, " +ICCID: %s", buf);
    return MODEM_RESPONSE_OK;
}

int UARTmodem::check_ssl_cert(const char *filename, int filesize)
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

int UARTmodem::load_ssl(const char *filename, const char *cert, int filesize)
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


int UARTmodem::ssl_set_cert(const char *filename)
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

int SIM800::enable_ssl(void)
{
    send_cmd("AT+CIPSSL=1", DEFAULT_TIMEOUT, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int A7672::enable_ssl(void)
{
    use_ssl = true;
    return MODEM_RESPONSE_OK;
}

int SIM800::disable_ssl(void)
{
    send_cmd("AT+CIPSSL=0", DEFAULT_TIMEOUT, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int A7672::disable_ssl(void)
{
    use_ssl = false;
    return MODEM_RESPONSE_OK;
}

int SIM800::setup_clock(void)
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

int A7672::setup_clock(void)
{
    send_cmd("AT+CNTP=\"time1.google.com\",0", DEFAULT_TIMEOUT, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+CNTP", 1500, "+CNTP: 0");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int SIM800::setup_bearer(const char* apn, const char* user, const char* pass)
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

int A7672::setup_bearer(const char* apn, const char* user, const char* pass) {

    // Set connection ID 1 to use IP(v4) with APN address provided
    char cmd[50];
    snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", apn);
    send_cmd(cmd, 9000, "OK");

    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    return MODEM_RESPONSE_OK;
}

int SIM800::enable_bearer(void)
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

int A7672::enable_bearer(void)
{
    // Activate PDP context for connection ID 1
    // NOTE: this is automatic on LTE networks
    // but is retained for GSM
    send_cmd("AT+CGACT=1,1", 9000, "OK");
    if (ack_received) {
        return MODEM_RESPONSE_OK;
    }

    // Error or no response
    return MODEM_RESPONSE_ERROR;
}

int UARTmodem::check_bearer_status(void)
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

int UARTmodem::get_active_network(void)
{
    // Ensure network format is long alphanumeric name
    send_cmd("AT+COPS=3,0", DEFAULT_TIMEOUT, "OK");

    // Query the current network name
    send_cmd("AT+COPS?", DEFAULT_TIMEOUT, "OK");
    if (ack_received) {
        return MODEM_RESPONSE_OK;
    }
    // Invalid or no response
    return MODEM_RESPONSE_ERROR;
}

int UARTmodem::search_networks(void)
{
    send_cmd("AT+COPS=?", 45000, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    return MODEM_RESPONSE_OK;
}

int UARTmodem::select_network(const char *network)
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

int UARTmodem::network_registration_gsm(void)
{
    send_cmd("AT+CREG?", DEFAULT_TIMEOUT, NULL);
    if ((NULL != strstr(resp_buf, "+CREG: 0,1")) ||
        (NULL != strstr(resp_buf, "+CREG: 0,5"))) {
        return MODEM_RESPONSE_OK;
    }
    // Not registered yet
    return MODEM_RESPONSE_ERROR;
}

int UARTmodem::network_registration_gprs(void)
{
    send_cmd("AT+CGREG?", DEFAULT_TIMEOUT, NULL);
    if ((NULL != strstr(resp_buf, "+CGREG: 0,1")) ||
        (NULL != strstr(resp_buf, "+CGREG: 0,5"))) {
        return MODEM_RESPONSE_OK;
    }
    // Not registered yet
    return MODEM_RESPONSE_ERROR;
}

int SIM800::network_registration_lte(void)
{
    // No LTE functionality
    return MODEM_RESPONSE_ERROR;
}

int A7672::network_registration_lte(void)
{
    send_cmd("AT+CEGREG?", DEFAULT_TIMEOUT, NULL);
    if ((NULL != strstr(resp_buf, "+CGREG: 0,1")) ||
        (NULL != strstr(resp_buf, "+CGREG: 0,5"))) {
        return MODEM_RESPONSE_OK;
    }
    // Not registered yet
    return MODEM_RESPONSE_ERROR;
}

int UARTmodem::check_signal_strength(void)
{
    int value = 0;
    send_cmd("AT+CSQ", 9000, "OK");
    if (ack_received) {
        // Extract the integer value from the received string.
        sscanf(resp_buf, " +CSQ: %d,", &value);
        int rssi = (2*value - 113);
        if ((rssi >= 0) || (rssi < -115)) {
            return MODEM_RESPONSE_ERROR;
        }
        return rssi;
    }
    return MODEM_RESPONSE_ERROR; // If no response
}

int SIM800::get_connection_info() {
    // No info to get
    return MODEM_RESPONSE_ERROR;
}

int A7672::get_connection_info() {
    send_cmd("AT+CPSI?", 9000, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    return MODEM_RESPONSE_OK;
}

uint32_t UARTmodem::get_time(void)
{
    send_cmd("AT+CCLK?", 9000, "OK");
    if (ack_received) {
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
    }
    return MODEM_RESPONSE_ERROR; // If no response
}

int UARTmodem::attach_gprs(void)
{
    // Attach GPRS
    send_cmd("AT+CGATT=1", 1000, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the responses are as expected
    return MODEM_RESPONSE_OK;
}

int SIM800::enable_get_data_manually(void)
{
    // CIPMUX=0 -> set single IP connection
    // CIPRXGET=1 -> poll for RX data.
    send_cmd("AT+CIPMUX=0;+CIPRXGET=1", DEFAULT_TIMEOUT * 10, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    return MODEM_RESPONSE_OK;
}

int A7672::enable_get_data_manually(void)
{
    send_cmd("AT+CCHMODE=0", 1000, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+CCHSET=1,1", 1000, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    return MODEM_RESPONSE_OK;
}

int SIM800::pdp_open(const char* apn, const char* user, const char* pass)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CSTT=\"%s\",\"%s\",\"%s\"", apn, user, pass);
    send_cmd(cmd, DEFAULT_TIMEOUT, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    send_cmd("AT+CIICR", 1000, NULL); // Upto 85 seconds
    if ((NULL != strstr(resp_buf, "OK")) ||
        (NULL != strstr(resp_buf, "ERROR"))) {
        // If the responses are as expected - OK or ERROR received
        return MODEM_RESPONSE_OK;
    }
    // If no response or invalid
    return MODEM_RESPONSE_ERROR;
}

int A7672::pdp_open(const char* apn, const char* user, const char* pass)
{
    send_cmd("AT+CCHSTART", 10000, "+CCHSTART: 0"); // Upto 120 seconds
    if (ack_received == false) {
        //NB. can return ERROR if already open, unhelpful
        return MODEM_RESPONSE_ERROR;
    }
return MODEM_RESPONSE_OK;
    // Could actually take up to 120s but there are two positive
    // ack's so we should search earlier than this. 120s could be
    // reduced using AT+CIPTIMEOUT
    // send_cmd("AT+NETOPEN", 9000, "OK");
    // if ((NULL != strstr(resp_buf, "OK")) ||
    //     (NULL != strstr(resp_buf, "already"))) {
    //     // If the responses are as expected - OK or
    //     // "already opened" received
    //     return MODEM_RESPONSE_OK;
    // }

    // If no response or invalid
    return MODEM_RESPONSE_ERROR;
}

int SIM800::get_ip(void)
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

int A7672::get_ip(void)
{
    // AT+IPADDR shouldnt be used when using the SSL command set,
    // as it requires AT+NETOPEN which is also not required
    send_cmd("AT+CGPADDR=1", 9000, "OK");
    char *ack_pos = strstr(resp_buf, "+CGPADDR: 1,");
    int ip1, ip2, ip3, ip4 = 0;
    if (NULL != ack_pos) {
        sscanf(ack_pos, "+CGPADDR: 1,%d.%d.%d.%d", &ip1,&ip2,&ip3,&ip4);
        LOG_DBG("IP: %d.%d.%d.%d", ip1,ip2,ip3,ip4);
        if (!(ip1 == 0 && ip2 == 0 && ip3 == 0 && ip4 == 0)) {
            return MODEM_RESPONSE_OK;
        }
    }
    return MODEM_RESPONSE_ERROR;
}

int SIM800::connect_tcp(const char *domain, const char *port)
{
    char cmd[100];
    this->tcp_send_len = 0;
    sprintf(cmd, "AT+CIPSTART=TCP,%s,%s", domain, port);
    send_cmd(cmd, 5000, "CONNECT OK");
    if (ack_received || (NULL != strstr(resp_buf, "ALREADY CONNECT"))) {
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

int A7672::connect_tcp(const char *domain, const char *port)
{
    char cmd[100];
    this->tcp_send_len = 0;
    uint8_t client_type = use_ssl ? 2 : 1;

    // Use and check against session ID 0 (first parameter)
    sprintf(cmd, "AT+CCHOPEN=0,\"%s\",%s,%d", domain, port, client_type);
    send_cmd(cmd, 10000, "+CCHOPEN: 0,0");
    if (ack_received) {
        return MODEM_RESPONSE_OK;
    }
    return MODEM_RESPONSE_ERROR; // Invalid
}

int SIM800::ip_rx_data(void)
{
    char cmd[19];
    uint16_t tcp_packet_len = 0;
    uint16_t tcp_avail = 65535;

    uint8_t retries = 0;
    while((tcp_packet_len != tcp_avail) && (retries++ < 10)) {
        send_cmd("AT+CIPRXGET=4", DEFAULT_TIMEOUT, "OK");
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

    size_t wasted = strip_modem_response(tcp_packet_len, NULL);
    if (tcp_packet_len > (resp_buf_len - wasted)) {
        tcp_packet_len = resp_buf_len - wasted;
    }
    return tcp_packet_len;
}

int A7672::ip_rx_data(void)
{
    char cmd[19];
    uint16_t tcp_packet_len = 0;
    uint16_t tcp_avail = 65535;

    uint8_t retries = 0;
    while((tcp_packet_len != tcp_avail) && (retries++ < 10)) {
        send_cmd("AT+CCHRECV?", 5000, "OK");
        // ACK string position varies between 2 and three characters
        // with mixed whitespace so we can't sscanf for it properly.
        char *ack_pos = strstr(resp_buf, "+CCHRECV:");
        if (NULL != ack_pos) {
            tcp_packet_len = tcp_avail;
            sscanf(ack_pos, "+CCHRECV: LEN,%hu", &tcp_avail);
        }
        k_sleep(K_MSEC(150));
    }
    if (tcp_packet_len == 0) {
        return 0;
    }

    // The length of output data can not exceed 2048 bytes at a time,
    // based on A76xx Series_AT Command Manual.
    // We assume the worst case transfer speed of 9.6kbps, equivalent
    // to ~1byte/ms. We also append a margin of 55ms.
    if (tcp_packet_len > 2048) {
        tcp_packet_len = 2048;
    }

    // Recieve len bytes using session ID 0
    snprintf(cmd, sizeof(cmd), "AT+CCHRECV=0,%d", tcp_packet_len);
    int timeout = tcp_packet_len + 55;
    send_cmd(cmd, timeout, NULL);

    // If the expected response was not present
    if (NULL == strstr(resp_buf, "+CCHRECV: DATA")) {
        return 0;
    }

    size_t wasted = strip_modem_response(tcp_packet_len, "+CCHRECV:");
    if (tcp_packet_len > (resp_buf_len - wasted)) {
        tcp_packet_len = resp_buf_len - wasted;
    }
    return tcp_packet_len;
}

int SIM800::pdp_close(void)
{
    // Close the GPRS PDP context.
    send_cmd("AT+CIPSHUT", 65000, "SHUT OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int A7672::pdp_close(void)
{
    // Close the PDP context.
    send_cmd("AT+CCHSTOP", 10000, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int SIM800::close_tcp()
{
    this->tcp_send_len = 0;
    // Use quick-close (first param) to close TCP
    send_cmd("AT+CIPCLOSE=1", DEFAULT_TIMEOUT, NULL);
    if ((NULL != strstr(resp_buf, "CLOSE OK")) ||
        (NULL != strstr(resp_buf, "ERROR"))) { // If TCP not opened previously
        return MODEM_RESPONSE_OK;
    }
    return MODEM_RESPONSE_ERROR; // Invalid
}

int A7672::close_tcp()
{
    this->tcp_send_len = 0;
    // Close TCP connection on session 0
    send_cmd("AT+CCHCLOSE=0", 1000, "+CCHCLOSE: 0,0");
    // If successful close or TCP not opened previously
    if (ack_received || (NULL != strstr(resp_buf, "ERROR"))) {
        return MODEM_RESPONSE_OK;
    }
    return MODEM_RESPONSE_ERROR; // Invalid
}

int UARTmodem::detach_gprs(void)
{
    send_cmd("AT+CGATT=0", DEFAULT_TIMEOUT, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int SIM800::disable_bearer(void)
{
    send_cmd("AT+SAPBR=0,1", 1000, "OK");
    // Can take up to 65 seconds but we dont really
    // care about error response so dont wait
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int A7672::disable_bearer(void)
{
    // Disable the PDP context for connection ID 1
    // NOTE: this is not possible for LTE cat 1-12
    // as the bearer is always active by default
    send_cmd("AT+CGACT=0,1", 1000, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

void UARTmodem::sleep(void)
{
    send_cmd("AT+CSCLK=2", DEFAULT_TIMEOUT, "OK", true);
}

void UARTmodem::powerdown(void)
{
    send_cmd("AT+CPOWD=0", DEFAULT_TIMEOUT, NULL, true);
}

int SIM800::send_tcp_data(const void *data, size_t len)
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
        uart_poll_out(modem_dev, ((char *)data)[i]);
    }
#ifdef CONFIG_SIM800_TCP_QUICKSEND
    // Send Ctrl-Z, if using quicksend
    k_sleep(K_MSEC(10));
    uart_poll_out(modem_dev, '\x1A');
    prepare_for_rx(3000, "DATA ACCEPT");
#else
    prepare_for_rx(3000, "SEND OK");
#endif

    // Sleep in 20ms slices until we get the ack back.
    int wait_count = 3000 / 20;
    while (wait_count--) {
        k_sleep(K_MSEC(20));
        ack_check();
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

int A7672::send_tcp_data(const void *data, size_t len)
{
    char cmd[64];
    this->tcp_send_len += len;

    // Send len bytes using session ID 0
    snprintf(cmd, sizeof(cmd), "AT+CCHSEND=0,%d", len);
    send_cmd(cmd, 900, ">");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // Send data
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(modem_dev, ((char *)data)[i]);
    }
    prepare_for_rx(3000, "+CCHSEND: 0,0");

    // Sleep in 20ms slices until we get the ack back.
    int wait_count = 3000 / 20;
    while (wait_count--) {
        k_sleep(K_MSEC(20));
        ack_check();
        if (ack_received){
            break;
        }
    }

    if (!ack_received) {
        return MODEM_RESPONSE_ERROR;
    }
    return MODEM_RESPONSE_OK;
}

void UARTmodem::clear_buffer(void)
{
    // For non-buffered serial, even one getc would be enough...
    // Using for-loop to prevent accidental indefinite loop
    unsigned char rec_char;
    for (int i = 0; i < 1000 && (uart_poll_in(modem_dev, &rec_char) == 0); i++) {;}
}

int UARTmodem::reset(void)
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

// Delete a file in the modem file system root
void UARTmodem::delete_file(const char *file_name)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+FSDEL=\"%s\"", file_name);

    send_cmd(cmd, 400, "OK");
    // dont care about errors, as if the file doesnt exist
    // this is an error, but the desired outcome
}

int SIM800::ftp_init(const char *server, const char *user, const char *pw, const char *file_name,
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

int A7672::ftp_init(const char *server, const char *user, const char *pw, const char *file_name,
                    const char *file_path)
{
    // Start FTP using existing bearer config
    send_cmd("AT+CFTPSSTART", 9000, "+CFTPSSTART: 0");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // Login to server with credentials
    char cmd[100];
    snprintf(cmd, sizeof(cmd), "AT+CFTPSLOGIN=\"%s\",21,\"%s\",\"%s\",0",
             server, user, pw);
    send_cmd(cmd, 9000, "+CFTPSLOGIN: 0");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }

    // If the response is as expected
    return MODEM_RESPONSE_OK;
}

int SIM800::ftp_get_ota(const char *server, const char *user, const char *pw,
                        const char *file_name, const char *file_path)
{
    if (ftp_init(server, user, pw, file_name, file_path) != MODEM_RESPONSE_OK) {
        return MODEM_RESPONSE_ERROR;
    }

    // Open the FTP session
    send_cmd("AT+FTPEXTGET=1", DEFAULT_TIMEOUT, "OK");
    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    // The caller needs to wait until the response "+FTPEXTGET: 1,0"
    // is received, which indicates that the downloading is
    // successful.
    prepare_for_rx(90000, "EXTGET: 1,0");

    return MODEM_RESPONSE_OK;
}

int A7672::ftp_get_ota(const char *server, const char *user, const char *pw,
                        const char *file_name, const char *file_path)
{
    if (ftp_init(server, user, pw, file_name, file_path) != MODEM_RESPONSE_OK) {
        return MODEM_RESPONSE_ERROR;
    }

    // Open the FTP session
    char cmd[100];
    snprintf(cmd, sizeof(cmd), "AT+CFTPSGETFILE=\"%s%s\"",
             file_path, file_name);

    // Up to 90s for the download to finish
    // Specify no wait to return to caller
    send_cmd(cmd, 90000, "+CFTPSGETFILE: 0", true);

    // Wait a short time to check there was no immediate error
    k_sleep(K_MSEC(50));
    if (NULL == strstr(resp_buf, "OK")) {
        return MODEM_RESPONSE_ERROR;
    }

    // The caller needs to wait until the response "+CFTPSGETFILE: 0"
    // is received, which indicates that the downloading is
    // successful.
    return MODEM_RESPONSE_OK;
}

int SIM800::ftp_dl_cplt(const char *file_name) {
    // check for download complete string
    if (NULL == strstr(resp_buf, "+FTPEXTGET: 1,0")) {
        // check for any non-zero return code or error
        if (NULL != strstr(resp_buf, "+FTPEXTGET:") ||
            NULL != strstr(resp_buf, "+CME ERROR:")) {
                return -1;
        }
        return 1;
    }
    return 0;
}

int A7672::ftp_dl_cplt(const char *file_name) {
    // check for download complete string
    if (NULL == strstr(resp_buf, "+CFTPSGETFILE: 0")) {
        // check for any non-zero return code or error
        if (NULL != strstr(resp_buf, "+CFTPSGETFILE:") ||
            NULL != strstr(resp_buf, "ERROR")) {
                return -1;
        }
        return 1;
    }

    // rename the downloaded file to ota.bin
    delete_file("ota.bin");
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+FSRENAME=\"%s\",\"ota.bin\"", file_name);
    send_cmd(cmd, 9000, "OK");
    if (ack_received == false) {
        return -1;
    }
    delete_file(file_name);
    return 0;
}

int SIM800::ftp_ota_filesize(void)
{
    int file_size = 0;
    send_cmd("AT+FTPEXTGET?", 1000, "OK");

    // The response should ideally be like, +FTPEXTGET: 1,108944... OK
    char *ptr = strstr(resp_buf, "+FTPEXTGET: 1,");
    if (ptr == NULL) {
        return MODEM_RESPONSE_ERROR;
    }
    // Success
    sscanf(ptr, "+FTPEXTGET: 1,%d,", &file_size);
    return file_size;
}

int A7672::ftp_ota_filesize(void)
{
    int file_size = 0;
    send_cmd("AT+FSATTRI=\"ota.bin\"", 1000, "OK");

    if (ack_received == false) {
        return MODEM_RESPONSE_ERROR;
    }
    // Success
    sscanf(resp_buf, " +FSATTRI: %d", &file_size);
    return file_size;
}

int SIM800::ftp_read_ota(int length, int offset)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+FTPEXTGET=3,%d,%d", offset, length);
    // Previous "clever" scaling of the read wait didnt work once the baud
    // became 115200, and it was very empirical anyway.
    send_cmd(cmd, 90, NULL);
    if (strip_modem_response(length, NULL) > 0) {
        return MODEM_RESPONSE_OK;
    }
    else {
        return MODEM_RESPONSE_ERROR;
    }
}

int A7672::ftp_read_ota(int length, int offset)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CFTRANTX=\"c:/ota.bin\",%d,%d",
            offset, length);
    // cant rely on +CFTRANTX: 0 ack as strstr will not find it if the
    // OTA data contains a \0 character
    send_cmd(cmd, 90, NULL);
    if (strip_modem_response(length, "+CFTRANTX:") > 0) {
        return MODEM_RESPONSE_OK;
    }
    else {
        return MODEM_RESPONSE_ERROR;
    }
}

int SIM800::ftp_end_session(void)
{
    send_cmd("AT+FTPEXTGET=0", DEFAULT_TIMEOUT, "OK");

    return MODEM_RESPONSE_OK;
}

int A7672::ftp_end_session(void)
{
    send_cmd("AT+CFTPSLOGOUT", 500, NULL);
    send_cmd("AT+CFTPSSTOP", DEFAULT_TIMEOUT, NULL);
    return MODEM_RESPONSE_OK;
}

#endif /* UNIT_TEST */
