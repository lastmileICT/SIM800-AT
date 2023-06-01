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

#ifndef __MODEM_H__
#define __MODEM_H__

#ifndef UNIT_TEST

#include <stdio.h>
#include <stdint.h>
#include <string>
#if defined(__ZEPHYR__)
#include <zephyr.h>
#endif
#ifdef CONFIG_SOC_SERIES_STM32L0X
#include "stm32l072xx.h" // For the USART_TypeDef declaration
#endif

#define DEFAULT_TIMEOUT 50
#define MODEM_RESPONSE_OK 0
#define MODEM_RESPONSE_ERROR -1
#define MODEM_CME_ERROR -2


/** UARTmodem class.
 *  Used for mobile communication.
 */
class UARTmodem
{
public:
    char *resp_buf;
    size_t resp_buf_len;
    size_t tcp_send_len;

    const struct device *modem_dev;
    USART_TypeDef *UART_PERIPH;
    char ack_message[12];
    size_t len_ack;
    int time_out = DEFAULT_TIMEOUT;
    uint32_t time_initial;
    uint16_t actual_ack_num_bytes;
    volatile size_t current_index = 0;
    volatile bool ack_received = false;
    void (*feed_watchdog)(int);
    int* wdt_channel;

    UARTmodem(uint8_t *rx_buf, size_t rx_buf_size,
            void (*feed_watchdog)(int) = NULL, int* wdt_channel = NULL);

    /** Set / reset the reception buffer. Ideally should be executed when no RX is in progress.
     *  @param buf Pointer to the buffer that will store the received UART data.
     *  @param size_buf Size of the UART buffer
     */
    void set_rx_buf(uint8_t *buf, size_t len);
    int init(void);
    int test_uart(); // simple AT -> OK serial comm test
    int wakeup(void);
    int check_pin(void);
    int set_pin(const char* pin);
    int check_ssl_cert(const char *filename, int filesize);
    int load_ssl(const char *filename, const char *cert, int filesize);
    int ssl_set_cert(const char *filename);
    int enable_ssl(void);
    int disable_ssl(void);
    int enable_get_data_manually(void);
    int setup_clock(void);
    /**
     * Bearer Settings for Applications Based on IP. This configures the Access Point Name (APN)
     * parameters for internet access. The function sends the AT command "AT+SAPBR=3,......"
     * to the GSM modem.
     * @param apn APN for the internet gateway
     * @param user User ID for APN
     * @param pass Password for APN
     * @return Returns -1 if invalid or no response from the modem
     */

    int setup_bearer(const char* apn, const char* user, const char* pass);
    /**
     * Enables/opens the bearer, provided the settings have been done.
     * The function sends the AT command "AT+SAPBR=1,.." to the GSM modem.
     * @return Returns -1 if invalid or no response from the modem
     */
    int enable_bearer(void);

    /**
     * Check GSM registration.
     * The function sends the AT command "AT+CREG?" to the GSM modem.
     * @return Returns -1 if invalid or no response from the modem
     */
    int network_registration_gsm(void);

    /**
     * Check packet switched registration.
     * The function sends the AT command "AT+CGREG?" to the GSM modem.
     * @return Returns -1 if invalid or no response from the modem
     */
    int network_registration_gprs(void);

    /**
     * The function sends the AT command "AT+CSQ" to the GSM modem and gets
     * the signal strength value (RSSI) as the response.
     * @return Returns -1 if invalid RSSI or no response from the modem
     */
    int check_signal_strength(void);

    uint32_t get_time(void);
    int attach_gprs(void);

    /**
     * Start Task and Set APN, USER NAME, PASSWORD.
     * The function sends the AT command "AT+CSTT=apn,user,pass" to the GSM modem.
     * @param apn APN details
     * @param user User ID
     * @param pass Password
     * @return Returns -1 if invalid or no response from the modem, 0 if success.
     */
    int set_apn(const char* apn, const char* user, const char* pass);

    /**
     * Bring Up Wireless Connection with GPRS.
     * The function sends the AT commands "AT+CIICR" to the GSM modem.
     * If this was setup already, the modem will respond with "ERROR", and this
     * is actually not an error.
     * @return Returns 0 if the modem responds with "OK" or "ERROR".
     * Returns -1 if no response or invalid.
     */
    int activate_gprs(void);

    /**
     * Get Local IP address, if the PDP context has been activated before.
     * The function sends the AT commands "AT+CIFSR" to the GSM modem.
     * @return Returns 0 if the modem response is a valid IP address. Returns -1 if the GSM modem
     * responds with an "ERROR". This can happen if the PDP context has not been activated already.
     */
    int get_ip(void);

    /**
     * Opens TCP connection sending the AT command "AT+CIPSTART=TCP,domain,port" to the GSM modem.
     * Expects a "CONNECT OK" or "ALREADY CONNECT" from the modem on successful connection.
     * @param domain  Remote server domain name
     * @param port Remote server port
     * @return Returns -1 if invalid or no response from the modem
     */
    int connect_tcp(const char* domain, const char* port);

    /**
     * Closes the TCP connection.
     * The function sends the AT command "AT+CIPCLOSE=0" to the GSM modem.
     * The modem may return ERROR if TCP is not already opened.
     * @return Returns -1 on receiving an invalid response.
     * Returns 0 if CLOSE OK or ERROR received from the modem.
     */
    int close_tcp(void);

    /**
     * Quickly closes the TCP connection.
     * The function sends the AT command "AT+CIPCLOSE=1" to the GSM modem.
     * The modem may return ERROR if TCP is not already opened.
     * @return Returns -1 on receiving an invalid response.
     * Returns 0 if CLOSE OK or ERROR received from the modem.
     */
    int close_tcp_quick(void);

    int detach_gprs(void);
    int disable_bearer(void);
    int send_tcp_data(const void *data, size_t len);
    int reset(void);

    /**
     * Get the currently selected network operator.
     * This function sends the command "AT+COPS?" to the modem.
     * @return Returns -1 if invalid or no response from the modem, and 0 if success
     */
    int get_active_network(void);

    /**
     * Function to manually switch to a specific available network and then
     * switches back to auto mode if the manual selection fails.
     * This function sends the command (AT+COPS=4,1,"OperatorShortName").
     * This can take upto 2 minutes to get a response.
     * So, need to be careful about a watchdog reset in the main program.
     * Immediate response-check is also handled in the function.
     * @return Returns -1 if invalid or no response from the modem
     */
    int select_network(const char *network);

    /**
     * Requests IP data using CIPRXGET.
     * @return Returns the size of TCP/UDP data.
     */
    virtual int ip_rx_data(void) = 0;

    /**
     * Strip the modem response from the start of a buffer and move the rest
     * of the contents to the start of the buffer
     * @param count The number of bytes after the strip to move to start
     * @return The number of bytes stripped
     */
    size_t strip_modem_response(size_t count);

    /**
     * Check the current bearer status sending the command "AT+SAPBR=2,1"
     * to the GSM modem.
     * @return Returns -1 if invalid or no response from the modem.
     * 0 = Bearer is connecting, 1 = Bearer is connected, 2 = Bearer is closing
     * 3 = Bearer is closed
     */
    int check_bearer_status(void);

    /**
     * Closes the GPRS PDP context.
     * The function sends the AT command "AT+CIPSHUT" to the GSM modem.
     * @return Returns -1 on receiving an invalid response.
     * Returns 0 if SHUT OK received from the modem.
     */
    int close_pdp_context(void);

    void sleep(void);
    void powerdown(void);

    /**
     * This function sends the command (AT+COPS=?) for searching the available networks.
     * Reading the response must be done separately.
     * It can take up to 45 seconds to get a response.
     * So, need to be careful about a watchdog reset in the main program.
     */
    int search_networks(void);

    /**
     * This function needs to be called after a command is sent to the GSM modem.
     * The function prepares the variables before receiving data from the modem in the ISR.
     * It resets the flags, starts the timer for checking timeouts, and sets the
     * acknowledgement string to the global variable. It also enables the Rx interrupt after
     * a command is sent to the modem.
     * @param timeout Maximum time (seconds) to wait for the modem response, after which
     * the Rx interrupt will be disabled.
     * @param ack_message Pointer to a string containing the expected acknowledgement.
     */
    void prepare_for_rx(size_t timeout, const char *ack);

    /**
     * Implements FTP session initialisation to be used by FTPGETTOFS as well as FTPEXTGET.
     * @param server FTP Server Name
     * @param user FTP Server User Name
     * @param pw FTP Server Password
     * @param file_name The file name to be read
     * @param file_path FTP Server File Path
     */
    int ftp_init(const char *server, const char *user, const char *pw, const char *file_name,
                    const char *file_path);

    /**
     * Remove the existing file in the SIM800 flash.
     * @param file_name The file name to be deleted.
     */
    int delete_file(const char *file_name);

    /**
     * Implements the FTP get functionality to download a file from an FTP server using
     * the command AT+FTPGETTOFS. The final response from the SIM800 modem will be as below,
     * +FTPGETTOFS: 0, <filesize>
     * The downloaded file will be saved to the SIM800 system (flash) memory C:\User\FTP\
     * The final response and the acknowledgement from the modem can take 30-90 seconds, and hence, the calling function
     * will have to wait for this.
     * @param server FTP Server Name
     * @param user FTP Server User Name
     * @param pw FTP Server Password
     * @param file_name The file name to be read
     * @param file_path FTP Server File Path
     */
    int ftp_get(const char *server, const char *user, const char *pw, const char *file_name,
                const char *file_path);

    /**
     * Read bytes from the downloaded file in the SIM800 flash.
     * Implements the FTP file read from the SIM800 modem using the command, for example,
     * AT+FSREAD=C:\\User\\FTP\\zephyr.bin,1,10240,<starting byte>.
     * Each response from the modem starts with 2 bytes 0d 0a and ends with 7 bytes
     * (0d 0a 4f 4b 0d 0a 00), which needs to be ignored while processing.
     * @param file_name The file name to be read.
     * @param length Number of bytes to be read.
     * @param offset The starting index to read from.
     */
    int read_ftp_file(const char *file_name, int length, int offset);

    /**
     * Implements the "FTP extend get" functionality to download a file from an FTP server using
     * the command AT+FTPEXTGET. The final response from the SIM800 modem will be as below,
     * +FTPEXTGET: 1,0
     * The downloaded file will be saved to the SIM800 RAM.
     * The final response from the modem can take nearly 1 minute, and hence,
     * the calling function will have to wait for this.
     * @param server FTP Server Name
     * @param user FTP Server User Name
     * @param pw FTP Server Password
     * @param file_name The file name to be read
     * @param file_path FTP Server File Path
     * @return Returns 0 on success, and -1 on receiving an invalid response or error.
     */
    int ftp_get_to_ram(const char *server, const char *user, const char *pw, const char *file_name,
                    const char *file_path);

    /**
     * Read the size of the downloaded file in the SIM800 RAM after a FTPEXTGET request.
     * A command "AT+FTPEXTGET?" to the modem will be responded with, for example
     * "+FTPEXTGET: 1,108944" which contains the file size info.
     * @return Returns the file size if success or -1 on receiving an invalid response.
     */
    int ftp_get_ram_filesize(void);

    /**
     * Read bytes from the downloaded file in the SIM800 RAM.
     * Implements the FTPEXTGET file read from the SIM800 modem using the command,
     * for example, AT+FTPEXTGET=3,0,128.
     * The read bytes will be available in the buffer "resp_buf" after a successful read operation.
     * The wait time after sending the read command is configurable at compile-time
     * (CONFIG_FTP_READ_WAIT).
     * A response for the request AT+FTPEXTGET=3,0,128 will always start with
     * 2 bytes 0x0D 0x0A followed by "+FTPEXTGET: 3,128" and then 2 additional bytes 0x0D 0x0A.
     * Then the actual bytes come and then ends with 6 bytes (0x0d 0x0a 0x4f 0x4b 0x0d 0x0a).
     * These additional bytes needs to be ignored while processing.
     * @param length Number of bytes to be read.
     * @param offset The starting index to read from.
     */
    int ftp_read_from_ram(int length, int offset);

    /**
     * Ends the FTPEXTGET session by sending the command, "AT+FTPEXTGET=0"
     */
    int ftp_end_session(void);

protected:
    /**
     * Function to send the AT commands to SIM800 module.
     * UART transmission is still done using polling method.
     * Reception happens in the background through an interrupt service routine.
     * Use some delay after sending the command before processing the response,
     * since SIM800 might take few seconds to respond.
     * @param cmd Command string to send to the modem
     * @param timeout Maximum time (seconds) to wait for the modem response, after which
     * the Rx interrupt will be disabled.
     * @param ack Pointer to a string containing the expected acknowledgement. Pass NULL as the
     * argument if there is a chance that the modem takes more time to respond and then handled the
     * response checking externally. If NULL is passed as argument, the buffer need to be emptied
     * for handling few special cases. For example, when the timeout didn't reach and there is no more
     * data coming from the modem. Here, the null terminating part in the ISR won't reach, since
     * this can only happen if there is a reception at the Rx pin. This is fixed by clearing the buffer.
     * See the implementation of the ISR function "read_resp()" for more details.
     */
    void send_cmd(const char *cmd, size_t timeout, const char *ack,
                  bool no_wait=false);
private:
    void clear_buffer(void);
};

class SIM800 : public UARTmodem {
public:
    SIM800(uint8_t *rx_buf, size_t rx_buf_size,
            void (*feed_watchdog)(int) = NULL, int* wdt_channel = NULL);

    /**
     * Requests IP data by sending the command "AT+CIPRXGET=2,x" to the modem.
     * @return Returns the size of TCP/UDP data.
     */
    int ip_rx_data(void);

private:
};

class A7672 : public UARTmodem {
public:
    A7672(uint8_t *rx_buf, size_t rx_buf_size,
            void (*feed_watchdog)(int) = NULL, int* wdt_channel = NULL);

    /**
     * Requests IP data by sending the command "AT+CIPRXGET=2,x" to the modem.
     * @return Returns the size of TCP/UDP data.
     */
    int ip_rx_data(void);

private:
};

#endif /* UNIT_TEST */

#endif /* __MODEM_H__ */
