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

#ifndef __GPRS_H__
#define __GPRS_H__

#ifndef UNIT_TEST

#include <stdio.h>
#include <stdint.h>
#include <string>
#ifdef __MBED__
#include "mbed.h"
#elif defined(__ZEPHYR__)
#include <zephyr.h>
#endif

#define DEFAULT_TIMEOUT 2
#define MODEM_RESPONSE_OK 0
#define MODEM_RESPONSE_ERROR -1
#define MODEM_CME_ERROR -2

#ifdef __ZEPHYR__

/** Initializes the UART interrupts and the buffer.
 * The ISR call-back function read_resp() is configured here and no arguments are passed to the ISR
 * as of now. Also char buffer for collecting the UART data and it's size are set here.
 *  @param buf Pointer to the buffer that will store the received UART data.
 *  @param size_buf Size of the UART buffer
 */
void init_modem(char *buf, uint32_t size_buf);

#endif // #ifdef __ZEPHYR__

/** GPRS class.
 *  Used for mobile communication. attention that GPRS module communicate with MCU in serial protocol
 */
class GPRS
{
public:
#ifdef __MBED__
    /** Create GPRS instance
     *  @param tx  uart transmit pin to communicate with GPRS module
     *  @param rx  uart receive pin to communicate with GPRS module
     *  @param ri  ring indicator goes low for 120ms when an SMS is received
     *  @param baudRate baud rate of uart communication
     */
    GPRS(PinName tx, PinName rx, PinName ri, int baudRate) : gprsSerial(tx, rx) {};

    Serial gprsSerial;

    /**
     * Reads the server response through serial port. The function is used in 2 ways.
     * 1) To read-out the buffer, in which case, no return is actually expected.
     * 2) To read-out and also check the response against an expected message.
     * @param time_out Maximum time (seconds) spent for reading the serial port
     * @param ack_message Pointer to a string containing the expected message.
     * This argument should be NULL, if no acknowledgment-check is needed.
     * @return While checking the response against an expected message,
     * Invalid response : -1
     * Valid response   : 0
     * When used only to read-out the buffer, the function always returns -1.
     */
    int read_resp(char *resp_buf, int size_buf, int time_out, const char *ack_message);

    int init(char *resp_buf, int size_buf);
    int wakeup(char *resp_buf, int size_buf);
    int check_pin(char *resp_buf, int size_buf);
    int set_pin(const char* pin, char *resp_buf, int size_buf);
    int check_ssl_cert(const char *filename, int filesize, char *resp_buf, int size_buf);
    int load_ssl(const char *filename, const char *cert, int filesize, char *resp_buf, int size_buf);
    int ssl_set_cert(const char *filename, char *resp_buf, int size_buf);
    int enable_ssl(char *resp_buf, int size_buf);
    int enable_get_data_manually(char *resp_buf, int size_buf);
    int setup_clock(char *resp_buf, int size_buf);
    int setup_bearer(const char* apn, const char* user, const char* pass,
                    char *resp_buf, int size_buf);
    int enable_bearer(char *resp_buf, int size_buf);
    int network_registration_gsm(char *resp_buf, int size_buf);
    int network_registration_gprs(char *resp_buf, int size_buf);
    int check_signal_strength(char *resp_buf, int size_buf);
    uint32_t get_time(char *resp_buf, int size_buf);
    int attach_gprs(char *resp_buf, int size_buf);
    int set_apn(const char* apn, const char* user, const char* pass,
                char *resp_buf, int size_buf);
    int activate_gprs(char *resp_buf, int size_buf);
    int get_ip(char *resp_buf, int size_buf);
    int connect_tcp(const char* domain, const char* port, char *resp_buf, int size_buf);
    int close_tcp(char *resp_buf, int size_buf);
    int close_tcp_quick(char *resp_buf, int size_buf);
    int detach_gprs(char *resp_buf, int size_buf);
    int disable_bearer(char *resp_buf, int size_buf);
    int send_tcp_data(unsigned char *data, int len, char *resp_buf, int size_buf, uint8_t timeout);
    int reset(char *resp_buf, int size_buf);
    int init_sms(char *resp_buf, int size_buf);
    int check_new_sms(char *resp_buf, int size_buf);
    int get_sms(int index, char* message, int length_message);
    int send_get_request(char* url, char *resp_buf, int size_buf);
    int bt_power_on(char *resp_buf, int size_buf);
    int accept_bt(char *resp_buf, int size_buf);
    int accept_bt_pair(char *resp_buf, int size_buf);
    int send_bt_data(unsigned char *data, int len, char *resp_buf, int size_buf);
    int check_bt_host(const char *host, char *resp_buf, int size_buf);
    int change_bt_host(const char* host, char *resp_buf, int size_buf);
    int send_sms(char *number, char *data, char *resp_buf, int size_buf);
    int call_up(char *number, char *resp_buf, int size_buf);
    bool get_location(float *latitude, float *longitude, char *resp_buf, int size_buf);
    int get_active_network(char *resp_buf, int size_buf);
    int select_network(const char *network, char *resp_buf, int size_buf);

#elif defined(__ZEPHYR__)

    GPRS() {};

    int init(void);
    int test_uart(); // simple AT -> OK serial comm test
    int wakeup(void);
    int check_pin(void);
    int set_pin(const char* pin);
    int check_ssl_cert(const char *filename, int filesize);
    int load_ssl(const char *filename, const char *cert, int filesize);
    int ssl_set_cert(const char *filename);
    int enable_ssl(void);
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
    int send_tcp_data(unsigned char *data, int len, uint8_t timeout);
    int reset(void);
    int init_sms(void);
    int check_new_sms(void);
    int get_sms(int index);
    int send_get_request(char* url);
    int bt_power_on(void);
    int accept_bt(void);
    int accept_bt_pair(void);
    int send_bt_data(unsigned char *data, int len);
    int check_bt_host(const char *host);
    int change_bt_host(const char* host);
    int send_sms(char *number, char *data);
    int call_up(char *number);
    bool get_location(float *latitude, float *longitude);

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


#endif /* MBED or ZEPHYR */

    /**
     * Requests TCP data by sending the command "AT+CIPRXGET=2,100" to the modem.
     * @return Returns MODEM_RESPONSE_OK always.
     */
    int request_data(void);

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
    int delete_sms(int index);
    int answer(void);

    /**
     * This function only sends the command (AT+COPS=?) for searching the available networks.
     * Reading the response must be done separately using the read_resp() function.
     * It can take upto 3 minutes to get a response.
     * So, need to be careful about a watchdog reset in the main program.
     */
    void search_networks(void);

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
    void prepare_for_rx(int timeout, const char *ack);

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

private:
#ifdef __MBED__
    void send_cmd(const char *cmd);
#elif defined(__ZEPHYR__)
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
    void send_cmd(const char *cmd, int timeout, const char *ack);
#endif

    void clear_buffer(void);
};

#endif /* UNIT_TEST */

#endif /* __GPRS_H__ */
