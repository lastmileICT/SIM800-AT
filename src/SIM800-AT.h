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

#ifndef __GPRS_H__
#define __GPRS_H__

#ifndef UNIT_TEST

#include <stdio.h>
#include <stdint.h>
#include <string>
#include "mbed.h"

#define DEFAULT_TIMEOUT 3

/** GPRS class.
 *  Used for mobile communication. attention that GPRS module communicate with MCU in serial protocol
 */
class GPRS
{
public:
    /** Create GPRS instance
     *  @param tx  uart transmit pin to communicate with GPRS module
     *  @param rx  uart receive pin to communicate with GPRS module
     *  @param ri  ring indicator goes low for 120ms when an SMS is received
     *  @param baudRate baud rate of uart communication
     */
    GPRS(PinName tx, PinName rx, PinName ri, int baudRate) : gprsSerial(tx, rx) {};

    Serial gprsSerial;

    int request_data();

    /** 
     * Reads the server response through serial port. The function is used in 2 ways. 
     * 1) To read-out the buffer, in which case, no return is actually expected.
     * 2) To read-out and also check the response against an expected message.
     * @param resp_buf Pointer to the buffer that will store the received data
     * @param size_buf Size of the receive buffer. This must be set carefully to
     * check the acknowledgement message correctly.
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
    int enable_ssl(const char *filename, char *resp_buf, int size_buf);
    int setup_clock(char *resp_buf, int size_buf);

    /**
     * Bearer Settings for Applications Based on IP. This configures the Access Point Name (APN) 
     * parameters for internet access. The function sends the AT command "AT+SAPBR=3,......" 
     * to the GSM modem.
     * @param apn APN for the internet gateway
     * @param user User ID for APN
     * @param pass Password for APN
     * @param resp_buf Pointer to the buffer that will store the received data
     * @param size_buf Size of the receive buffer
     * @return Returns -1 if invalid or no response from the modem
     */
    int setup_bearer(const char* apn, const char* user, const char* pass, 
                        char *resp_buf, int size_buf);

    /**
     * Enables/opens the bearer, provided the settings have been done. 
     * The function sends the AT command "AT+SAPBR=1,.." to the GSM modem.
     * @param resp_buf Pointer to the buffer that will store the received data
     * @param size_buf Size of the receive buffer
     * @return Returns -1 if invalid or no response from the modem
     */
    int enable_bearer(char *resp_buf, int size_buf);

    /**
     * Check GSM registration.
     * The function sends the AT command "AT+CREG?" to the GSM modem.
     * @param resp_buf Pointer to the buffer that will store the received data
     * @param size_buf Size of the receive buffer
     * @return Returns -1 if invalid or no response from the modem
     */
    int network_registration_gsm(char *resp_buf, int size_buf);

    /**
     * Check packet switched registration.
     * The function sends the AT command "AT+CGREG?" to the GSM modem.
     * @param resp_buf Pointer to the buffer that will store the received data
     * @param size_buf Size of the receive buffer
     * @return Returns -1 if invalid or no response from the modem
     */
    int network_registration_gprs(char *resp_buf, int size_buf);

    /**
     * The function sends the AT command "AT+CSQ" to the GSM modem and gets
     * the signal strength value (RSSI) as the response.
     * @param resp_buf Pointer to the buffer that will store the received data
     * @param size_buf Size of the receive buffer
     * @return Returns -1 if invalid RSSI or no response from the modem
     */
    int check_signal_strength(char *resp_buf, int size_buf);
    uint32_t get_time(char *resp_buf, int size_buf);
    int attach_gprs(char *resp_buf, int size_buf);
    int enable_get_data_manually(char *resp_buf, int size_buf);

    /**
     * Start Task and Set APN, USER NAME, PASSWORD.
     * The function sends the AT command "AT+CSTT=apn,user,pass" to the GSM modem.
     * @param apn APN details
     * @param user User ID
     * @param pass Password
     * @param resp_buf Pointer to the buffer that will store the received data
     * @param size_buf Size of the receive buffer
     * @return Returns -1 if invalid or no response from the modem, 0 if success.
     */
    int set_apn(const char* apn, const char* user, const char* pass, 
                char *resp_buf, int size_buf);

    /**
     * Bring Up Wireless Connection with GPRS.
     * The function sends the AT commands "AT+CIICR" to the GSM modem.
     * @param resp_buf Pointer to the buffer that will store the received data
     * @param size_buf Size of the receive buffer
     * @return Returns 0 if the modem responds with "OK".
     * Returns -1 if no response or invalid.
     */
    int activate_gprs(char *resp_buf, int size_buf);

    /**
     * Get Local IP address, if the PDP context has been activated before.
     * The function sends the AT commands "AT+CIFSR" to the GSM modem.
     * @param resp_buf Pointer to the buffer that will store the received data
     * @param size_buf Size of the receive buffer
     * @return Returns 0 if the modem response is a valid IP address. Returns -1 if the GSM modem 
     * responds with an "ERROR". This can happen if the PDP context has not been activated already.
     */
    int get_ip(char *resp_buf, int size_buf);

    /**
     * Opens TCP connection sending the AT command "AT+CIPSTART=TCP,domain,port" to the GSM modem.
     * Expects a "CONNECT OK" or "ALREADY CONNECT" from the modem on successful connection.
     * @param domain  Remote server domain name
     * @param port Remote server port
     * @param resp_buf Pointer to the buffer that will store the received data
     * @param size_buf Size of the receive buffer
     * @return Returns -1 if invalid or no response from the modem
     */
    int connect_tcp(const char* domain, const char* port, char *resp_buf, int size_buf);

    /**
     * Closes the TCP connection.
     * The function sends the AT command "AT+CIPCLOSE=0" to the GSM modem.
     * The modem may return ERROR if TCP is not already opened.
     * @param resp_buf Pointer to the buffer that will store the received data
     * @param size_buf Size of the receive buffer
     * @return Returns -1 on receiving an ERROR or invalid response.
     * Returns 0 if CLOSE OK is received from the modem.
     */
    int close_tcp(char *resp_buf, int size_buf);


    /**
     * Quickly closes the TCP connection.
     * The function sends the AT command "AT+CIPCLOSE=1" to the GSM modem.
     * The modem may return ERROR if TCP is not already opened.
     * @param resp_buf Pointer to the buffer that will store the received data
     * @param size_buf Size of the receive buffer
     * @return Returns -1 on receiving an ERROR or invalid response.
     * Returns 0 if CLOSE OK is received from the modem.
     */
    int close_tcp_quick(char *resp_buf, int size_buf);

    /**
     * Closes the GPRS PDP context.
     * The function sends the AT command "AT+CIPSHUT" to the GSM modem.
     */
    void close_pdp_context(void);

    int detach_gprs(char *resp_buf, int size_buf);
    int disable_bearer(char *resp_buf, int size_buf);
    void sleep(void);
    int send_tcp_data(unsigned char *data, int len, char *resp_buf, int size_buf);
    int check_ssl_cert(const char *filename, int filesize, char *resp_buf, int size_buf);
    int load_ssl(const char *filename, const char *cert, int filesize, char *resp_buf, int size_buf);
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
    int delete_sms(int index);
    int answer(void);
    int call_up(char *number, char *resp_buf, int size_buf);

    bool get_location(float *latitude, float *longitude, char *resp_buf, int size_buf);

    /**
     * This function only sends the command (AT+COPS=?) for searching the available networks.
     * Reading the response must be done separately using the read_resp() function.
     * It can take upto 3 minutes to get a response.
     * So, need to be careful about a watchdog reset in the main program.
     */
    void search_networks(void);

    /**
     * Function to manually switch to a specific available network.
     * This function sends the command (AT+COPS=1,1,"OperatorShortName").
     * This can take upto 3 minutes to get a response. 
     * So, need to be careful about a watchdog reset in the main program.
     * Immediate response-check is also handled in the function.
     * @param resp_buf Pointer to the buffer that will store the received data
     * @param size_buf Size of the receive buffer
     * @return Returns -1 if invalid or no response from the modem
     */
    int select_network(const char *network, char *resp_buf, int size_buf);

private:
    void send_cmd(const char *cmd);
    void clear_buffer(void);
};

#endif

#endif /* UNIT_TEST */
