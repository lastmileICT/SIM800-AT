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
    int enable_bearer(const char* apn, const char* user, const char* pass, 
                        char *resp_buf, int size_buf);
    int network_registration(char *resp_buf, int size_buf);

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
    int set_apn(const char* apn, const char* user, const char* pass, 
                char *resp_buf, int size_buf);
    int activate_gprs(char *resp_buf, int size_buf);
    int get_ip(char *resp_buf, int size_buf);
    int connect_tcp(const char* ip, const char* port, char *resp_buf, int size_buf);
    int close_tcp(char *resp_buf, int size_buf);
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

    int send_sms(char *number, char *data, char *resp_buf, int size_buf);
    int delete_sms(int index);
    int answer(void);
    int call_up(char *number, char *resp_buf, int size_buf);

    bool get_location(float *latitude, float *longitude, char *resp_buf, int size_buf);

    /** This function only sends the command for searching the available networks.
     * Reading the response must be done separately using the read_resp() function.
     * It can take upto 3 minutes to get a response.
     * So, need to be careful about a watchdog reset in the main program.
     */
    void search_networks(void);
    int select_network(char *network, char *resp_buf, int size_buf);

private:
    void send_cmd(const char *cmd);
    void clear_buffer(void);
};

#endif

#endif /* UNIT_TEST */
