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

#include <stdio.h>
#include <stdint.h>
#include <string>
#include "mbed.h"

#define DEFAULT_TIMEOUT         3
#define SMS_MAX_LENGTH          160


enum GPRS_MESSAGE {
    MESSAGE_RING = 0,
    MESSAGE_SMS  = 1,
    MESSAGE_ERROR
};


/** GPRS class.
 *  Used for mobile communication. attention that GPRS module communicate with MCU in serial protocol
 */
class GPRS
{
public:
    /** Create GPRS instance
     *  @param tx  uart transmit pin to communicate with GPRS module
     *  @param rx  uart receive pin to communicate with GPRS module
     *  @param baudRate baud rate of uart communication
     *  @param number default phone number during mobile communication
     */
    GPRS(PinName tx, PinName rx, int baudRate,char *number) : gprsSerial(tx, rx) {
        phoneNumber = number;
    };

    Serial gprsSerial;

    int request_data();
    int read_resp(char *buffer, int count);
    int init();
    int wakeup();
    int check_pin();
    int set_pin(const char* pin);
    int enable_SSL(const char *filename);
    int setup_clock();
    int enable_bearer(const char* apn, const char* user, const char* pass);
    int network_availablity();
    int check_signal_strength();
    uint32_t get_time();
    int attach_gprs();
    int set_apn(const char* apn, const char* user, const char* pass);
    int activate_gprs();
    int get_ip();
    int connect_tcp(const char* ip, const char* port);
    int close_tcp();
    int detach_gprs();
    int disable_bearer();
    void sleep();
    int send_tcp_data(unsigned char *data, int len);
    int load_ssl(const char *filename, const char *cert, int filesize);
    int check_ssl_cert(const char *filename, int filesize);
    int init_SMS();
    int check_unread_SMS();
    int reset();

    bool get_location(float *latitude, float *longitude);

    int check_new_SMS();
    int settingSMS();
    int sendSMS(char *number, char *data);
    int deleteSMS(int index);
    int getSMS(char* message);
    int callUp(char *number);
    int answer(void);
    int loopHandle(void);

private:
    void send_cmd(const char *cmd);
    int check_resp(const char *resp, int timeout);
    void clear_buffer();

    char *phoneNumber;
    char messageBuffer[SMS_MAX_LENGTH];
};

#endif