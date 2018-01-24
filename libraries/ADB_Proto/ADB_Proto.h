/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>


#define ADB_MSG_LENGTH          42
#define ADB_MAX_DEVICE_COUNT    8
#define ADB_REC_BUF_SIZE        256
#define ADB_REC_MSG_LEN         4
#define BIT_LEN_VALUE 			16
#define MAX_MOTOR_COUNT			ADB_MAX_DEVICE_COUNT
#define MOTOR_HIGH_ADDRESS 		(MOTOR_COUNT-1)
#define DATA_BITS_OF_BYTE		7
#define USED_BITS_OF_LAST_BYTE 	((BIT_LEN_VALUE * MOTOR_COUNT) % DATA_BITS_OF_BYTE)
#define EMPTY_BITS_OF_LAST_BYTE (DATA_BITS_OF_BYTE - USED_BITS_OF_LAST_BYTE)
#define DATA_BYTES				(((BIT_LEN_VALUE * MOTOR_COUNT) / DATA_BITS_OF_BYTE) + ((USED_BITS_OF_LAST_BYTE == 0)?0:1))
#define ADB_LIGHT_HEADER_BYTE   0x8e
#define ADB_LIGHT_END_BYTE      0x8f
#define ADB_MESSAGE_ID_COUNT    6
#define SEND_EVERY_NTH_TIME     1
#define RECEIVE_TIMEOUT         2500
#define CONNECTION_TIMEOUT_MS   5000


typedef struct {
  int8_t    deviceAddr;
  float     speed;
  float     voltage_s;
  float     current_s;
  float     v_bus;
  uint32_t  pwm;
  uint32_t  temp;
  uint32_t good_msgs;
  uint32_t bad_msgs;
} internal_log_msg;

typedef struct {
    bool ch_1;
    bool ch_2;    
    bool ch_3;
    bool ch_4;
    bool ch_5;
    bool ch_6;
    bool ch_7;
    bool ch_8;
} active_esc;

class ADB_Proto {
    public:
        ADB_Proto();
        ~ADB_Proto();
        void init(const AP_SerialManager &serial_manager);
        internal_log_msg tmp_log;
        bool esc_disconnected;
        bool errorCheck();

    private:
        void tick(void);
        void test_tick(void);
        void test_msgProc();
        int GLOBAL_ADDR_COUNT;
        int GLOBAL_ID_COUNT;
        void msgProc();
        //port used for ADB Protocol 
        AP_HAL::UARTDriver *ADB_Port;
        AP_SerialManager::SerialProtocol ADB_protocol; 
        bool init_uart;
        uint16_t desiredValue[8]; // range 800 - 2000 ms
        int parseToMsg(uint16_t *checksum, uint16_t actualIndex);
        void recByteProc(uint8_t received);
        void prepareMsg();
        void discoverEsc(uint32_t discovery_time);
        void calc_esc_responses();
        uint32_t errorCount;
        uint32_t lastErrorTime;
        uint8_t msg[ADB_MSG_LENGTH];
        uint8_t recBuf[ADB_REC_BUF_SIZE];
        uint32_t current_msg_size;
        uint8_t count_to_n;
        uint8_t recMsgBuf[ADB_REC_MSG_LEN];
        uint32_t discover_time_ms;
        int recIndex;
		int startRec;
        int ADB_DEVICE_COUNT;
        int MOTOR_COUNT;
        int8_t active_device_addr[ADB_MAX_DEVICE_COUNT];
        bool active_channels[ADB_MAX_DEVICE_COUNT];
        uint16_t ADB_LookUpTableMask[17] = {
								0x0000, 0x0001, 0x0003, 0x0007,
								0x000f, 0x001f, 0x003f, 0x007f,
								0x00ff, 0x01ff, 0x03ff, 0x07ff,
								0x0fff, 0x1fff, 0x3fff, 0x7fff,
								0xffff
							    };
        uint8_t device_address[8] = {0, 1, 2, 3, 4, 5, 6, 7};  
        uint8_t message_ids[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
        bool esc_discovery;
        bool esc_discovery_started;
        bool set_active_devices;
        bool STOP_SEND;
        uint32_t esc_discovery_start;
        uint32_t last_discovery_time_ms;
        active_esc discovered_esc;
        uint32_t good_packet_count;
        uint32_t bad_packet_count;
        bool processing_packet;
        int status_counter;
        uint32_t esc_responses_ms[ADB_MAX_DEVICE_COUNT];
}; 

