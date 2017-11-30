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
#include <AP_HAL/utility/RingBuffer.h>


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


typedef struct {
  int8_t    deviceAddr;
  float     speed;
  float     voltage_s;
  float     current_s;
  float     v_bus;
  uint32_t  pwm;
  uint32_t  temp;
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
        void tick(void);
        int GLOBAL_ADDR_COUNT;
        int GLOBAL_ID_COUNT;
        void msgProc();
        internal_log_msg tmp_log;
        
    private:
        //port used for ADB Protocol 
        AP_HAL::UARTDriver *ADB_Port;
        AP_SerialManager::SerialProtocol ADB_protocol; 
        bool tmp;
        bool init_uart;
        bool init_uart_2;
        uint16_t desiredValue[8]; // range 800 - 2000 ms
        uint32_t sync_timer_counter_current;
        uint32_t sync_timer_counter_past;
        int parseToMsg(uint16_t *checksum, uint16_t actualIndex);
        void recByteProc(uint8_t received);
        void prepareMsg();
        uint8_t msg[ADB_MSG_LENGTH];
        uint8_t recBuf[ADB_REC_BUF_SIZE];
        uint8_t recMsgBuf[ADB_REC_MSG_LEN];
        int recIndex;
		int startRec;
        int ADB_DEVICE_COUNT;
        int MOTOR_COUNT;
        int8_t active_device_addr[ADB_MAX_DEVICE_COUNT];
        ByteBuffer ADB_ringBuf;
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
        uint32_t esc_discovery_start;
        active_esc discovered_esc;
}; 

