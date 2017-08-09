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
#include "/home/edo/ardupilot/ArduCopter/GCS_Copter.h"
#include "/home/edo/ardupilot/ArduCopter/GCS_Mavlink.h"

struct ADB_Frame {
        int8_t START;
        int8_t ADDR;
        int8_t MSG_ID;
        int8_t DATA[37];
        int8_t CHECKSUM;
        int8_t END;
    };

class ADB_Proto {

    public:
        ADB_Proto();
        //send to all ESCs
        void send_frame_to_esc(ADB_Frame frame);
        void init(const AP_SerialManager &serial_manager);
        void tick(void);
        int8_t getCheckSum();

    
    private:
        //port used for ADB Protocol 
        AP_HAL::UARTDriver *ADB_Port;
        AP_SerialManager::SerialProtocol ADB_protocol; 
        bool init_uart;
        //Frame to be sent
        ADB_Frame frame;
        int8_t checksum_calc();  
        void sendEscData(); 
        void send_frame(ADB_Frame frame);
        GCS_Copter _gcs; // avoid using this; use gcs()
        GCS_Copter &gcs() { return _gcs; }

}; 

