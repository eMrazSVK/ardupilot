/*

   Inspired by FrSky telemetry protocol

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

/*
    ADB Protocol library
*/

#include "ADB_Proto.h"

extern const AP_HAL::HAL& hal;

//constructor
ADB_Proto::ADB_Proto(){};


void ADB_Proto::init(const AP_SerialManager &serial_manager){
/* HANDLED IN tick()
    frame.START = 0;
    frame.END = 0;
    frame.ADDR = 0;
    frame.MSG_ID = 0;
    
    //Setting bits of frame header
    frame.START |= (1 << 7);
    frame.START |= (1 << 1);
    frame.START |= (1 << 3);
    
    frame.END |= (1 << 7);
    frame.END |= (1 << 0);
    frame.END |= (1 << 1);
    frame.END |= (1 << 2);
    frame.END |= (1 << 3);   
    */
    int i;
    
    //fill DATA with default value (temp solution)
        for (i=0;i<16;i++){
            desiredValue[i] = 800;
        }
        
    //assign checksum value to frame
    frame.CHECKSUM = checksum_calc(); 
    

    //find serial available for ADB_Protocol and SET protocol
    if ((ADB_Port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ADB_Proto, 0))) {
        ADB_protocol = AP_SerialManager::SerialProtocol_ADB_Proto;
    }
    
    //register *ADB::Proto tick()* to scheduler
    if (ADB_Port != nullptr) {
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&ADB_Proto::tick, void));
        
        //we don't want flow control for either protocol
        ADB_Port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
         
    }   
      
}

void ADB_Proto::tick(void){
    
    if (!init_uart) {
        if (ADB_protocol == AP_SerialManager::SerialProtocol_ADB_Proto){
            ADB_Port->begin(AP_SERIALMANAGER_ADB_Proto_BAUD, AP_SERIALMANAGER_ADB_Proto_BUFSIZE_RX, AP_SERIALMANAGER_ADB_Proto_BUFSIZE_TX); 
            hal.uartE->begin(AP_SERIALMANAGER_ADB_Proto_BAUD, AP_SERIALMANAGER_ADB_Proto_BUFSIZE_RX, AP_SERIALMANAGER_ADB_Proto_BUFSIZE_TX);
            init_uart = true;
        }
    }

    if (send == 0){
        send = 1;
    }
    else{
        send = 0;
    }

    int i;
    //DAVID, povodne baseTimerHandler
    uint16_t checksum = 0;
	msg[0] = 0x8a;
	checksum += msg[1] = 0; //deviceAddress;
	checksum += msg[2] = 0; //messageId;

    for (i=0;i<16;i++){
        desiredValue[i] += 1;
    }

     if (desiredValue[0] > 2000){
         for (i = 0;i < 16;i++){
             desiredValue[i] = 800;
         }
     }
	parseToMsg(&checksum);
	msg[40] = (~checksum) & 0x7f;
	msg[41] = 0x8f;
	//usart1_putbDma(msg, 42); - nahradzuje hal.uartE->write(const uint8_t *buffer, size_t size)

    if (AP_HAL::millis() % 100 == 0){
        hal.uartE->write(msg,42);
    }
    
    
}    

void ADB_Proto::parseToMsg(uint16_t *checksum)
{
	uint16_t count = 16;
	uint8_t index = 15;
	uint32_t res = 0;
	res |= desiredValue[index];
	*checksum += msg[39] = (res & 0x0f) << 3;
	res >>= 4;
	count -= 4;
	for(int16_t k = 38; k >= 3; k--)
	{
		if(count < 7)
		{
			uint32_t act = 0;
			act |= desiredValue[--index];
			act &= 0xffff;
			res |= (act << count);
			count += 16;
		}
		*checksum += msg[k] = res & 0x7f;
		res >>= 7;
		count -= 7;
	}
}



void ADB_Proto::sendEscData(){
    int16_t numc;
    numc = ADB_Port->available();

    // check if available is negative
    if (numc < 0) {
        return;
    }

    // this is the constant for hub data frame
    if (ADB_Port->txspace() < 19) {
        return;
    } 


}


void ADB_Proto::send_frame(ADB_Frame _frame)
{
    if (ADB_protocol == AP_SerialManager::SerialProtocol_ADB_Proto) { // ADB Proto
        //ADB_Port->write(0x5D); 
        hal.uartE->printf("umri ak nejdes \n");
    } 
}

int8_t ADB_Proto::checksum_calc(){   
        int8_t tmp_chck;
        int i;
        
        tmp_chck = frame.DATA[0];
        
            for (i=1;i<37;i++){
                tmp_chck ^= frame.DATA[i];
            }  
            
        return ~tmp_chck;
}


int8_t ADB_Proto::getCheckSum(){
    return frame.CHECKSUM;
}

