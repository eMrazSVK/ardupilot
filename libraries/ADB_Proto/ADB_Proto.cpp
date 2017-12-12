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
//#include "/home/edo/ardupilot/ArduCopter/Copter.h"
extern const AP_HAL::HAL& hal;

//constructor
ADB_Proto::ADB_Proto() : ADB_ringBuf(128) {}

ADB_Proto::~ADB_Proto(){
    free(active_device_addr);
}

void ADB_Proto::init(const AP_SerialManager &serial_manager){

    int i;
    GLOBAL_ADDR_COUNT = 0;
    esc_discovery  = false;
    discovered_esc = {}; 
    esc_discovery_start = 0;
    set_active_devices = false;
    init_uart = false;
    init_uart_2 = false;
    ADB_DEVICE_COUNT = ADB_MAX_DEVICE_COUNT;
    MOTOR_COUNT = ADB_MAX_DEVICE_COUNT;
    ADB_ringBuf.set_size(128);
    sync_timer_counter_current = 0;
    sync_timer_counter_past = 0;
    good_packet_count = 0;
    bad_packet_count = 0;
    processing_packet = false;

    discovered_esc.ch_1 = false;
    discovered_esc.ch_2 = false;
    discovered_esc.ch_3 = false;
    discovered_esc.ch_4 = false;
    discovered_esc.ch_5 = false;
    discovered_esc.ch_6 = false;
    discovered_esc.ch_7 = false;
    discovered_esc.ch_8 = false;

    //Init PWM values (while discovering ESCs)
    for (i=0;i<ADB_DEVICE_COUNT;i++){
            desiredValue[i] = 0;
            active_device_addr[i] = 0;
    }

    //find serial available for ADB_Protocol and SET protocol
    if ((ADB_Port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ADB_Proto, 0))) {
        ADB_protocol = AP_SerialManager::SerialProtocol_ADB_Proto;
    }
    
    // register *ADB::Proto tick()* to scheduler - NOW called from Copter::fast_loop()
    if (ADB_Port != nullptr) {
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&ADB_Proto::tick, void));
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&ADB_Proto::msgProc, void));
        //we don't want flow control for either protocol
        //ADB_Port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);   
    }
}

void ADB_Proto::tick(void){

    //Initialize UART for ADB Protocol, it have to be initialised from thread which it is used from
    if (!init_uart) {
        if (ADB_protocol == AP_SerialManager::SerialProtocol_ADB_Proto){
            ADB_Port->begin(AP_SERIALMANAGER_ADB_Proto_BAUD, AP_SERIALMANAGER_ADB_Proto_BUFSIZE_RX, AP_SERIALMANAGER_ADB_Proto_BUFSIZE_TX); 
            init_uart = true;
            //hal.uartD->begin(AP_SERIALMANAGER_ADB_Proto_BAUD, AP_SERIALMANAGER_ADB_Proto_BUFSIZE_RX, AP_SERIALMANAGER_ADB_Proto_BUFSIZE_TX);
        }    
    }
    
    sync_timer_counter_current = AP_HAL::micros();
    
    if ((sync_timer_counter_current - sync_timer_counter_past) >= 3500) {
        //SET number of online ESCs and their addresses 
        if (set_active_devices){
            int i = 0;
            ADB_DEVICE_COUNT = 0;

            if (discovered_esc.ch_1) ADB_DEVICE_COUNT++; 
            if (discovered_esc.ch_2) ADB_DEVICE_COUNT++; 
            if (discovered_esc.ch_3) ADB_DEVICE_COUNT++; 
            if (discovered_esc.ch_4) ADB_DEVICE_COUNT++; 
            if (discovered_esc.ch_5) ADB_DEVICE_COUNT++; 
            if (discovered_esc.ch_6) ADB_DEVICE_COUNT++; 
            if (discovered_esc.ch_7) ADB_DEVICE_COUNT++; 
            if (discovered_esc.ch_8) ADB_DEVICE_COUNT++; 

            if (discovered_esc.ch_1) active_device_addr[i++] = 0;
            if (discovered_esc.ch_2) active_device_addr[i++] = 1;
            if (discovered_esc.ch_3) active_device_addr[i++] = 2;
            if (discovered_esc.ch_4) active_device_addr[i++] = 3;
            if (discovered_esc.ch_5) active_device_addr[i++] = 4;
            if (discovered_esc.ch_6) active_device_addr[i++] = 5;
            if (discovered_esc.ch_7) active_device_addr[i++] = 6;
            if (discovered_esc.ch_8) active_device_addr[i++] = 7;
            
            set_active_devices = false;
            GLOBAL_ADDR_COUNT  = 0;
            esc_discovery      = true;
        }
        
        uint16_t checksum;
        uint16_t ind;
        uint32_t bytesAvailable;

        // SCAN/Discovery connected ESCs for the first 10 seconds after boot
        if (!esc_discovery){

            if (!esc_discovery_started){
                esc_discovery_start = AP_HAL::millis();
                esc_discovery_started = true;
            }

            checksum = 0;
            ind = 0;
            bytesAvailable = 0;

            msg[ind++] = ADB_LIGHT_HEADER_BYTE;
            checksum += msg[ind++] = device_address[GLOBAL_ADDR_COUNT];
            checksum += msg[ind++] = 1;
            ind = parseToMsg(&checksum, ind);
            msg[ind++] = (~checksum) & 0x7f;
            msg[ind++] = ADB_LIGHT_END_BYTE;
            ADB_Port->write(msg, ind);

            bytesAvailable = ADB_Port->available();

            if (bytesAvailable >= 1){
                for (uint32_t k = 0; k < bytesAvailable; k++){
                    //recBuf[k] = hal.uartE->read();
                    recBuf[k] = ADB_Port->read();
                    //hal.uartD->write(recBuf[k]);
                }
            }
            
            ADB_ringBuf.write(recBuf, bytesAvailable);

            if ((AP_HAL::millis() - esc_discovery_start) > 10000){
                set_active_devices = true;
            }
        }

        // Main cycle after ESC Discovery is finished
        if ((esc_discovery) && (ADB_DEVICE_COUNT != 0)){

            checksum = 0;
            ind = 0;
            bytesAvailable = 0;

            msg[ind++] = ADB_LIGHT_HEADER_BYTE;
            checksum += msg[ind++] = active_device_addr[GLOBAL_ADDR_COUNT % ADB_DEVICE_COUNT];
            checksum += msg[ind++] = message_ids[GLOBAL_ID_COUNT];

            // Set desired PWM value equal to RCOUT
            switch (active_device_addr[GLOBAL_ADDR_COUNT % ADB_DEVICE_COUNT]) {
                case 0x00: 
                    desiredValue[active_device_addr[GLOBAL_ADDR_COUNT % ADB_DEVICE_COUNT]] = hal.rcout->read(CH_1);
                    break;
                case 0x01:
                    desiredValue[active_device_addr[GLOBAL_ADDR_COUNT % ADB_DEVICE_COUNT]] = hal.rcout->read(CH_2);
                    break;
                case 0x02:
                    desiredValue[active_device_addr[GLOBAL_ADDR_COUNT % ADB_DEVICE_COUNT]] = hal.rcout->read(CH_3); 
                    break;
                case 0x03:
                    desiredValue[active_device_addr[GLOBAL_ADDR_COUNT % ADB_DEVICE_COUNT]] = hal.rcout->read(CH_4); 
                    break;
                case 0x04: 
                    desiredValue[active_device_addr[GLOBAL_ADDR_COUNT % ADB_DEVICE_COUNT]] = hal.rcout->read(CH_5);
                    break;
                case 0x05:
                    desiredValue[active_device_addr[GLOBAL_ADDR_COUNT % ADB_DEVICE_COUNT]] = hal.rcout->read(CH_6);
                    break;
                case 0x06:
                    desiredValue[active_device_addr[GLOBAL_ADDR_COUNT % ADB_DEVICE_COUNT]] = hal.rcout->read(CH_7);
                    break;
                case 0x07:
                    desiredValue[active_device_addr[GLOBAL_ADDR_COUNT % ADB_DEVICE_COUNT]] = hal.rcout->read(CH_8);
                    break;
                default:
                    return;
            }

            //Send message to ESC
            ind = parseToMsg(&checksum, ind);  
            msg[ind++] = (~checksum) & 0x7f;
            msg[ind++] = ADB_LIGHT_END_BYTE;
            
            ADB_Port->write(msg, ind);

            bytesAvailable = ADB_Port->available();

            // Read Available bytes from UART aand write it to RingBuffer
            if (bytesAvailable >= 1) {
                
                for (uint32_t j = 0; j < bytesAvailable; j++){
                    //recBuf[j] = hal.uartE->read();
                    recBuf[j] = ADB_Port->read();
                  //hal.uartD->write(recBuf[j]); 
                }
                ADB_ringBuf.write(recBuf, bytesAvailable);
            }
            
        }
            
        GLOBAL_ADDR_COUNT++;
        
        // Change ESC addresses and MSGs cyclically
        if ((GLOBAL_ADDR_COUNT % ADB_DEVICE_COUNT) == 0){
            GLOBAL_ADDR_COUNT = 0;

            if ((GLOBAL_ID_COUNT % ADB_MESSAGE_ID_COUNT) == 0){
                GLOBAL_ID_COUNT = 0;
            }

            GLOBAL_ID_COUNT++;
        }

        sync_timer_counter_past = AP_HAL::micros();
    }
}    


int ADB_Proto::parseToMsg(uint16_t *checksum, uint16_t actualIndex){
	uint16_t count = BIT_LEN_VALUE;
	uint8_t index = MOTOR_HIGH_ADDRESS;
	uint32_t res = 0;
	res |= desiredValue[index];
	*checksum += msg[DATA_BYTES + actualIndex - 1] = (res & ADB_LookUpTableMask[USED_BITS_OF_LAST_BYTE]) << EMPTY_BITS_OF_LAST_BYTE;
	res >>= USED_BITS_OF_LAST_BYTE;
    count -= USED_BITS_OF_LAST_BYTE;
    
	for(int16_t k = DATA_BYTES + actualIndex - 2; k >= actualIndex; k--) {
		if(count < DATA_BITS_OF_BYTE)
		{
			uint32_t act = 0;
			act |= desiredValue[--index];
			act &= ADB_LookUpTableMask[BIT_LEN_VALUE];
			res |= (act << count);
			count += BIT_LEN_VALUE;
		}
		*checksum += msg[k] = res & ADB_LookUpTableMask[DATA_BITS_OF_BYTE];
		res >>= DATA_BITS_OF_BYTE;
		count -= DATA_BITS_OF_BYTE;
	}

	return actualIndex + DATA_BYTES;
}

void ADB_Proto::msgProc(){
   
    if (!init_uart_2){
        //hal.uartD->begin(115200, AP_SERIALMANAGER_ADB_Proto_BUFSIZE_RX, AP_SERIALMANAGER_ADB_Proto_BUFSIZE_TX);
        hal.uartA->begin(115200, AP_SERIALMANAGER_ADB_Proto_BUFSIZE_RX, AP_SERIALMANAGER_ADB_Proto_BUFSIZE_TX);        
        init_uart_2 = true;
    }
    uint8_t procByte;
    uint32_t ringBuf_available;

    ringBuf_available = ADB_ringBuf.available();

    if (ringBuf_available >= 1){
        //Read all available bytes from RingBuffer
        for (uint32_t i = 0; i < ringBuf_available; i++){
            ADB_ringBuf.read_byte(&procByte);
            //hal.uartA->write(procByte);
            if (startRec && (procByte == 0x8e)){
                startRec = false;
            }
            
            if (startRec){
                recMsgBuf[recIndex++] = procByte;
                
                if (recIndex == ADB_REC_MSG_LEN){
                    recIndex = 0;
                    startRec = false;

                    //Unpack received msg/value
                    prepareMsg();
                }
            }
            if (procByte == 0x8f){
                startRec = true;
            }
        }
    }
}


/* UNUSED
void ADB_Proto::recByteProc(uint8_t received){
    recMsgBuf[recIndex++] = received;
    if (recIndex == 4){
        recIndex = 0;
        prepareMsg();
    }
    
	if((received & 0x80) != 0 && (received & 0x0f) <= 0x0d)
	{
		recIndex = 0;
		startRec = 1;
		recMsgBuf[recIndex++] = received;
        //hal.uartD->write(68); //D
        
	}
	if(startRec)
	{
		recMsgBuf[recIndex++] = received;
		if(recIndex == 4)
		{
            //hal.uartD->write(69); //E 
			startRec = 0;
			prepareMsg();
		}
	}
    
}
*/

void ADB_Proto::prepareMsg(){
	uint8_t chcksum = 0;
	uint32_t value = 0;
	int8_t deviceAddress = -1;
	int8_t messageId = -1;
    deviceAddress = (recMsgBuf[0] >> 4) & 0x07;
    //hal.uartA->write(recMsgBuf[0]);
	messageId = recMsgBuf[0] & 0x0f;
    chcksum ^= value = (recMsgBuf[3] >> 4) & 0x07;

    uint8_t temp_buf[4];

	for(uint8_t i = 2 ; i > 0; i--){
		value <<= 7;
		value |= recMsgBuf[i] & 0x7f;
		chcksum ^= (recMsgBuf[i] >> 4) & 0x07;
		chcksum ^= recMsgBuf[i] & 0x0f;
	}

	chcksum = (~chcksum) & 0x0f;

    //Save which ESCs are responding
    if((chcksum == (recMsgBuf[3] & 0x0f)) && !esc_discovery){ 

        switch (deviceAddress) {
            case 0:
                discovered_esc.ch_1 = true;
                break;
            case 1:
                discovered_esc.ch_2 = true;
                break;
            case 2:
                discovered_esc.ch_3 = true;
                break;
            case 3:
                discovered_esc.ch_4 = true;
                break;
            case 4:
                discovered_esc.ch_5 = true;
                break;
            case 5:
                discovered_esc.ch_6 = true;
                break;
            case 6:
                discovered_esc.ch_7 = true;
                break;
            case 7:
                discovered_esc.ch_8 = true;
                break;
        }
    }

    processing_packet = false;
    //Save value received from ESC
    if((chcksum == (recMsgBuf[3] & 0x0f)) && (esc_discovery)){ 
        tmp_log.deviceAddr = deviceAddress;
        
        processing_packet = true;

        tmp_log.good_msgs = good_packet_count;
        tmp_log.bad_msgs = bad_packet_count;

        uint32_t tmp_mask = 0;

        int i;
        bool isGreaterThanZero;

        float val0, val1, val2, val3;
        uint8_t *p = (uint8_t*)&value; 
    
        hal.uartA->write(0x0e);
        hal.uartA->write(messageId);
        hal.uartA->write(p, 4);
        hal.uartA->write(0x0f);
        
        switch (messageId)
            {
                case 0x01:
                {
                    for (i = 1; i < 17; i++){
                        tmp_mask |= 1 << i;
                    }
                    value &= tmp_mask;
                    val0 = value/1024.0f;

                    if (0 & (1 << value)) isGreaterThanZero = true;
                    else isGreaterThanZero = false;

                    if (isGreaterThanZero) tmp_log.speed = -val0;
                    else if (!isGreaterThanZero) tmp_log.speed = val0;
                }
                    break;
                case 0x02: 
                {
                    for (i = 1; i < 11; i++){
                        tmp_mask |= 1 << i;
                    }
                    value &= tmp_mask;
                    val1 = value/8.0f;

                    if (0 & (1 << value)) isGreaterThanZero = true;
                    else isGreaterThanZero = false;
  
                    if (isGreaterThanZero) tmp_log.voltage_s = -val1;
                    else if (!isGreaterThanZero) tmp_log.voltage_s = val1;
                }
                    break;
                case 0x03: 
                {
                    for (i = 1; i < 12; i++){
                        tmp_mask |= 1 << i;
                    }
                    value &= tmp_mask;
                    val3 = value/8.0f;

                    if (0 & (1 << value)) isGreaterThanZero = true;
                    else isGreaterThanZero = false;
                    
                    if (isGreaterThanZero) tmp_log.current_s = -val3;
                    else if (!isGreaterThanZero) tmp_log.current_s = val3;
                }
                    break;
                case 0x04: 
                {
                    for (i = 0; i < 10; i++){
                        tmp_mask |= 1 << i;
                    }
                    value &= tmp_mask;
                    val2 = value/8.0f;
                    tmp_log.v_bus = val2;
                }
                    break;
                case 0x05:
                {
                    for (i = 0; i < 12; i++){
                        tmp_mask |= 1 << i;
                    }
                    value &= tmp_mask;
                    tmp_log.pwm = value;
                }
                    break;
                case 0x06:
                {
                    for (i = 0; i < 7; i++){
                        tmp_mask |= 1 << i;
                    }
                    value &= tmp_mask;
                    tmp_log.temp = value - 0x1f;
                }
                    break;
                default:
                    return;
            } 

    }

    if (esc_discovery){
        if (!processing_packet) bad_packet_count++;
        else if (processing_packet) good_packet_count++;
    }
    
    
}
