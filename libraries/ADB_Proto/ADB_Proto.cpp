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

ADB_Proto::ADB_Proto() {}

ADB_Proto::~ADB_Proto() {}

void ADB_Proto::init(const AP_SerialManager &serial_manager) {

    GLOBAL_ADDR_COUNT = 0;
    esc_discovery  = false;
    discovered_esc = {}; 
    esc_discovery_start = 0;
    set_active_devices = false;
    init_uart = false;
    ADB_DEVICE_COUNT = ADB_MAX_DEVICE_COUNT;
    MOTOR_COUNT = ADB_MAX_DEVICE_COUNT;
    good_packet_count = 0;
    bad_packet_count = 0;
    processing_packet = false;
    current_msg_size = 28;
    count_to_n = 0;
    STOP_SEND = false;
    status_counter = 0;
    discover_time_ms = 10000;
    last_discovery_time_ms = 0;

    discovered_esc.ch_1 = false;
    discovered_esc.ch_2 = false;
    discovered_esc.ch_3 = false;
    discovered_esc.ch_4 = false;
    discovered_esc.ch_5 = false;
    discovered_esc.ch_6 = false;
    discovered_esc.ch_7 = false;
    discovered_esc.ch_8 = false;

    // Init PWM values (while discovering ESCs)
    for (int i = 0; i < ADB_DEVICE_COUNT; i++) {
            desiredValue[i] = 0;
            active_device_addr[i] = 0;
            esc_responses_ms[i] = 0;
    }

    // find serial available for ADB_Protocol and SET protocol
    if ((ADB_Port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ADB_Proto, 0))) {
        ADB_protocol = AP_SerialManager::SerialProtocol_ADB_Proto;
        //ADB_Port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_ENABLE);
    }

    // register *ADB::Proto tick()* to timer thread
    if (ADB_Port != nullptr) {
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&ADB_Proto::tick, void));
    }
}

/*
void ADB_Proto::test_tick(void) {
     //Initialize UART for ADB Protocol, it have to be initialised from thread which it is used from  
    if (!init_uart) {
        if (ADB_protocol == AP_SerialManager::SerialProtocol_ADB_Proto){
            ADB_Port->begin(AP_SERIALMANAGER_ADB_Proto_BAUD, AP_SERIALMANAGER_ADB_Proto_BUFSIZE_RX, AP_SERIALMANAGER_ADB_Proto_BUFSIZE_TX); 
            init_uart = true;
        }
    }
    uint16_t checksum;
    uint16_t ind;

    sync_timer_counter_current = AP_HAL::millis();
    //writing to serial
   
    if (!STOP_SEND) {
        //if ((AP_HAL::micros() - sync_timer_counter_past) >= 3000) {
            if (test_counter == 2) ADB_Port->flush_tx_buffer();
            
            if (test_counter == 3) {
                if (!ADB_Port->tx_pending()) {
                    current_msg_size = 0;
                    checksum = 0;
                    ind = 0;
                    msg[ind++] = ADB_LIGHT_HEADER_BYTE;
                    
                    if((count_to_n % SEND_EVERY_NTH_TIME) == 0) checksum += msg[ind++] = 0;//device_address[GLOBAL_ADDR_COUNT % ADB_DEVICE_COUNT];
                    // send request only every nth time
                    else if((count_to_n % SEND_EVERY_NTH_TIME) != 0) checksum += msg[ind++] = 0x7f;
                    
                    checksum += msg[ind++] = message_ids[GLOBAL_ID_COUNT];
                    ind = parseToMsg(&checksum, ind);
                    msg[ind++] = (~checksum) & 0x7f;
                    msg[ind++] = ADB_LIGHT_END_BYTE;
                    ADB_Port->write(msg, ind);        
                    sync_timer_counter_past = AP_HAL::micros();
                    count_to_n++;
                    test_counter = 0;
                    // Change ESC addresses and MSGs cyclically
                    if ((count_to_n % SEND_EVERY_NTH_TIME) == 0) {
                        count_to_n = 0;
                        GLOBAL_ADDR_COUNT++;
                        if ((GLOBAL_ADDR_COUNT % ADB_DEVICE_COUNT) == 0) {
                            GLOBAL_ADDR_COUNT = 0;

                            if ((GLOBAL_ID_COUNT % ADB_MESSAGE_ID_COUNT) == 0) {
                                GLOBAL_ID_COUNT = 0;
                            }
                            GLOBAL_ID_COUNT++;
                        }
                    }
                } else {
                    test_counter = 3;
                    ADB_Port->flush_tx_buffer();   
                  }
            } else test_counter++;
    // }
    } 
        
        uint32_t j;
        j = 0;
        while (ADB_Port->available() > 0) {
            recBuf[j++] = ADB_Port->read();
        }
        test_msgProc();
        
        //reading from serial
        
        if (AP_HAL::millis() > 1) {
            current_msg_size = ADB_Port->available();
            // Read Available bytes from UART aand write it to RingBuffer
            if (current_msg_size >= 1) {
                for (uint32_t j = 0; j < current_msg_size; j++) {
                    recBuf[j] = ADB_Port->read();
                }
                test_msgProc();  
            }
        }   
}
*/

/*
void ADB_Proto::test_msgProc(void) {
    uint8_t procByte;
    uint32_t i;
    
    if (!init_uart_2) {
        if (ADB_protocol == AP_SerialManager::SerialProtocol_ADB_Proto) {
            init_uart_2 = true;
            //hal.uartA->begin(115200, AP_SERIALMANAGER_ADB_Proto_BUFSIZE_RX, AP_SERIALMANAGER_ADB_Proto_BUFSIZE_TX);                    
        }
    }
    
    for (i = 0; i < current_msg_size; i++) {
        procByte = recBuf[i];

        if (startRec && (procByte == 0x8e)) {
            startRec = false;
        }
        
        if (startRec) {
            recMsgBuf[recIndex++] = procByte;
            
            if (recIndex == ADB_REC_MSG_LEN) {
                recIndex = 0;
                startRec = false;
                //Unpack received msg/value
                prepareMsg();
            }
        }
        if (procByte == 0x8f) {
            startRec = true;
        }
    }
}
*/

void ADB_Proto::tick(void) {
    
    //Initialize UART for ADB Protocol, it have to be initialised from thread which it is used from
    if (!init_uart) {
        if (ADB_protocol == AP_SerialManager::SerialProtocol_ADB_Proto){
            ADB_Port->begin(AP_SERIALMANAGER_ADB_Proto_BAUD, AP_SERIALMANAGER_ADB_Proto_BUFSIZE_RX, AP_SERIALMANAGER_ADB_Proto_BUFSIZE_TX); 
            init_uart = true;
        }
    }

    uint16_t checksum;
    uint16_t ind;

    if (init_uart) {
        // prepare msg and writing to serial
        if (status_counter == 2) {
            ADB_Port->flush_tx_buffer();
        }

        if (status_counter == 3) {
            if (!ADB_Port->tx_pending()) {
                if (esc_discovery && ((AP_HAL::millis() - last_discovery_time_ms) > 10000)) {
                    esc_discovery = false;
                    discover_time_ms = 1000;
                }
                if (!esc_discovery) {
                    discoverEsc(discover_time_ms);
                    status_counter = -1;
                }
                if (esc_discovery && (ADB_DEVICE_COUNT != 0)) {
                    current_msg_size = 0;
                    checksum = 0;
                    ind = 0;
                    msg[ind++] = ADB_LIGHT_HEADER_BYTE;

                    //send request only nth time of 1kHz thread run
                    if (count_to_n == (SEND_EVERY_NTH_TIME)) {
                        count_to_n = 0;
                        checksum += msg[ind++] = active_device_addr[GLOBAL_ADDR_COUNT];
                    }
                    else {
                        checksum += msg[ind++] = 0x7f; //do nothing
                    }
                    count_to_n++;

                    checksum += msg[ind++] = message_ids[GLOBAL_ID_COUNT];

                    // Set desired PWM value equal to RCOUT
                    for (int i = 0; i < ADB_DEVICE_COUNT; i++) {
                        desiredValue[active_device_addr[i]] = hal.rcout->read(active_device_addr[i]);
                    }
                    
                    /*
                    switch (active_device_addr[GLOBAL_ADDR_COUNT]) {
                        case 0x00: 
                            desiredValue[active_device_addr[GLOBAL_ADDR_COUNT]] = hal.rcout->read(CH_1);
                            break;
                        case 0x01:
                            desiredValue[active_device_addr[GLOBAL_ADDR_COUNT]] = hal.rcout->read(CH_2);
                            break;
                        case 0x02:
                            desiredValue[active_device_addr[GLOBAL_ADDR_COUNT]] = hal.rcout->read(CH_3); 
                            break;
                        case 0x03:
                            desiredValue[active_device_addr[GLOBAL_ADDR_COUNT]] = hal.rcout->read(CH_4); 
                            break;
                        case 0x04: 
                            desiredValue[active_device_addr[GLOBAL_ADDR_COUNT]] = hal.rcout->read(CH_5);
                            break;
                        case 0x05:
                            desiredValue[active_device_addr[GLOBAL_ADDR_COUNT]] = hal.rcout->read(CH_6);
                            break;
                        case 0x06:
                            desiredValue[active_device_addr[GLOBAL_ADDR_COUNT]] = hal.rcout->read(CH_7);
                            break;
                        case 0x07:
                            desiredValue[active_device_addr[GLOBAL_ADDR_COUNT]] = hal.rcout->read(CH_8);
                            break;
                        default:
                            return;
                    }
                    */

                    ind = parseToMsg(&checksum, ind);
                    msg[ind++] = (~checksum) & 0x7f;
                    msg[ind++] = ADB_LIGHT_END_BYTE;

                    ADB_Port->write(msg,ind);

                    // start the sending cycle again
                    status_counter = -1;
                    GLOBAL_ADDR_COUNT++;

                    if (GLOBAL_ADDR_COUNT == (ADB_DEVICE_COUNT)) {
                        GLOBAL_ADDR_COUNT = 0;
                        GLOBAL_ID_COUNT++;
                        if ((GLOBAL_ID_COUNT % ADB_MESSAGE_ID_COUNT) == 0) GLOBAL_ID_COUNT = 0;
                    }

                    // testing communication error
                    /*
                    if (!STOP_SEND && (AP_HAL::millis() > 15000)) {
                        errorCount = 5;
                        STOP_SEND = true;
                        lastErrorTime = AP_HAL::millis();
                    }
                    */
                    
                    calc_esc_responses();
                    errorCheck();   
                }
                

            } else ADB_Port->flush_tx_buffer();
        }
        if ((status_counter == -1) || (status_counter == 0) || (status_counter == 1) || (status_counter == 2)) {
            status_counter++;
        }

        // reading from serial
        current_msg_size = ADB_Port->available();
        if (current_msg_size > 0) {
            for (uint32_t j = 0; j < current_msg_size; j++) {
                recBuf[j] = ADB_Port->read();
            }
            msgProc();
        }
    }
    /*
    if ((AP_HAL::micros() - sync_timer_counter_past) >= 5000) {
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
    */
}

// return true if communication error is detected
void ADB_Proto::errorCheck() {

    if ((AP_HAL::millis() - lastErrorTime) > 60000) {
        errorCount = 0;
    }

    if (errorCount >= 2) {
        current_esc_error.error = communication_error;
        error_occured = true;
    }

    if (current_esc_error.error = communication_error) {
        if (error_occured && ((AP_HAL::millis() - lastErrorTime) > 5000)) {
            error_occured = false;
            errorCount = 0;
        }
    }

}

// discover connected ESCs for first xy ms
void ADB_Proto::discoverEsc(uint32_t discovery_time) {
    static uint32_t message_last_sent_us;
    uint16_t checksum;
    uint16_t ind;

    //SET number of online ESCs and their addresses 
    if (set_active_devices) {
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
        esc_discovery_started = false;
        last_discovery_time_ms = AP_HAL::millis();
    }

    // SCAN/Discover connected ESCs for the first *discover_time_ms* ms after boot
    if (!esc_discovery) {
        
        if (!esc_discovery_started) {
            esc_discovery_start = AP_HAL::millis();
            esc_discovery_started = true;
        }

        checksum = 0;
        ind = 0;

        msg[ind++] = ADB_LIGHT_HEADER_BYTE;
        checksum += msg[ind++] = device_address[GLOBAL_ADDR_COUNT];
        checksum += msg[ind++] = 1; //we do not need to change requests here

        // Set desired PWM value equal to RCOUT
        for (int i = 0; i < ADB_MAX_DEVICE_COUNT; i++) {
            // hal.rcout->read(0) means the same as hal.rcout->read(CH_1) etc
            desiredValue[device_address[0]] = hal.rcout->read(i);
        }
        
        /*
        // Set desired PWM value equal to RCOUT
        switch (device_address[GLOBAL_ADDR_COUNT]) {
            case 0x00: 
                desiredValue[device_address[GLOBAL_ADDR_COUNT]] = hal.rcout->read(CH_1);
                break;
            case 0x01:
                desiredValue[device_address[GLOBAL_ADDR_COUNT]] = hal.rcout->read(CH_2);
                break;
            case 0x02:
                desiredValue[device_address[GLOBAL_ADDR_COUNT]] = hal.rcout->read(CH_3); 
                break;
            case 0x03:
                desiredValue[device_address[GLOBAL_ADDR_COUNT]] = hal.rcout->read(CH_4); 
                break;
            case 0x04: 
                desiredValue[device_address[GLOBAL_ADDR_COUNT]] = hal.rcout->read(CH_5);
                break;
            case 0x05:
                desiredValue[device_address[GLOBAL_ADDR_COUNT]] = hal.rcout->read(CH_6);
                break;
            case 0x06:
                desiredValue[device_address[GLOBAL_ADDR_COUNT]] = hal.rcout->read(CH_7);
                break;
            case 0x07:
                desiredValue[device_address[GLOBAL_ADDR_COUNT]] = hal.rcout->read(CH_8);
                break;
            default:
                return;
        }
        */

        ind = parseToMsg(&checksum, ind);
        msg[ind++] = (~checksum) & 0x7f;
        msg[ind++] = ADB_LIGHT_END_BYTE;

        ADB_Port->write(msg, ind);
        GLOBAL_ADDR_COUNT++;

        if (GLOBAL_ADDR_COUNT == ADB_MAX_DEVICE_COUNT) {
            GLOBAL_ADDR_COUNT = 0;
        }

        // finish discovery
        if ((AP_HAL::millis() - esc_discovery_start) > discover_time_ms) {
            set_active_devices = true;
        }
    }
}

int ADB_Proto::parseToMsg(uint16_t *checksum, uint16_t actualIndex) {
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

void ADB_Proto::msgProc() {
    uint8_t procByte;

    for (uint32_t i = 0; i < current_msg_size; i++) {
        procByte = recBuf[i];

        if (startRec && (procByte == 0x8e)) {
            startRec = false;
        }
        
        if (startRec){
            recMsgBuf[recIndex++] = procByte;
            
            if (recIndex == ADB_REC_MSG_LEN) {
                recIndex = 0;
                startRec = false;

                //Unpack received msg/value
                prepareMsg();
            }
        }
        if (procByte == 0x8f) {
            startRec = true;
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

// Parse incoming message, prepare for Mavlink
void ADB_Proto::prepareMsg() {
	uint8_t chcksum = 0;
	uint32_t value = 0;
	int8_t deviceAddress = -1;
	int8_t messageId = -1;
    deviceAddress = (recMsgBuf[0] >> 4) & 0x07;
	messageId = recMsgBuf[0] & 0x0f;
    chcksum ^= value = (recMsgBuf[3] >> 4) & 0x07;

	for(uint8_t i = 2 ; i > 0; i--) {
		value <<= 7;
		value |= recMsgBuf[i] & 0x7f;
		chcksum ^= (recMsgBuf[i] >> 4) & 0x07;
		chcksum ^= recMsgBuf[i] & 0x0f;
	}

	chcksum = (~chcksum) & 0x0f;

    //Save which ESCs are responding while doing discovery
    if((chcksum == (recMsgBuf[3] & 0x0f)) && !esc_discovery) { 

        esc_responses_ms[deviceAddress] = AP_HAL::millis();
        
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
    if((chcksum == (recMsgBuf[3] & 0x0f)) && (esc_discovery)) {
        processing_packet = true;
        tmp_log.deviceAddr = deviceAddress;
        tmp_log.good_msgs = good_packet_count;
        tmp_log.bad_msgs = bad_packet_count;

        uint32_t tmp_mask = 0;
        bool isGreaterThanZero;
        float val0, val1, val2, val3;
        int i;

        // save response time of esc which sent the message
        esc_responses_ms[deviceAddress] = AP_HAL::millis();
        
        // parsing received data for latter mavlink communication
        switch (messageId) {
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
   
    // counting MSG errors
    if (esc_discovery) {
        if (!processing_packet) {
            bad_packet_count++;
            errorCount++;
            lastErrorTime = AP_HAL::millis();
            current_esc_error.esc_id = deviceAddress;
        }
        else if (processing_packet) good_packet_count++;
    }
}

// searching if any of discovered ESCs disconnected
void ADB_Proto::calc_esc_responses() {
    int j = 0;

    for (int i = 0; i < ADB_DEVICE_COUNT; i++) {
        if ((AP_HAL::millis() - esc_responses_ms[active_device_addr[i]]) > CONNECTION_TIMEOUT_MS) {
            not_responding_esc[j++] = active_device_addr[i];
            current_esc_error.error = esc_disconnected;
            current_esc_error.esc_id = i;
            error_occured = true;
        }

        else {
            // do something when esc connected or reconnected
            error_occured  = false;
        }
    }
    if (error_occured) not_responding_esc[j] = -1; 
}

// return warning for MAVLink
char *ADB_Proto::get_warning_string() {
    if (error_occured) {
        switch (current_esc_error.error) {
            case communication_error:
                strcpy(warning_msg, "ESC comm error");
                break;
            case esc_disconnected:
                strcpy(warning_msg, "ESCs disconnected, IDs: ");
                break;
        }

        if (current_esc_error.error == esc_disconnected) {
            for (int i = 0; i < ADB_DEVICE_COUNT; i++) {
                if (not_responding_esc[i] == -1) break;

                switch (not_responding_esc[i]) {
                    case 0:
                        strncat(warning_msg, "0 ",2);
                        break;
                    case 1:
                        strncat(warning_msg, "1 ",2);
                        break;
                    case 2:
                        strncat(warning_msg, "2 ",2);
                        break;
                    case 3:
                        strncat(warning_msg, "3 ",2);
                        break;
                    case 4:
                        strncat(warning_msg, "4 ",2);
                        break;
                    case 5:
                        strncat(warning_msg, "5 ",2);
                        break;
                    case 6:
                        strncat(warning_msg, "6 ",2);
                        break;
                    case 7:
                        strncat(warning_msg, "7 ",2);
                        break;
                }
            }
        }
        strncat(warning_msg, "\0",1);
        return warning_msg;
    }
    else return nullptr;
}