#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include <SPI.h>
#include <ArduinoBLE.h>
#include "Adafruit_DRV2605.h"
#include "vtf_driver.h"
#include <stdint.h>

BLEService vtfService("d2411652-234a-11ec-9621-0242ac130002"); // BLE LED Service

const int WRITE_BUFFER_SIZE = 256;
bool WRITE_BUFFER_FIZED_LENGTH = false;

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLECharacteristic switchCharacteristic("b8aff320-234a-11ec-9621-0242ac130002",BLEWriteWithoutResponse | BLEWrite, WRITE_BUFFER_SIZE, WRITE_BUFFER_FIZED_LENGTH);


/* MESSSAGE TYPES */
#define PreDefinedType 		0x50
#define RealTimeType 		0x52
#define GoType 				0x47
#define PauseType 			0x53
#define LoopType 			0x4C

#define StartBlock 			0x41
#define EndBlock 			0x45
#define StartCommand 		0x73
#define EndCommand 			0x78

/* VIBRATION TYPES */
#define PRE_DEFINED 0
#define REAL_TIME 1

/* MOTOR DRIVER INSTANCES */
Adafruit_DRV2605 drv_0;
Adafruit_DRV2605 drv_1;
Adafruit_DRV2605 drv_2;
Adafruit_DRV2605 drv_3;
Adafruit_DRV2605 drv_4;
Adafruit_DRV2605 drv_5;
Adafruit_DRV2605 drv_6;
Adafruit_DRV2605 drv_7;
Adafruit_DRV2605 drv_8;
Adafruit_DRV2605 drv_9;
Adafruit_DRV2605 drv_10;
Adafruit_DRV2605 drv_11;
Adafruit_DRV2605 drv_12;
Adafruit_DRV2605* driver_list[13] = {&drv_0, &drv_1, &drv_2, &drv_3, &drv_4, &drv_5, &drv_6, &drv_7, &drv_8, &drv_9, &drv_11, &drv_12};

/* VTF DRIVER INSTANCE */
vtf_driver driver;

/* GLOBAL MOTOR VALUES */
int global_motors_set[14] = {40};
int global_motor_pos = 0;
int motor_0_pos = 0;
int motor_1_pos = 0;
int motor_2_pos = 0;
int motor_3_pos = 0;
int motor_4_pos = 0;
int motor_5_pos = 0;
int motor_6_pos = 0;
int motor_7_pos = 0;
int motor_8_pos = 0;
int motor_9_pos = 0;
int motor_10_pos = 0;
int motor_11_pos = 0;
int motor_12_pos = 0;

/* GLOBAL VIBRATION MODE*/
int current_vibration_mode = PRE_DEFINED;
int current_real_time_motor = 40;



void rgb_setup(void){
  // set LED's pin to output mode
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);         // when the central disconnects, turn off the LED
  digitalWrite(LEDR, HIGH);               // will turn the LED off
  digitalWrite(LEDG, HIGH);               // will turn the LED off
  digitalWrite(LEDB, HIGH);                // will turn the LED off
}

void set_multi(int pre_programmed_num, int *motors){
	    for(int i = 0; i < 14; i++){ 
        if(motors[i] == -1){
            break;
			
        } else if(motors[i] == 0){
			drv_0.setWaveform(motor_0_pos, (pre_programmed_num));
			motor_0_pos++;

        } else if (motors[i] == 1){
            drv_1.setWaveform(motor_1_pos, (pre_programmed_num));
			motor_1_pos++;
            
        } else if (motors[i] == 2){
            drv_2.setWaveform(motor_3_pos, (pre_programmed_num));
			motor_2_pos++;
        
        } else if (motors[i] == 3){
            drv_3.setWaveform(motor_3_pos, (pre_programmed_num));
			motor_3_pos++;
        
        } else if (motors[i] == 4){
			drv_4.setWaveform(motor_4_pos, (pre_programmed_num));
			motor_4_pos++;
        
        } else if (motors[i] == 5){
			drv_5.setWaveform(motor_5_pos, (pre_programmed_num));
			motor_5_pos++;
        
        } else if (motors[i] == 6){
			drv_6.setWaveform(motor_6_pos, (pre_programmed_num));
			motor_6_pos++;
        
        } else if (motors[i] == 7){
			drv_7.setWaveform(motor_7_pos, (pre_programmed_num));
			motor_7_pos++;
        
        } else if (motors[i] == 8){
			drv_8.setWaveform(motor_8_pos, (pre_programmed_num));
			motor_8_pos++;
        
        } else if (motors[i] == 9){
			drv_9.setWaveform(motor_9_pos, (pre_programmed_num));
			motor_9_pos++;
        
        } else if (motors[i] == 10){
			drv_10.setWaveform(motor_10_pos, (pre_programmed_num));
			motor_10_pos++;
        
        } else if (motors[i] == 11){
			drv_11.setWaveform(motor_11_pos, (pre_programmed_num));
			motor_11_pos++;
        
        } else if (motors[i] == 12){
			drv_12.setWaveform(motor_12_pos, (pre_programmed_num));
			motor_12_pos++;

        }
    }
}

void switchVibrationType(int vibration_type){
	//int all_motors[14] = {0,1,2,3,4,5,6,7,8,9,10,11,12,-1};
	//driver.tcas_set_multi(all_motors);
	if(vibration_type == PRE_DEFINED){
		driver.change_motor(MOTOR3);
		drv_3.setMode(DRV2605_MODE_INTTRIG); 
	} else {
		//Serial.println("TEST");
		driver.change_motor(MOTOR3);
		drv_3.setMode(DRV2605_MODE_REALTIME); 
	}

}

void processGo(int pause){
		//Serial.println("\n Firing Motors");
		
		global_motors_set[global_motor_pos] = -1;
		// Serial.println("\n Motors Set");
		// for (int i = 0; i < 13; i++)
		// {
		// 	Serial.print(global_motors_set[i]);
		// }

		Serial.println();
		set_multi(0,global_motors_set);
		driver.go_multi(global_motors_set, drv_0);
		while((drv_0.readRegister8(DRV2605_REG_GO)  & 0x01)!=0){}
		
		//Clear Set motors
		for(int i = 0; i < 14; i++){
			global_motors_set[i] = 40;
		}
		
		global_motor_pos = 0;
		motor_0_pos = 0;
		motor_1_pos = 0;
		motor_2_pos = 0;
		motor_3_pos = 0;
		motor_4_pos = 0;
		motor_5_pos = 0;
		motor_6_pos = 0;
		motor_7_pos = 0;
		motor_8_pos = 0;
		motor_9_pos = 0;
		motor_10_pos = 0;
		motor_11_pos = 0;
		motor_12_pos = 0;
		if(!pause){
			delay(250);
		}
}

void processPause(int* data){
	processGo(1);
	int pause_time = 0;
	for(int i = 0; i < 4; i ++){
		pause_time += data[i+1] << (i * 8);
	}
	
	delay(pause_time);
}

void processRealTime(int* data){
	Serial.print("\n The Motor Number is:");
	Serial.println(data[1]);
	if(data[1] != current_real_time_motor){
		driver.change_motor(data[1]);
		current_real_time_motor = data[1];
		delay(250);
	}

	Serial.print("\n The Intensity is:");
	Serial.println(data[2]);
	drv_3.setRealtimeValue(data[2]);
}


void processPreDefined(int* data){
	int motor_pos = 0;
	int motors[14], first[8] = {0}, second[8]= {0};
	int firstint = data[3];
	int secondint = data[2];

	for(int i=0;firstint>0;i++){    
		first[i]=firstint%2;    
		firstint=firstint/2;
		Serial.print(first[i]);    
	} 

	for(int i=0;secondint>0;i++){    
		second[i]=secondint%2;    
		secondint=secondint/2;   
		Serial.print(second[i]);   
	} 

	for(int i  = 0; i<8; i++){
		if(first[i]){
			motors[motor_pos] = i;
			motor_pos++;
		}
	}

	for(int i  = 0; i<8; i++){
		if(second[i]){
			motors[motor_pos] = i+8;
			motor_pos++;
		}
	}

	motors[motor_pos] = -1;

	int already_set = 0;
	for(int i  = 0; i<=motor_pos; i++){
		
		for(int j  = 0; j<14; j++){
			if(global_motors_set[j] == motors[i]){
				already_set = 1;
			}
		}

		if((!already_set) && (motors[i] != -1)){
			global_motors_set[global_motor_pos] = motors[i];
			global_motor_pos ++;
		}   

		already_set = 0;
	}

	driver.tcas_set_multi(motors);


	set_multi(data[1]+1, motors);
}



void processSerialBlock(int* data, int size_of_data){
	//2D Block to pack data into
	int data_block [size_of_data][10];

	//Data counters
	int command_start_cnt = 0;
	int command_end_cnt = 0;
	int command_pos = 0;
	int command_num = 0;
	int in_command = 0;

	//Loop counters
	int num_loops = 0;
    int start_loop = 0;
    int end_loop = 0;

	
	//Pack data blocks
	for(int i = 0; i < size_of_data; i ++){

		if(in_command){
			
			if(data[i] == EndCommand){
				command_end_cnt ++;
				if(command_end_cnt == 2){
					command_start_cnt = command_end_cnt = 0;
					command_pos = 0;
					command_num ++;
					in_command = 0;
				}
			} else {
				data_block[command_num][command_pos] = data[i];
				command_pos ++;
			}
		}


		if(data[i] == StartCommand){
			command_start_cnt ++;
			if(command_start_cnt == 2){
				in_command = 1;
			} 
		} else {
			command_start_cnt = 0;
		}
		
	}


	//Execute data blocks
	for(int i = 0; i < command_num; i++){
		Serial.print("Poistion in loop : ");
		Serial.println(i);
		if(data_block[i][0] == PreDefinedType){
			if(current_vibration_mode == PRE_DEFINED){
				Serial.println("Pre Def Type");
				processPreDefined(data_block[i]);
			} else {
				switchVibrationType(PRE_DEFINED);
				current_vibration_mode = PRE_DEFINED;
				processPreDefined(data_block[i]);
			}
		
		} else if (data_block[i][0]  == RealTimeType){
			Serial.println("Real Time Type");
			if(current_vibration_mode == REAL_TIME){
				processRealTime(data_block[i]);
			} else {
				switchVibrationType(REAL_TIME);
				current_vibration_mode = REAL_TIME;
				processRealTime(data_block[i]);
			}
		
		} else if (data_block[i][0]  == GoType){
			processGo(0);

		} else if (data_block[i][0]  == PauseType){
			processPause(data_block[i]);

		} else if (data_block[i][0]  == LoopType){
			
			if(num_loops == 0 && end_loop != i){
				num_loops = data_block[i][1] - 1;
				Serial.print("NUMBER OF LOOPS ");
				Serial.println(num_loops);
				end_loop = i;
				i = start_loop-1;
				processGo(1);

			} else if (num_loops == 0 && end_loop == i){
				Serial.println("End of Loop ");
				start_loop = i;

			} else if (num_loops > 0 && end_loop == i){
				Serial.print("Number of loops left ");
				num_loops --;
				Serial.println(num_loops);
				i = start_loop-1;
				processGo(1);
			} 
		}
	}
}  


/*
 * BLE PROCESING
 */

void processBLEBlock(byte* data, int data_size){
	uint8_t current_data = 0;
	int* ble_data_to_process;
	ble_data_to_process = (int*)malloc(sizeof(int) * data_size);

	int start_cnt = 0;
	int endcnt = 0;
	int getting_data = 0;
	int data_pos = 0;


	//Processing
	Serial.print("DATA RECIVED");
	for(int i =0 ; i<data_size; i++){

		current_data = (int)data[i];
		//Serial.println(data[i]);
		//Serial.println(current_data);

		if(getting_data){
			if(current_data == EndBlock){
				endcnt ++;
				if(endcnt == 2){
					processSerialBlock(ble_data_to_process, data_pos);
					getting_data = 0;
					data_pos = 0;
					endcnt = 0;
					break;

				} 
			} else {
				ble_data_to_process[data_pos] = current_data;
				//Serial.print(current_data);
				data_pos ++;
			}
		
		}

		if(current_data == StartBlock){

			start_cnt ++;		
			if(start_cnt == 2){
				getting_data = 1;
			}

		} else {
			start_cnt = 0;
		}

	}

}

void switchCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
	int ble_data_size = 256;
	//char* ble_data;
	byte ble_data[256]; 
	int num_read;
	//switchCharacteristic.read();
	num_read = switchCharacteristic.readValue(ble_data,ble_data_size);
	Serial.println("RECIVED DATA");
	Serial.print("NUMBER OF BYTES: ");
	Serial.println(num_read);
	for(int i = 0; i < 40; i++){
		Serial.println((int)ble_data[i]);
	}
	processBLEBlock(ble_data, num_read);
}

void bluetooth_setup(void){

  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("Nano 33 BLE");
  BLE.setAdvertisedService(vtfService);

  // add the characteristic to the service
  vtfService.addCharacteristic(switchCharacteristic);

  // add service
  BLE.addService(vtfService);

  //add callback
  switchCharacteristic.setEventHandler(BLEWritten, switchCharacteristicWritten);

  // start advertising
  BLE.advertise();
  

  Serial.println(switchCharacteristic);

  Serial.println("BLE VTF Peripheral");
}

void setup() {
  // put your setup code here, to run once:
  	Serial.begin(9600);
	while (!Serial);
	Serial.println("Haptic Motor Driver test");
	delay(1000);

	Wire.begin();
	Wire.setClock(100000);
	Wire.setTimeout(10000);

	rgb_setup();
	bluetooth_setup();

	//driver.LRA_setup(MOTOR1,drv_1);
	driver.change_motor(MOTOR0);
    drv_0.begin();
    drv_0.useLRA();
	drv_0.selectLibrary(6);
    drv_0.setMode(DRV2605_MODE_INTTRIG); 
    drv_0.setWaveform(0, 0);
	drv_0.go();

	driver.change_motor(MOTOR2);
    drv_2.begin();
	drv_2.selectLibrary(1);
    drv_2.setMode(DRV2605_MODE_INTTRIG); 
    drv_2.setWaveform(0, 0);
	drv_2.go();

	Serial.println("Setup Done");
  
}

void loop() {
	// put your main code here, to run repeatedly:
	//Serial.println("Setup Done");
	int incomingByte = 0;
	int buffer = 20;
	int start_cnt = 0;
	int endcnt = 0;
	int getting_data = 0;
	int data_pos = 0;
	int* data_to_process;
	char* ble_data;
	data_to_process = (int*)malloc(sizeof(int) * buffer);
	ble_data = (char*)malloc(sizeof(char) * 512*8);
	int ble_data_size = 512;

	driver.change_motor(MOTOR0);
	drv_0.setWaveform(0, 1);
	drv_0.setWaveform(1, 0);
	drv_0.go();
	delay(250);

	
	
	while(1){

		//BLE PROCESSING//
		 // listen for BLE peripherals to connect:
		BLEDevice central = BLE.central();

		// if a central is connected to peripheral:
		if (central) {
			Serial.print("Connected to central: ");
			// print the central's MAC address:
			Serial.println(central.address());
			digitalWrite(LED_BUILTIN, HIGH);            // turn on the LED to indicate the connection

			// while the central is still connected to peripheral:
			while (central.connected()) {
			// if the remote device wrote to the characteristic,
			// use the value to control the LED:
			
			//if (switchCharacteristic.written()) {
				
			// 	switchCharacteristic.readValue(ble_data,ble_data_size);
			// 	Serial.println("BLE DATA REVIVED");
			// 	processBLEBlock(ble_data);


				// switch (switchCharacteristic.value()) {   // any value other than 0
				// 	case 01:
				// 		Serial.println("Red LED on");
				// 		digitalWrite(LEDR, LOW);            // will turn the LED on
				// 		digitalWrite(LEDG, HIGH);         // will turn the LED off
				// 		digitalWrite(LEDB, HIGH);         // will turn the LED off
				// 		break;
				// 	case 02:
				// 		Serial.println("Green LED on");
				// 		digitalWrite(LEDR, HIGH);         // will turn the LED off
				// 		digitalWrite(LEDG, LOW);        // will turn the LED on
				// 		digitalWrite(LEDB, HIGH);        // will turn the LED off
				// 		break;
				// 	case 03:
				// 		Serial.println("Blue LED on");
				// 		digitalWrite(LEDR, HIGH);         // will turn the LED off
				// 		digitalWrite(LEDG, HIGH);       // will turn the LED off
				// 		digitalWrite(LEDB, LOW);         // will turn the LED on
				// 		break;
				// 	default:
				// 		Serial.println(F("LEDs off"));
				// 		digitalWrite(LEDR, HIGH);          // will turn the LED off
				// 		digitalWrite(LEDG, HIGH);        // will turn the LED off
				// 		digitalWrite(LEDB, HIGH);         // will turn the LED off
				// 		break;
				// 	}
				//}
			}
		} else {
			digitalWrite(LED_BUILTIN, LOW); 
		}

		


		//SERIAL PROCESSING//
		if(Serial.available() > 0){
			incomingByte = Serial.read();
		}

		if(getting_data){
			if(incomingByte == EndBlock){
				endcnt ++;
				if(endcnt == 2){
					processSerialBlock(data_to_process, data_pos);
					getting_data = 0;
					data_pos = 0;
					endcnt = 0;

				} 
			} else {
				data_to_process[data_pos] = incomingByte;
				//Serial.print(incomingByte);
				data_pos ++;
				if(data_pos == buffer){
					buffer = buffer*2;
					data_to_process = (int*)realloc(data_to_process,buffer);
				}
			}
		
		}

		if(incomingByte == StartBlock){

			start_cnt ++;		
			if(start_cnt == 2){
				getting_data = 1;
			}

		} else {
			start_cnt = 0;
		}

		incomingByte = 0;
		delay(1);
			
	}
	
}
