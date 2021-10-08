#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include <SPI.h>
#include <ArduinoBLE.h>
#include "Adafruit_DRV2605.h"
#include "vtf_driver.h"
#include <stdint.h>
#include "Arduino_LSM9DS1.h"
#include <MadgwickAHRS.h>

// BLE Service that is advertised
BLEService vtfService("d2411652-234a-11ec-9621-0242ac130002"); // BLE LED Service


const int WRITE_BUFFER_SIZE = 256;			// Max byte size of the incoming message (can be increased to 512)
bool WRITE_BUFFER_FIZED_LENGTH = false;		// Arduino BLE Docs talk about this unsure of function 

// BLE VTF Switch Characteristic - custom 128-bit UUID, read and writable by central
BLECharacteristic switchCharacteristic("b8aff320-234a-11ec-9621-0242ac130002",BLEWriteWithoutResponse | BLEWrite, WRITE_BUFFER_SIZE, WRITE_BUFFER_FIZED_LENGTH);


/* MESSSAGE TYPES */
/* Define the type of message being sent over serial or BLE*/
#define PreDefinedType 		0x50
#define RealTimeType 		0x52
#define DemoType			0x44
#define GoType 				0x47
#define PauseType 			0x53
#define LoopType 			0x4C

//Encode the start and end of blocks and commands
#define StartBlock 			0x41
#define EndBlock 			0x45
#define StartCommand 		0x73
#define EndCommand 			0x78

/* VIBRATION TYPES */
#define PRE_DEFINED 		0
#define REAL_TIME 			1
#define DEMO				2

/*DEMO NUMBERS*/
#define Demo1				0
#define Demo2				1
int current_demo_number = 0;

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

/*Accelerometer Info*/
Madgwick filter;
const float sensorRate = 104.00;


/**************************************************************************/
/*!
  @brief Sets up the onboard RGB LEDs on the nano 33 ble. Can be used for
		debug or conveying information to users
*/
/**************************************************************************/
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

/**************************************************************************/
/*!
  @brief Set the waveform of multiple DRVs. Assumes they have already been 
		 selected on the TCAS with tcas_set_multi
  @param pre_programmed_num Vibration type to program into motors
  @param motors Pointer to array of motors to set. End array with -1
*/
/**************************************************************************/
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

/**************************************************************************/
/*!
  @brief Changes the vibration type of all motors
  @param vibration_type Either REAL_TIME or PRE_DEFINED
*/
/**************************************************************************/
void switchVibrationType(int vibration_type){
	//driver.tcas_set_multi(all_motors);

	//Work around for multiple writing error
	int first[3] = {0,1,-1};
	int second[3] = {2,3,-1};
	int third[3] = {4,5,-1};
	int fourth[3] = {6,7,-1};
	int fifth[3] = {8,9,-1};
	int sixth[3] = {10,11,-1};


	if(vibration_type == REAL_TIME){
		Serial.println("CHANGING TO REALTIME");
		driver.tcas_set_multi(first);
		drv_1.setMode(DRV2605_MODE_REALTIME); 
		delay(300);
		driver.tcas_set_multi(second);
		drv_1.setMode(DRV2605_MODE_REALTIME); 
		delay(300);
		driver.tcas_set_multi(third);
		drv_1.setMode(DRV2605_MODE_REALTIME); 
		delay(300);
		driver.tcas_set_multi(fourth);
		drv_1.setMode(DRV2605_MODE_REALTIME); 
		delay(300);
		driver.tcas_set_multi(fifth);
		drv_1.setMode(DRV2605_MODE_REALTIME); 
		delay(300);
		driver.tcas_set_multi(sixth);
		drv_1.setMode(DRV2605_MODE_REALTIME); 
		delay(300);
		driver.change_motor(MOTOR12);
		drv_1.setMode(DRV2605_MODE_REALTIME); 
		delay(300);
		Serial.println("Done");
	
	} else {
		Serial.println("CHANGING TO REALTIME");
		driver.tcas_set_multi(first);
		drv_1.setMode(DRV2605_MODE_INTTRIG); 
		delay(300);
		driver.tcas_set_multi(second);
		drv_1.setMode(DRV2605_MODE_INTTRIG); 
		delay(300);
		driver.tcas_set_multi(third);
		drv_1.setMode(DRV2605_MODE_INTTRIG); 
		delay(300);
		driver.tcas_set_multi(fourth);
		drv_1.setMode(DRV2605_MODE_INTTRIG); 
		delay(300);
		driver.tcas_set_multi(fifth);
		drv_1.setMode(DRV2605_MODE_INTTRIG); 
		delay(300);
		driver.tcas_set_multi(sixth);
		drv_1.setMode(DRV2605_MODE_INTTRIG); 
		delay(300);
		driver.change_motor(MOTOR12);
		drv_1.setMode(DRV2605_MODE_INTTRIG); 
		delay(300);
		Serial.println("Done");
	}

}

/**************************************************************************/
/*!
  @brief Will send a go command to all motors that currently have a function
		writen to them
  @param pause if go will pause directly after then 1 else 0
*/
/**************************************************************************/
void processGo(int pause){
		Serial.println("\n Firing Motors");
		
		global_motors_set[global_motor_pos] = -1;
		Serial.println("\n Motors Set");
		for (int i = 0; i < 13; i++)
		{
			Serial.print(global_motors_set[i]);
		}

		Serial.println();
		set_multi(0,global_motors_set);
		driver.go_multi(global_motors_set, drv_0);
		//while((drv_5.readRegister8(DRV2605_REG_GO)  & 0x01)!=0){}
		
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

/**************************************************************************/
/*!
  @brief Process information from a go command
  @param data Contents of the pause command
*/
/**************************************************************************/
void processPause(int* data){
	processGo(1);
	int pause_time = 0;

	for(int i = 0; i < 4; i ++){
		pause_time += data[i+1] << (i * 8);
	}
	
	delay(pause_time);
}


void run_demo(int demo_num){

	int vib_intensity = 0;

	int left_motors[3] = {3,4,-1};
	int right_motors[3] = {8,9,-1};

	int roll_deadzone = 10;
	int current_roll_dirrection = 0;
	float max_roll_right = 90;
	float max_roll_left = 90;
	
	int front_motors[3] = {3,8,-1};
	int back_motors[3] = {4,9,-1};	

	int pitch_deadzone = 10;
	int current_pitch_dirrection = 0;
	float max_pitch_back = 90;
	float max_pitch_front= 90;

	float xAcc, yAcc, zAcc;
	float xGyro, yGyro, zGyro;
	float roll, pitch, heading;


	if(demo_num == Demo1){
		//Gyroscope Processing//
		if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
			IMU.readAcceleration(xAcc, yAcc, zAcc);
        	IMU.readGyroscope(xGyro, yGyro, zGyro);
			filter.updateIMU(xGyro, yGyro, zGyro, xAcc, yAcc, zAcc);
			roll = filter.getRoll();
			//pitch = filter.getPitch();
			//heading = filter.getYaw();
			//Serial.print("Roll: ");
			Serial.println(roll);
			// Serial.print(" Pitch: ");
			// Serial.print(pitch);
			// Serial.print(" Heding: ");
			// Serial.println(heading);

			if(roll>0){
				if(roll>roll_deadzone){
					if(current_roll_dirrection != 1){
						// driver.tcas_set_multi(left_motors);
					}
					vib_intensity = round((roll/max_roll_left)*255);
					drv_1.setRealtimeValue(vib_intensity);
					Serial.print("Left motor Intensity: ");
					Serial.println(vib_intensity);
				}
			} else if(roll<0){
				if(roll<(-1*roll_deadzone)){
					if(current_roll_dirrection != 0){
						// driver.tcas_set_multi(right_motors);
					}
					vib_intensity = abs(round((roll/max_roll_right)*255));
					drv_1.setRealtimeValue(vib_intensity);
						Serial.print("Right motor Intensity: ");
					Serial.println(vib_intensity);
				}
			}
			
		}
	} else if(demo_num == Demo2){
		if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
			IMU.readAcceleration(xAcc, yAcc, zAcc);
        	IMU.readGyroscope(xGyro, yGyro, zGyro);
			filter.updateIMU(xGyro, yGyro, zGyro, xAcc, yAcc, zAcc);
			pitch = filter.getPitch();

			if(pitch>0){
				if(pitch>pitch_deadzone){
					if(current_pitch_dirrection != 1){
						// driver.tcas_set_multi(back_motors);
					}
					vib_intensity = round((pitch/max_pitch_back)*255);
					drv_1.setRealtimeValue(vib_intensity);
					Serial.print("Back motor Intensity: ");
					Serial.println(vib_intensity);
				}
			} else if(pitch<0){
				if(pitch<(-1*pitch_deadzone)){
					if(current_pitch_dirrection != 0){
						// driver.tcas_set_multi(front_motors);
					}
					vib_intensity = abs(round((pitch/max_pitch_front)*255));
					drv_1.setRealtimeValue(vib_intensity);
						Serial.print("Front motor Intensity: ");
					Serial.println(vib_intensity);
				} if(pitch >= max_pitch_front){
					// driver.tcas_set_multi(front_motors);
					// set_multi()
					//processGo
				}
			}
		}
	}
}


/**************************************************************************/
/*!
  @brief Begin a given demo
*/
/**************************************************************************/
void processDemo(int* data){
	current_demo_number=1;
	int first[3] = {3,4,-1};
	int second[3] = {8,9,-1};
	if(data[1] == Demo1){
		Serial.println("BEGIN DEMO 1");
		current_demo_number = Demo1;
		driver.tcas_set_multi(first);
		drv_1.setMode(DRV2605_MODE_REALTIME); 
		delay(300);
		driver.tcas_set_multi(second);
		drv_1.setMode(DRV2605_MODE_REALTIME); 
		delay(300);
	} else if(data[1] == Demo2){
		Serial.println("BEGIN DEMO 2");
		current_demo_number = Demo2;
		driver.tcas_set_multi(first);
		drv_1.setMode(DRV2605_MODE_REALTIME); 
		delay(300);
		driver.tcas_set_multi(second);
		drv_1.setMode(DRV2605_MODE_REALTIME); 
		delay(300);
	}
}

/**************************************************************************/
/*!
  @brief Process information from a real time command
  @param data Contents of the real time command
*/
/**************************************************************************/
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
	driver_list[current_real_time_motor]->setRealtimeValue(data[2]);
}


/**************************************************************************/
/*!
  @brief Process information from a pre defined command
  @param data Contents of the pre defined command
*/
/**************************************************************************/
void processPreDefined(int* data){
	int motor_pos = 0;
	int motors[14], first[8] = {0}, second[8]= {0};
	int firstint = data[3];
	int secondint = data[2];

	for(int i=0;firstint>0;i++){    
		first[i]=firstint%2;    
		firstint=firstint/2;
	} 

	for(int i=0;secondint>0;i++){    
		second[i]=secondint%2;    
		secondint=secondint/2;   
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


/**************************************************************************/
/*!
  @brief Process information from a block of data. Will break into
		commands and then call functions to execute them
  @param data Contents of the data block
  @param size_of_data size of bytes in data block
  
*/
/**************************************************************************/
void processBlock(int* data, int size_of_data){
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
		}else if(data_block[i][0]  == DemoType){
			current_vibration_mode = DEMO;
			processDemo(data_block[i]);
		
		} else if (data_block[i][0]  == GoType){
			processGo(0);

		} else if (data_block[i][0]  == PauseType){
			processPause(data_block[i]);

		} else if (data_block[i][0]  == LoopType){
			
			if(num_loops == 0 && end_loop != i){
				num_loops = data_block[i][1] - 1;
				end_loop = i;
				i = start_loop-1;
				processGo(1);

			} else if (num_loops == 0 && end_loop == i){
				start_loop = i;

			} else if (num_loops > 0 && end_loop == i){
				num_loops --;
				i = start_loop-1;
				processGo(1);
			} 
		}
	}
}  


/**************************************************************************/
/*!
  @brief Process information from a BLE block of data. Will send to  
		processBlock for execution
  @param data Contents of the data block
  @param data_size size of bytes in data block
  
*/
/**************************************************************************/
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
					processBlock(ble_data_to_process, data_pos);
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


/**************************************************************************/
/*!
  @brief This is a callback function that will be called apon reciving BLE 
  		data 
  @param central Device connected to BLE 'Board'
  @param characteristic The characteristic data was writen to
  
*/
/**************************************************************************/
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

/**************************************************************************/
/*!
  @brief Setup BLE   
*/
/**************************************************************************/
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

  Serial.println("BLE VTF Peripheral Set");
}

void imu_setup(void){
	if (!IMU.begin()) {
    	Serial.println("Failed to initialize IMU!");
    	while (1);
	}
	filter.begin(sensorRate);
}


/**************************************************************************/
/*!
  @brief Setup all motors. Will all start in PRE_DEFINED mode   
*/
/**************************************************************************/
void motor_setup(void){
	driver.change_motor(MOTOR0);
    drv_0.begin();
    drv_0.useLRA();
	drv_0.selectLibrary(6);
    drv_0.setMode(DRV2605_MODE_INTTRIG); 
    drv_0.setWaveform(0, 0);
	drv_0.go();

	driver.change_motor(MOTOR1);
    drv_1.begin();
    drv_1.useLRA();
	drv_1.selectLibrary(6);
    drv_1.setMode(DRV2605_MODE_INTTRIG); 
    drv_1.setWaveform(0, 0);
	drv_1.go();

	driver.change_motor(MOTOR2);
    drv_2.begin();
    drv_2.useLRA();
	drv_2.selectLibrary(6);
    drv_2.setMode(DRV2605_MODE_INTTRIG); 
    drv_2.setWaveform(0, 0);
	drv_2.go();

	driver.change_motor(MOTOR3);
    drv_3.begin();
	drv_3.selectLibrary(1);
    drv_3.setMode(DRV2605_MODE_INTTRIG); 
    drv_3.setWaveform(0, 0);
	drv_3.go();

	driver.change_motor(MOTOR4);
    drv_4.begin();
	drv_4.selectLibrary(1);
    drv_4.setMode(DRV2605_MODE_INTTRIG); 
    drv_4.setWaveform(0, 0);
	drv_4.go();

	driver.change_motor(MOTOR5);
    drv_5.begin();
    drv_5.useLRA();
	drv_5.selectLibrary(6);
    drv_5.setMode(DRV2605_MODE_INTTRIG); 
    drv_5.setWaveform(0, 0);
	drv_5.go();

	driver.change_motor(MOTOR6);
    drv_6.begin();
    drv_6.useLRA();
	drv_6.selectLibrary(6);
    drv_6.setMode(DRV2605_MODE_INTTRIG); 
    drv_6.setWaveform(0, 0);
	drv_6.go();

	driver.change_motor(MOTOR7);
    drv_7.begin();
    drv_7.useLRA();
	drv_7.selectLibrary(6);
    drv_7.setMode(DRV2605_MODE_INTTRIG); 
    drv_7.setWaveform(0, 0);
	drv_7.go();

	driver.change_motor(MOTOR8);
    drv_8.begin();
	drv_8.selectLibrary(1);
    drv_8.setMode(DRV2605_MODE_INTTRIG); 
    drv_8.setWaveform(0, 0);
	drv_8.go();

	driver.change_motor(MOTOR9);
    drv_9.begin();
	drv_9.selectLibrary(1);
    drv_9.setMode(DRV2605_MODE_INTTRIG); 
    drv_9.setWaveform(0, 0);
	drv_9.go();
	
	driver.change_motor(MOTOR10);
    drv_10.begin();
    drv_10.useLRA();
	drv_10.selectLibrary(6);
    drv_10.setMode(DRV2605_MODE_INTTRIG); 
    drv_10.setWaveform(0, 0);
	drv_10.go();

	driver.change_motor(MOTOR11);
    drv_11.begin();
    drv_11.useLRA();
	drv_11.selectLibrary(6);
    drv_11.setMode(DRV2605_MODE_INTTRIG); 
    drv_11.setWaveform(0, 0);
	drv_11.go();

	driver.change_motor(MOTOR12);
    drv_12.begin();
    drv_12.useLRA();
	drv_12.selectLibrary(6);
    drv_12.setMode(DRV2605_MODE_INTTRIG); 
    drv_12.setWaveform(0, 0);
	drv_12.go();

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

	//rgb_setup();
	//bluetooth_setup();
	motor_setup();	
	//imu_setup();

	Serial.println("Setup Done");
  
}

void loop() {
	// put your main code here, to run repeatedly:
	//Serial.println("Setup Done");
	float xAcc, yAcc, zAcc;
	float xGyro, yGyro, zGyro;
	float roll, pitch, heading;
	int incomingByte = 0;
	int buffer = 20;
	int start_cnt = 0;
	int endcnt = 0;
	int getting_data = 0;
	int data_pos = 0;
	int* data_to_process;
	data_to_process = (int*)malloc(sizeof(int) * buffer);

	while(1){

		//BLE PROCESSING//
		 // listen for BLE peripherals to connect:
		BLEDevice central = BLE.central();

		// if a central is connected to peripheral:
		if (central) {
			digitalWrite(LED_BUILTIN, HIGH);            // turn on the LED to indicate the connection

			// while the central is still connected to peripheral:
			if(central.connected()) {
				digitalWrite(LED_BUILTIN, HIGH);
			}
		} else {
			digitalWrite(LED_BUILTIN, LOW); 
		}

		//Gyroscope Processing//
		// if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
		// 	IMU.readAcceleration(xAcc, yAcc, zAcc);
        // 	IMU.readGyroscope(xGyro, yGyro, zGyro);
		// 	filter.updateIMU(xGyro, yGyro, zGyro, xAcc, yAcc, zAcc);
		// 	roll = filter.getRoll();
		// 	pitch = filter.getPitch();
		// 	heading = filter.getYaw();
		// 	Serial.print("Roll: ");
		// 	Serial.print(roll);
		// 	Serial.print(" Pitch: ");
		// 	Serial.print(pitch);
		// 	Serial.print(" Heding: ");
		// 	Serial.println(heading);
			
		// }
		


		//SERIAL PROCESSING//
		if(Serial.available() > 0){
			incomingByte = Serial.read();
		}

		if(getting_data){
			if(incomingByte == EndBlock){
				endcnt ++;
				if(endcnt == 2){
					processBlock(data_to_process, data_pos);
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

		// if(current_vibration_mode == DEMO){
		// run_demo(current_demo_number);
		// }
			
	}

	
	
}
