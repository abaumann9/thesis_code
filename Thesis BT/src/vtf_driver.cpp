#include "Arduino.h"
#include <Wire.h>
#include "vtf_driver.h"


Adafruit_DRV2605 drv;



vtf_driver::vtf_driver() {}


/**************************************************************************/
/*!
  @brief Change to a single motor
  @param motorNum Motor to be selected
*/
/**************************************************************************/
void vtf_driver::change_motor(int motorNum){
    byte error;
    byte error1;


    //Serial.println(get_tcas(motorNum), HEX);
    //Serial.println(get_tcas_port(motorNum), DEC);


    Wire.beginTransmission(get_tcas(motorNum));
    Wire.write(1 << get_tcas_port(motorNum));
    error = Wire.endTransmission(get_tcas(motorNum));
    if (error) {
		Serial.print("Error occured when writing to ");
        if(get_tcas(motorNum) == TCAADDR0){
            Serial.println("TCAADDR0");
        } else {
            Serial.println("TCAADDR1");
        }
		if (error == 5)
		Serial.println("It was a timeout");
		Serial.println();
	}

    if(get_tcas(motorNum) == TCAADDR0){
        Wire.beginTransmission(TCAADDR1);
        Wire.write(0);
        error1 = Wire.endTransmission(TCAADDR1);
        if (error1) {
		    Serial.println("Error occured when writing to TCAADDR1");
		if (error1 == 5)
            Serial.println("It was a timeout");
            Serial.println();
        }
	} else {
        Wire.beginTransmission(TCAADDR0);
        Wire.write(0);
        error1 = Wire.endTransmission(TCAADDR0);
        if (error1) {
		    Serial.println("Error occured when writing to TCAADDR0");
		if (error1 == 5)
            Serial.println("It was a timeout");
            Serial.println();
        }
    }
    
}




/**************************************************************************/
/*!
  @brief Select multiple motor drivers at the same time for easy programming
  @param motors Motors to be set, Terminated with a -1
*/
/**************************************************************************/
void vtf_driver::tcas_set_multi(int *motors){
    uint8_t TCAADDR0_Addr = 0;
    uint8_t TCAADDR1_Addr = 0;


    for(int i = 0; i < 14; i++){ 
        if(motors[i] == -1){
            break;
        } else if(motors[i] == 0){
            TCAADDR1_Addr = TCAADDR1_Addr + (1<<3);

        } else if (motors[i] == 1){
            TCAADDR1_Addr = TCAADDR1_Addr + (1<<2);
            
        } else if (motors[i] == 2){
            TCAADDR1_Addr = TCAADDR1_Addr + (1<<1);
        
        } else if (motors[i] == 3){
            TCAADDR0_Addr = TCAADDR0_Addr + (1<<3);
        
        } else if (motors[i] == 4){
            TCAADDR0_Addr = TCAADDR0_Addr + (1<<2);
        
        } else if (motors[i] == 5){
            TCAADDR0_Addr = TCAADDR0_Addr + (1<<1);
        
        } else if (motors[i] == 6){
            TCAADDR0_Addr = TCAADDR0_Addr + (1<<0);
        
        } else if (motors[i] == 7){
            TCAADDR1_Addr = TCAADDR1_Addr + (1<<4);
        
        } else if (motors[i] == 8){
            TCAADDR1_Addr = TCAADDR1_Addr + (1<<5);
        
        } else if (motors[i] == 9){
            TCAADDR0_Addr = TCAADDR0_Addr + (1<<4);
        
        } else if (motors[i] == 10){
            TCAADDR0_Addr = TCAADDR0_Addr + (1<<5);
        
        } else if (motors[i] == 11){
            TCAADDR0_Addr = TCAADDR0_Addr + (1<<6);
        
        } else if (motors[i] == 12){
            TCAADDR0_Addr = TCAADDR0_Addr + (1<<7);

        }
    }
    // Serial.println("0x70 Add");
    // Serial.println(TCAADDR0_Addr, BIN);
    // Serial.println("0x74 Add");
    // Serial.println(TCAADDR1_Addr, BIN);
    
    Wire.beginTransmission(TCAADDR0);
    Wire.write(TCAADDR0_Addr);
    Wire.endTransmission(TCAADDR0);

    
	Wire.beginTransmission(TCAADDR1);
	Wire.write(TCAADDR1_Addr);
	Wire.endTransmission(TCAADDR1); 

}






/**************************************************************************/
/*!
  @brief Fire multipule motors at once
  @param motors Motors to be fired, Terminated with a -1
  @param drv_x A valid Motor drive initiated in main file (any will do)

*/
/**************************************************************************/
void vtf_driver::go_multi(int *motors, Adafruit_DRV2605 drv_x){
    tcas_set_multi(motors);
    drv_x.go();

}



/**************************************************************************/
/*!
  @brief Return the TCAS Address for a given Motor
  @param motorNum Motor number to return TCAs adress for

*/
/**************************************************************************/
uint8_t vtf_driver::get_tcas(int motorNum){
    if(motorNum < 13){
        if(motorNum == 0 || motorNum == 1 || motorNum == 2 || motorNum == 7 || motorNum == 8){
            return TCAADDR1;
        } else if(motorNum == 3 || motorNum == 4 || motorNum == 5 || motorNum == 6 || motorNum == 8 || motorNum == 9 || motorNum == 10 || motorNum == 11 || motorNum == 12) {
            return TCAADDR0;
        }
    }

    return TCAADDR0;

}


/**************************************************************************/
/*!
  @brief Return the TCAS port for a given motor
  @param motorNum Motor number to return TCAs port for

*/
/**************************************************************************/
uint8_t vtf_driver::get_tcas_port(int motorNum){
    if(motorNum < 13){
        if(motorNum == 6){
            return 0;
        } else if (motorNum == 2 || motorNum == 5){
            return 1;
        } else if (motorNum == 1 || motorNum == 4){
            return 2;
        } else if (motorNum == 0 || motorNum == 3){
            return 3;
        } else if (motorNum == 7 || motorNum == 9){
            return 4;
        } else if (motorNum == 8 || motorNum == 10){
            return 5;
        } else if (motorNum == 11){
            return 6;
        } else if (motorNum == 12){
            return 7;
        }

    }

    return -1;

}


/**************************************************************************/
/*!
  @brief Setup an LRA using the DRV2605 Auto Calibrate. Will only work
         with the ... LRA motor. Params must be changed for other motors
  @param motorNum Motor to be setup
  @param drv_x DRV2605 instance for given motorNum initiated in main

*/
/**************************************************************************/
void vtf_driver::LRA_setup(int motorNum, Adafruit_DRV2605 drv_x){
    change_motor(motorNum);
    drv_x.begin();
    drv_x.useLRA();
    // drv_x.writeRegister8(0x01, 0x00);     // out of standby

    // drv_x.writeRegister8(0x16, 0x63);                                               //Set RATED_VOLTAGE ->99
    // drv_x.writeRegister8(0x17, 0xA1);                                               //Set OD_CLAMP (Max Allowed Voltage) ->99

    // uint8_t feedback_val = 0;
    // feedback_val |= (1 << 7);                                                       // Set LRA mode
    // feedback_val |= (2 << 4);                                                       // Set Brake Factor to 2x
    // feedback_val |= (2 << 2);                                                       // Set Loop Gain to high
    // drv_x.writeRegister8(0x1A, feedback_val);

    // //Control 1
    // drv_x.writeRegister8(0x1B, (drv_x.readRegister8(0x1B) & 0xE0));                 //Clear DRIVE_TIME Bits
    // drv_x.writeRegister8(0x1B, (drv_x.readRegister8(0x1B) | (0xB)));                //Set DRIVE_TIME 11

    // //Control 2
    // drv_x.writeRegister8(0x1C, (drv_x.readRegister8(0x1C) & 0xCF));                 //Clear SAMPLE_TIME Bits
    // drv_x.writeRegister8(0x1C, (drv_x.readRegister8(0x1C) | (0x03<<4)));            //Set SAMPLE_TIME -> 3

    // //Control 3
    // drv_x.writeRegister8(0x1C, (drv_x.readRegister8(0x1C) & 0xF0));                 //Clear BLANKING_TIME & IDISS_TIME Bits
    // drv_x.writeRegister8(0x1C, (drv_x.readRegister8(0x1C) | (1<<2)));               //Set BLANKING_TIME -> 1
    // drv_x.writeRegister8(0x1C, (drv_x.readRegister8(0x1C) | (1<<0)));               //Set IDISS_TIME -> 1

    // drv_x.writeRegister8(DRV2605_REG_MODE, 0x07);                                   //Auto Calibration Mode

    // drv_x.writeRegister8(0x1E, (drv_x.readRegister8(0x1E) | (032<<4)));             //Set AUTO_CAL_TIME -> 3

    // drv_x.go();
    // int cnt = 0;
    // while((drv_x.readRegister8(DRV2605_REG_GO)  & 0x01)!=0){
    
    //     delay(250);
    //     cnt ++;
    

    //     if(cnt > 20){
    //         Serial.println("Timeout");
    //         //drv_x.writeRegister8(DRV2605_REG_GO, drv_x.readRegister8(DRV2605_REG_GO)  - 0x01);
    //         drv_x.begin();
    //         drv_x.useLRA();
    //         break;
    //     }
    // }

    // if((drv_x.readRegister8(0x00) & (0x08))){
    //     Serial.println("AUTO Calibration Failed");
        
    // }
    
    drv_x.selectLibrary(6);
    drv_x.setMode(DRV2605_MODE_INTTRIG); 
    drv_x.setWaveform(0, 0);
	drv_x.go();
    //Serial.println("AUTO Calibration Done");
    return;
}

/**************************************************************************/
/*!
  @brief Setup an ERM motor
  @param motorNum Motor to be setup
  @param drv_x DRV2605 instance for given motorNum initiated in main

*/
/**************************************************************************/
void vtf_driver::ERM_setup(int motorNum, Adafruit_DRV2605 drv_x){
    change_motor(motorNum);
    drv_x.begin();
    drv_x.selectLibrary(1);
	drv_x.setMode(DRV2605_MODE_INTTRIG); 
	drv_x.setWaveform(0, 0);
    drv_x.go();
}