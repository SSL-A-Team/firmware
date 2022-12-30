/*
 * This example bypasses the hardware motion interrupt pin
 * and polls the motion data registers at a fixed interval
 */

#include <SPI.h>
#include <avr/pgmspace.h>

// Registers
#define Product_ID  0x00
#define Revision_ID 0x01
#define Motion  0x02
#define Delta_X_L 0x03
#define Delta_X_H 0x04
#define Delta_Y_L 0x05
#define Delta_Y_H 0x06
#define SQUAL 0x07
#define Raw_Data_Sum  0x08
#define Maximum_Raw_data  0x09
#define Minimum_Raw_data  0x0A
#define Shutter_Lower 0x0B
#define Shutter_Upper 0x0C
#define Control 0x0D
#define Config1 0x0F
#define Config2 0x10
#define Angle_Tune  0x11
#define Frame_Capture 0x12
#define SROM_Enable 0x13
#define Run_Downshift 0x14
#define Rest1_Rate_Lower  0x15
#define Rest1_Rate_Upper  0x16
#define Rest1_Downshift 0x17
#define Rest2_Rate_Lower  0x18
#define Rest2_Rate_Upper  0x19
#define Rest2_Downshift 0x1A
#define Rest3_Rate_Lower  0x1B
#define Rest3_Rate_Upper  0x1C
#define Observation 0x24
#define Data_Out_Lower  0x25
#define Data_Out_Upper  0x26
#define Raw_Data_Dump 0x29
#define SROM_ID 0x2A
#define Min_SQ_Run  0x2B
#define Raw_Data_Threshold  0x2C
#define Config5 0x2F
#define Power_Up_Reset  0x3A
#define Shutdown  0x3B
#define Inverse_Product_ID  0x3F
#define LiftCutoff_Tune3  0x41
#define Angle_Snap  0x42
#define LiftCutoff_Tune1  0x4A
#define Motion_Burst  0x50
#define LiftCutoff_Tune_Timeout 0x58
#define LiftCutoff_Tune_Min_Length  0x5A
#define SROM_Load_Burst 0x62
#define Lift_Config 0x63
#define Raw_Data_Burst  0x64
#define LiftCutoff_Tune2  0x65

//Set this to what pin your "INT0" hardware interrupt feature is on
#define Motion_Interrupt_Pin 9

const int left_cs_n = D4;  //This is the SPI "slave select" pin that the sensor is hooked up to
const int right_cs_n = D5;  //This is the SPI "slave select" pin that the sensor is hooked up to

byte initComplete=0;
volatile int xydat[2];
volatile byte movementflag_left  = 0;
volatile byte movementflag_right = 0;

byte testctr=0;
unsigned long currTime;
unsigned long timer;
unsigned long pollTimer;

//Be sure to add the SROM file into this sketch via "Sketch->Add File"
extern const unsigned short firmware_length;
extern const unsigned char firmware_data[];

void setup() {
  Serial.begin(9600);
  
  pinMode (left_cs_n, OUTPUT);
  pinMode (right_cs_n, OUTPUT);

  // Set high to not select
  digitalWrite(left_cs_n, HIGH);
  digitalWrite(right_cs_n, HIGH);


  pinMode(Motion_Interrupt_Pin, INPUT);
  digitalWrite(Motion_Interrupt_Pin, HIGH);
  attachInterrupt(9, UpdatePointer, FALLING);

  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  //SPI.setClockDivider(4);

  
  performStartup(left_cs_n);  
  
  delay(5000);
  
  dispRegisters(left_cs_n);
  initComplete=9;

}

void adns_com_begin(int pin){
  digitalWrite(pin, LOW);
}

void adns_com_end(int pin){
  digitalWrite(pin, HIGH);
}

byte adns_read_reg(int pin, byte reg_addr){
  adns_com_begin(pin);
  
  // send adress of the register, with MSBit = 0 to indicate it's a read
  SPI.transfer(reg_addr & 0x7f );
  delayMicroseconds(100); // tSRAD
  // read data
  byte data = SPI.transfer(0);
  
  delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns
  adns_com_end(pin);
  delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-NCS

  return data;
}

void adns_write_reg(int pin, byte reg_addr, byte data){
  adns_com_begin(pin);
  
  //send adress of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(reg_addr | 0x80 );
  //sent data
  SPI.transfer(data);
  
  delayMicroseconds(20); // tSCLK-NCS for write operation
  adns_com_end(pin);
  delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound 
}

void adns_upload_firmware(int pin){
  // send the firmware to the chip, cf p.18 of the datasheet
  Serial.println("Uploading firmware...");

  //Write 0 to Rest_En bit of Config2 register to disable Rest mode.
  adns_write_reg(pin, Config2, 0x20);
  
  // write 0x1d in SROM_enable reg for initializing
  adns_write_reg(pin, SROM_Enable, 0x1d); 
  
  // wait for more than one frame period
  delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low
  
  // write 0x18 to SROM_enable to start SROM download
  adns_write_reg(pin, SROM_Enable, 0x18); 
  
  // write the SROM file (=firmware data) 
  adns_com_begin(pin);
  SPI.transfer(SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);
  
  // send all bytes of the firmware
  unsigned char c;
  for(int i = 0; i < firmware_length; i++){ 
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }

  //Read the SROM_ID register to verify the ID before any other register reads or writes.
  adns_read_reg(pin,SROM_ID);

  //Write 0x00 to Config2 register for wired mouse or 0x20 for wireless mouse design.
  adns_write_reg(pin,Config2, 0x00);

  // set initial CPI resolution
  adns_write_reg(pin, pinConfig1, 0x15);
  
  adns_com_end(pin);
  }


void performStartup(int pin){
  adns_com_end(pin); // ensure that the serial port is reset
  adns_com_begin(pin); // ensure that the serial port is reset
  adns_com_end(pin); // ensure that the serial port is reset
  adns_write_reg(pin, Power_Up_Reset, 0x5a); // force reset
  delay(50); // wait for it to reboot
  // read registers 0x02 to 0x06 (and discard the data)
  adns_read_reg(pin, Motion);
  adns_read_reg(pin, Delta_X_L);
  adns_read_reg(pin, Delta_X_H);
  adns_read_reg(pin, Delta_Y_L);
  adns_read_reg(pin, Delta_Y_H);
  // upload the firmware
  adns_upload_firmware(pin);
  delay(10);
  Serial.println("Optical Chip Initialized for pin ");
  Serial.print(pin);
  }

void UpdatePointerLeft(){
  if(initComplete==9){

    //write 0x01 to Motion register and read from it to freeze the motion values and make them available
    adns_write_reg(left_cs_n, Motion, 0x01);
    adns_read_reg(left_cs_n, Motion);

    xydat[0] = (int)adns_read_reg(left_cs_n,Delta_X_L);
    xydat[1] = (int)adns_read_reg(left_cs_n,Delta_Y_L);
    
    movementflag_left=1;
    }
  }

void dispRegisters(int pin){
  int oreg[7] = {
    0x00,0x3F,0x2A,0x02  };
  char* oregname[] = {
    "Product_ID","Inverse_Product_ID","SROM_Version","Motion"  };
  byte regres;

  adns_com_begin(pin);

  int rctr=0;
  for(rctr=0; rctr<4; rctr++){
    SPI.transfer(oreg[rctr]);
    delay(1);
    Serial.println("---");
    Serial.println(oregname[rctr]);
    Serial.println(oreg[rctr],HEX);
    regres = SPI.transfer(0);
    Serial.println(regres,BIN);  
    Serial.println(regres,HEX);  
    delay(1);
  }
  adns_com_end(pin);
}


int convTwosComp(int b){
  //Convert from 2's complement
  if(b & 0x80){
    b = -1 * ((b ^ 0xff) + 1);
    }
  return b;
  }
  

void loop() {

  currTime = millis();
  
  if(currTime > timer){    
    Serial.println(testctr++);
    timer = currTime + 2000;
    }
    
  if(currTime > pollTimer){
    UpdatePointer();
    xydat[0] = convTwosComp(xydat[0]);
    xydat[1] = convTwosComp(xydat[1]);
      if(xydat[0] != 0 || xydat[1] != 0){
        Serial.print("x = ");
        Serial.print(xydat[0]);
        Serial.print(" | ");
        Serial.print("y = ");
        Serial.println(xydat[1]);
        }
    pollTimer = currTime + 20;
    }
    
  }
