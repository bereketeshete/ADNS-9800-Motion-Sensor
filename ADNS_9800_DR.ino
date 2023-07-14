#include <SPI.h>
#include <avr/pgmspace.h>

// Registers
// ADNS-9800 Registers
#define REG_Product_ID                           0x00
#define REG_Revision_ID                          0x01
#define REG_Motion                               0x02
#define REG_Delta_X_L                            0x03
#define REG_Delta_X_H                            0x04
#define REG_Delta_Y_L                            0x05
#define REG_Delta_Y_H                            0x06
#define REG_SQUAL                                0x07
#define REG_Pixel_Sum                            0x08
#define REG_Maximum_Pixel                        0x09
#define REG_Minimum_Pixel                        0x0a
#define REG_Shutter_Lower                        0x0b
#define REG_Shutter_Upper                        0x0c
#define REG_Frame_Period_Lower                   0x0d
#define REG_Frame_Period_Upper                   0x0e
#define REG_Configuration_I                      0x0f
#define REG_Configuration_II                     0x10
#define REG_Frame_Capture                        0x12
#define REG_SROM_Enable                          0x13
#define REG_Run_Downshift                        0x14
#define REG_Rest1_Rate                           0x15
#define REG_Rest1_Downshift                      0x16
#define REG_Rest2_Rate                           0x17
#define REG_Rest2_Downshift                      0x18
#define REG_Rest3_Rate                           0x19
#define REG_Frame_Period_Max_Bound_Lower         0x1a
#define REG_Frame_Period_Max_Bound_Upper         0x1b
#define REG_Frame_Period_Min_Bound_Lower         0x1c
#define REG_Frame_Period_Min_Bound_Upper         0x1d
#define REG_Shutter_Max_Bound_Lower              0x1e
#define REG_Shutter_Max_Bound_Upper              0x1f
#define REG_LASER_CTRL0                          0x20
#define REG_Observation                          0x24
#define REG_Data_Out_Lower                       0x25
#define REG_Data_Out_Upper                       0x26
#define REG_SROM_ID                              0x2a
#define REG_Lift_Detection_Thr                   0x2e
#define REG_Configuration_V                      0x2f
#define REG_Configuration_IV                     0x39
#define REG_Power_Up_Reset                       0x3a
#define REG_Shutdown                             0x3b
#define REG_Inverse_Product_ID                   0x3f
#define REG_Motion_Burst                         0x50
#define REG_SROM_Load_Burst                      0x62
#define REG_Pixel_Burst                          0x64

//Set this to what pin your "INT0" hardware interrupt feature is on
#define Motion_Interrupt_Pin 2

const int ncs = 10;  //This is the SPI "slave select" pin that the sensor is hooked up to

uint8_t PIN_NCS;


void setup() {
  Serial.begin(9600);
  Serial.print("ADNS 9800 code_intitialized\n");

  pinMode (ncs, OUTPUT);  
  
  pinMode(Motion_Interrupt_Pin, INPUT);
  digitalWrite(Motion_Interrupt_Pin, HIGH);
  //attachInterrupt(9, UpdatePointer, FALLING);

  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV128);
}


// void UpdatePointer(void){
//   if(initComplete==9){
//     //write 0x01 to Motion register and read from it to freeze the motion values and make them available
//     adns_write_reg(Motion, 0x01);
//     adns_read_reg(Motion);
//     int16_t DELTA_X = adns_read_reg(Delta_X_L) + (int16_t)(adns_read_reg(Delta_X_H) << 8);
//     int16_t DELTA_Y = adns_read_reg(Delta_Y_L) + (int16_t)(adns_read_reg(Delta_Y_H) << 8); 
//     xydat[0] = DELTA_X;
//     xydat[1] = DELTA_Y;
//     movementflag=1;
//   }
// }



void loop() {
 
}

void writeRegister(const uint8_t reg, uint8_t output) {
  // Enable communication
  digitalWrite( PIN_NCS, LOW );
  SPI.transfer( reg | B10000000 );
  // Send value
  SPI.transfer( output );
  // Disable communcation
  digitalWrite( PIN_NCS, HIGH );
  delayMicroseconds( ADNS3080_T_SWW );

uint8_t readRegister( const uint8_t );

void motionBurst( uint8_t*, int8_t*, int8_t*, uint8_t*, uint16_t*, uint8_t* );




