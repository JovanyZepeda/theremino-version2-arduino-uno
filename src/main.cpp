/*
   
Theremino V2 Arduino Project



Note:
  Speaker must be connected between + 100 ohm resister
    Pin 9
    Pin 10

  Rotary Encoder:
    CLK => 1
    DT => 2
    SW => 0

  Gyro:
    SCL => SCL
    SDA => SDA

  Sonar:
    VOLUME ECHO Pin => 3
    VOLUME TRIG Pin => 4
  
    Pitch ECHO Pin => 5
    Pitch TRIG Pin => 6

*/
#include <Arduino.h>
#include <Wire.h>
#include <NewPing.h>
#include <toneAC.h>


//======================== SENSOR PIN LAYOUT START ======================================//
// Rotary Encoder Inputs
#define CLK 1
#define DT 2
#define SW 0
#define VOLUME_SONAR_ECHO_PIN 4
#define VOLUME_SONAR_TRIG_PIN 5 
#define PITCH_SONAR_ECHO_PIN 7 
#define PITCH_SONAR_TRIG_PIN 6
//======================== SENSOR PIN LAYOUT END======================================//


//==========================Variable Initialization Start =====================//

// -------------------------MPU VARS Start ------------------------//
const int MPU = 0x68; // MPU6050 I2C address From Datasheet
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw; // <------------------------------------------OUTPUT GYRO ANGLES HERE
float roll_temp, pitch_temp, yaw_temp; // <------------------Temporary OUTPUT GYRO ANGLES HERE
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
int error_samples = 1000;

float roll_temp_MAX = 30;
float roll_temp_MIN = -30;
float pitch_temp_MAX = 30;
float pitch_temp_MIN = -30;
float yaw_temp_MAX = 30;
float yaw_tamp_MIN = -30;


bool GYRO_DEBUG = false;

// --------- SERIAL MONITER MENU VARS Start ------------------ //
bool EM1_GYRO = false;
bool EM2_FOOT_SONAR = false;
bool EM3_SONAR_ONLY = false;
bool EM1_DEBUG = false;
bool EM2_DEBUG = false;
bool EM3_DEBUG = false;
bool SPEAKER_OUTPUT_DEBUG = false;
bool SPEAKER_PLAY = false;

// ----------------Sonar Vars Start--------------------------------//
#define MAX_DISTANCE  30  // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
int FREQ_OUTPUT = 0; // [Hz]
int VOLUME_OUTPUT = 0; // [0 to 10]
float VolumeSonarDistance = 0; //[cm]
float PitchSonarUs = 0; //[us]
int PitchSonarDistance = 0; //[cm]

NewPing sonar[2]= { // Sonar array for both sonar sensors
  NewPing(VOLUME_SONAR_TRIG_PIN, VOLUME_SONAR_ECHO_PIN, MAX_DISTANCE), //Volume Sonar
  NewPing(PITCH_SONAR_TRIG_PIN, PITCH_SONAR_ECHO_PIN, MAX_DISTANCE) // Pitch Sonar
};

// ----------------Rotary Encorder Vars Start--------------------------------//
int counter = 0;
int maxCounterCount = 10;
int currentStateCLK;
int lastStateCLK;
String currentDir ="";
unsigned long lastButtonPress = 0;

//----------------Speaker Variables ----------------------------//
int frequency_output_maximum = 10000;
int freqeuncy_output_minimum = 10;

//==========================Variable Initialization End =====================//


//===================function prototype Start=================//
void calculate_IMU_error(); 
void read_MPU_data();
void configure_MPU_register();
void print_MPU_gyro_xyz_angles();
void serial_monitor_menu_EM();
void serial_monitor_menu_speaker();
void Serial_monitor_menu_debug();
void setup_rotary_encoder();
void read_encoder();
void read_sonar_sensor(int which_sonar);
void play_tone_on_speaker();
// =========================== ARDUINO CODE ==============================//
void setup() {
  Serial.begin(9600);
  
  serial_monitor_menu_EM();
  serial_monitor_menu_speaker();
  Serial_monitor_menu_debug();

  //Depending on 
  if (EM1_GYRO){
    configure_MPU_register();

  } else 
  if(EM2_FOOT_SONAR){
    setup_rotary_encoder();

  } else 
  if(EM3_SONAR_ONLY){
    // SONAR ALREADY SET UP IN VARIABLE INITIALIZATION

  }

 
  delay(20);
}


void loop() {

  if(EM1_GYRO){
    read_MPU_data();
  } else 
  if(EM2_FOOT_SONAR){
    read_encoder();
    read_sonar_sensor(1);
  } else
  if(EM3_SONAR_ONLY){
    read_sonar_sensor(0);
    read_sonar_sensor(1);
  }

  if(SPEAKER_PLAY){
    play_tone_on_speaker();
  } else {
    noToneAC();
  }

}


// ============================  Functions ====================================//
//--------------GYRO Function Start--------------------//

void print_MPU_gyro_xyz_angles(){

  if(GYRO_DEBUG) // Print the values on the serial monitor
  {
    Serial.print(roll * 4.5); // |ROLL (X) = 
    Serial.print("/");
    Serial.print(pitch * 4.5); // | PITCH (y) = 
    Serial.print("/"); 
    Serial.println(yaw * 4.5); //| YAW (Z)= }
  }
}

void configure_MPU_register(){
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
}

void read_MPU_data(){
 // ====================================================== Read acceleromter data ============================================= //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) + AccErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY; // AccErrorY ~(-1.58)
  // ========================================================= Read gyroscope data =============================================== //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read

  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // =============================================== Correct the outputs with the calculated error values START=============================//
  GyroX = GyroX - GyroErrorX; // GyroErrorX ~(-0.56)
  GyroY = GyroY + GyroErrorY; // GyroErrorY ~(2)
  GyroZ = GyroZ - GyroErrorZ; // GyroErrorZ ~ (-0.8)
  // =============================================== Correct the outputs with the calculated error values =============================//

  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  
  //FINAL OUTPUT ARE
  //   YAW   for Z axis
  //   ROLL  for x axis
  //   PITCH for y axis

  //Constrain the gyro to a set interval of acceptable values
  roll_temp  = constrain(roll,roll_temp_MIN, roll_temp_MAX );
  pitch_temp = constrain(pitch,pitch_temp_MIN, pitch_temp_MAX);
  yaw_temp   = constrain(yaw_temp, yaw_tamp_MIN, yaw_temp_MAX);

  //Generate FREQ and VOLUME OUTPUT
  FREQ_OUTPUT   =  map(pitch_temp, pitch_temp_MIN, pitch_temp_MAX, freqeuncy_output_minimum, frequency_output_maximum ); // Use PITCH OR Y AXIS
  VOLUME_OUTPUT =  map(yaw_temp, yaw_tamp_MIN, yaw_temp_MAX, freqeuncy_output_minimum, frequency_output_maximum); // Use YAW or Z axis
}

void calculate_IMU_error() {

  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < error_samples) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / error_samples;
  AccErrorY = AccErrorY / error_samples;
  c = 0;
  // Read gyro values 200 times
  while (c < error_samples) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / error_samples;
  GyroErrorY = GyroErrorY / error_samples;
  GyroErrorZ = GyroErrorZ / error_samples;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}

// ---------------SERIAL MOnitor Menu Functions Start ----------//
void serial_monitor_menu_EM(){

  Serial.println("|**********************************************|");
  Serial.println("|**|Theremino V2 External Module Selection  |**|");
  Serial.println("|**|                  Menu                  |**|");
  Serial.println("|**********************************************|");
  Serial.println("");
  Serial.println("Select one of the following options:");
  Serial.println("EM#1: Gyro Head Band           -> ENTER 1");
  Serial.println("EM#2: Foot Pedal and One Sonar -> ENTER 2");
  Serial.println("EM#3: Sonar Sensor             -> ENTER 3");

  while(!Serial.available()){
    int USER_INPUT = Serial.parseInt(); // GET USER INPUT from Serial monitor

    switch(USER_INPUT){
      case 1:
        EM1_GYRO = true;
        Serial.println("External Module #1 Selected: Gyro Head Band");
        break;
      case 2:
        EM2_FOOT_SONAR = true;
        Serial.println("External Module #2 Selected: Foot Pedal and One Sonar");
        break;
      case 3:
        EM3_SONAR_ONLY = true;
        Serial.println("External Module #3 Selected: Sonar Sensor");
        break;
      default:
        Serial.println("Unrecognized command, try again!");
        serial_monitor_menu_EM(); //RESTART MENU PROGRAM 
    } 
  }

  Serial.println("*******EXIT EXTERNAL MODULE SELECTION MENU ********\n\n");

}

void Serial_monitor_menu_debug(){
  Serial.println("|**********************************************|");
  Serial.println("|**|      Theremino V2 DEBUG Selection      |**|");
  Serial.println("|**|                  Menu                  |**|");
  Serial.println("|**********************************************|");
  Serial.println("");
  Serial.println("Select one of the following options:");
  Serial.println("EM#1: Print GYRO and freqeuncy/volume values                  -> ENTER 1");
  Serial.println("EM#2: Print rotary encoder values and freqeuncy/volume values -> ENTER 2");
  Serial.println("EM#3: Print sonar data and freqeuncy/volume values            -> ENTER 3");

  while(!Serial.available()){
    int USER_INPUT = Serial.parseInt(); // GET USER INPUT from Serial monitor

    switch(USER_INPUT){
      case 1:
        EM1_DEBUG = true;
        Serial.println("External Module #1 debug Selected");
        break;
      case 2:
        EM2_DEBUG = true;
        Serial.println("External Module #2 debug Selected");
        break;
      case 3:
        EM3_DEBUG = true;
        Serial.println("External Module #3 debug Selected");
        break;
      default:
        Serial.println("Unrecognized command, try again!");
        Serial_monitor_menu_debug(); //RESTART MENU PROGRAM 
    } 
  }

  Serial.println("*******EXIT EXTERNAL MODULE SELECTION MENU ********\n\n");
}

void serial_monitor_menu_speaker(){

  Serial.println("|************************************************|");
  Serial.println("|**|      Theremino V2 Speaker Selection      |**|");
  Serial.println("|**|                  Menu                    |**|");
  Serial.println("|************************************************|");
  Serial.println("");
  Serial.println("Select one of the following options:");
  Serial.println("Play Sound Through the speaker     -> ENTER 1");
  Serial.println("Do Not Play Sound through speaker  -> ENTER 2");

  while(!Serial.available()){
    int USER_INPUT = Serial.parseInt(); // GET USER INPUT from Serial monitor

    switch(USER_INPUT){
      case 1:
        SPEAKER_PLAY = true;
        Serial.println("Speaker will play tone");
        break;
      case 2:
        SPEAKER_PLAY = true;
        Serial.println("Speaker will not play tone");
        break;
      default:
        Serial.println("Unrecognized command, try again!\n\n");
        serial_monitor_menu_speaker(); //RESTART MENU PROGRAM
    }
  }
}

// --------------- Rotary Encoder Functions Start ----------//
void setup_rotary_encoder(){

 // Set encoder pins as inputs
  pinMode(CLK,INPUT);
  pinMode(DT,INPUT);
  pinMode(SW, INPUT_PULLUP);

  // Read the initial state of CLK
  lastStateCLK = digitalRead(CLK);

}

void read_encoder(){
  // Read the current state of CLK
  currentStateCLK = digitalRead(CLK);

  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){

    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(DT) != currentStateCLK) {
      counter --;
      currentDir ="CCW";
    } else {
      // Encoder is rotating CW so increment
      counter ++;
      currentDir ="CW";
    }

    if (EM2_DEBUG){ //DEBUGING INFORMATION IF TRUE
      Serial.print("Direction: ");
      Serial.print(currentDir);
      Serial.print(" | Counter: ");
      Serial.println(counter);
    }
  }

  // Remember last CLK state
  lastStateCLK = currentStateCLK;

  //constrain the counter output to a specfic interval
  if(counter < 0){
    counter = 0;
  }  else 
  if(counter > maxCounterCount){
    counter = maxCounterCount;
  }

  //Re-scale distance values using Map Function
  VOLUME_OUTPUT = map(counter, 0, maxCounterCount, 0, 10); // 0 (off) up to 10 (loudest)

  // Read the button state
  int btnState = digitalRead(SW);

  //If we detect LOW signal, button is pressed
  if (btnState == LOW) {
    //if 50ms have passed since last LOW pulse, it means that the
    //button has been pressed, released and pressed again
    if (millis() - lastButtonPress > 50) {
      counter = 0;

      if(EM2_DEBUG){ //DEBUGING INFORMATION IF TRUE
        Serial.println("Button pressed: COUNTER RESET!");
      }
    }

    // Remember last button press event
    lastButtonPress = millis();
  }
  // Put in a slight delay to help debounce the reading
  delay(1);
}

// ---------------Sonar Functions Start ----------//
void read_sonar_sensor(int which_sonar){  // 0 = Volume sonar, 1 = freq sonar


  if(which_sonar = 0){

    //Get Distances Values in CM
    VolumeSonarDistance = sonar[which_sonar].ping_cm();

    //Re-scale distance values using Map Function
    VOLUME_OUTPUT = map(VolumeSonarDistance, 0, MAX_DISTANCE, 0, 10); // 0 (off) up to 10 (loudest)

  } else 
  
  if(which_sonar = 1){

    //Get Distances Values in CM
    PitchSonarUs = sonar[which_sonar].ping();
    PitchSonarDistance = sonar[which_sonar].ping_cm();

    //Re-scale distance values using Map Function
    FREQ_OUTPUT = map(PitchSonarDistance, 0, MAX_DISTANCE, freqeuncy_output_minimum, frequency_output_maximum); // 0 Hz up to 10 KHz

  } else {
    FREQ_OUTPUT = 0;
    VOLUME_OUTPUT = 0;
  }

}

void sonar_sensor_debug(){
  //Log data for debugging
  if (SPEAKER_OUTPUT_DEBUG){
    //Print Distance Values
    Serial.print("| Volume Distance CM: ");
    Serial.print(VolumeSonarDistance);
    Serial.print("\t");

    //Print Distance Values
    Serial.print("| Pitch Microseconds uS: ");
    Serial.print(PitchSonarUs);
    Serial.print("\t");

    Serial.print("| Pitch Distance CM: ");
    Serial.print(PitchSonarDistance);
    Serial.print("\n");
    
    //Print Freqeuncy
    Serial.print("| Freqeuncy Hz: ");
    Serial.print(FREQ_OUTPUT);
    Serial.print("\t");
    
    //Print Volume Level
     Serial.print("| Volume Level: ");
    Serial.print(VOLUME_OUTPUT);
    Serial.print("\n");
  }
}

// ---------------Speaker Functions Start    ----------//
void play_tone_on_speaker(){
  toneAC(FREQ_OUTPUT,VOLUME_OUTPUT);
}
