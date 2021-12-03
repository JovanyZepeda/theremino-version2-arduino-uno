/*
   Arduino and MPU6050 Accelerometer and Gyroscope Sensor Tutorial
   by Dejan, https://howtomechatronics.com
*/
#include <Arduino.h>
#include <Wire.h>

//==========================Variable Initialization Start =====================//

// -------------------------MPU VARS Start ------------------------//
const int MPU = 0x68; // MPU6050 I2C address From Datasheet
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw; // <------------------------------------------OUTPUT GYRO ANGLES HERE
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
int error_samples = 1000;

bool GYRO_DEBUG = false;
// -------------------------MPU VARS  End------------------------//

// --------- SERIAL MONITER MENU VARS Start ----------------- //
bool EM1_GYRO = false;
bool EM2_FOOT_SONAR = false;
bool EM3_SONAR_ONLY = false;
bool EM1_DEBUG = false;
bool EM2_DEBUG = false;
bool EM3_DEBUG = false;
// --------- SERIAL MONITER MENU VARS End   ----------------- //

// ----------------Speaker Vars Start--------------------------------//

// ----------------Speaker Vars End --------------------------------//


//==========================Variable Initialization End =====================//


//===================function prototype Start=================//
void calculate_IMU_error(); 
void read_MPU_data();
void configure_MPU_register();
void print_MPU_gyro_xyz_angles();
//===================function prototype End===================//



void setup() {
  Serial.begin(9600);
  configure_MPU_register();
  // Call this function if you need to get the IMU error values for your module
  calculate_IMU_error();
  delay(20);
}


void loop() {
  read_MPU_data(); 
  print_MPU_gyro_xyz_angles();

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

//-----------------GYRO Function End--------------------//

// ---------------SERIAL MOnitor Menu Functions Start ----------//
void serial_monitor_menu_EM(){

  Serial.begin(9600);
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

// ---------------SERIAL MOnitor Menu Functions End ----------//


// --------------- Rotary Encoder Functions Start ----------//
void setup_rotary_encoder(){

  Serial.begin(9600); // start serial coms



}

void read_read_encoder(){

}
// ---------------Rotary Encoder Functions End    ----------//


// ---------------Sonar Functions Start ----------//
void setup_sonar_sensor(){

}

void read_sonar_sensor(){

}

// ---------------Sonar Functions End    ----------//


// ---------------Speaker Functions Start    ----------//
void setup_speaker(){
}

void play_tone_on_speaker(){
}
// ---------------Speaker Functions End    ----------//
