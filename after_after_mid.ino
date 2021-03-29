#include <Wire.h>
#include <MadgwickAHRS.h>

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

//initializations
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

// MPU9250 Slave Device Address
const uint8_t MPU9250SlaveAddress = 0x68;

// Pins for serial data
const uint8_t scl = D6;
const uint8_t sda = D7;

// sensitivity scale factor of accelerometer and gyroscope 
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

// MPU9250 few configuration register addresses
const uint8_t MPU9250_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU9250_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU9250_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU9250_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU9250_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU9250_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU9250_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU9250_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU9250_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU9250_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU9250_REGISTER_SIGNAL_PATH_RESET  = 0x68;

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;

void setup() {
  Serial.begin(9600);
  Wire.begin(sda, scl);
  MPU9250_Init();
  filter.begin(25);
  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
}



void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelY = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelZ = (((int16_t)Wire.read()<<8) | Wire.read());
  Temperature = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroX = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroY = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroZ = (((int16_t)Wire.read()<<8) | Wire.read());
}

//configure MPU9250
void MPU9250_Init(){
  delay(150);
  I2C_Write(MPU9250SlaveAddress, MPU9250_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU9250SlaveAddress, MPU9250_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU9250SlaveAddress, MPU9250_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU9250SlaveAddress, MPU9250_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU9250SlaveAddress, MPU9250_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU9250SlaveAddress, MPU9250_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU9250SlaveAddress, MPU9250_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU9250SlaveAddress, MPU9250_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU9250SlaveAddress, MPU9250_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU9250SlaveAddress, MPU9250_REGISTER_USER_CTRL, 0x00);
}
void loop() {
  float Ax, Ay, Az, T, Gx, Gy, Gz;
  unsigned long microsNow;
  
  Read_RawValue(MPU9250SlaveAddress, MPU9250_REGISTER_ACCEL_XOUT_H);
  
  //divide each with their sensitivity scale factor
  Ax = (float)AccelX/AccelScaleFactor;
  Ay = (float)AccelY/AccelScaleFactor;
  Az = (float)AccelZ/AccelScaleFactor;
  T = (float)Temperature/340+36.53; //temperature formula
  Gx = (float)GyroX/GyroScaleFactor;
  Gy = (float)GyroY/GyroScaleFactor;
  Gz = (float)GyroZ/GyroScaleFactor;


// check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
    // update the filter, which computes orientation
    filter.updateIMU(Gx, Gy, Gz, Ax, Ay, Az);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();

  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.println(yaw);

     // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
    // Print the values on the serial monitor
 
  }




/*
  Serial.print("Ax: "); Serial.print(Ax);
  Serial.print(" Ay: "); Serial.print(Ay);
  Serial.print(" Az: "); Serial.print(Az);
  Serial.print(" T: "); Serial.print(T);
  Serial.print(" Gx: "); Serial.print(Gx);
  Serial.print(" Gy: "); Serial.print(Gy);
  Serial.print(" Gz: "); Serial.println(Gz);
*/

//calculating error
/*
  while (c<200){
    float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
AccErrorX = AccErrorX + ((atan((Ay) / sqrt(pow((Ax), 2) + pow((Az), 2))) * 180 / PI));
AccErrorY = AccErrorY + ((atan(-1 * (Ax) / sqrt(pow((Ay), 2) + pow((Az), 2))) * 180 / PI));
c++;
  }
      AccErrorX = AccErrorX / 200;
      AccErrorY = AccErrorY / 200;
      c = 0;
      // Read gyro values 200 times
      while (c < 200) {
        // Sum all readings
        GyroErrorX = GyroErrorX + (Gx);
        GyroErrorY = GyroErrorY + (Gy);
        GyroErrorZ = GyroErrorZ + (Gz);
        c++;
      }
      //Divide the sum by 200 to get the error value
      GyroErrorX = GyroErrorX / 200;
      GyroErrorY = GyroErrorY / 200;
      GyroErrorZ = GyroErrorZ / 200;
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
    
  */
  delay(100);
}
