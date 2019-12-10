#include <Servo.h> //Include Servo and Wire libraries for communicating with accelerometer
#include <Wire.h>  // and servo motors.


float currentTime, passedTime, previousTime;
float xAngle, yAngle, zAngle;
float yaw;
float xGyAngle, yGyAngle;
float xGyError, yGyError, zGyError;
int c;
int yawServoValue,pitchServoValue,rollServoValue;

Servo yawServo, pitchServo, rollServo;

void setup() {
  Serial.begin(9600);  //Setting up communication with the accelerometer
  Wire.begin();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x6B);
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B);       // Configure the Gyroscope
  Wire.write(0b00000000);
  Wire.endTransmission();

  c = 0;

  while (c < 500) {  

    Wire.beginTransmission(0b1101000);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0b1101000,6,true);
    
    xAngle = (Wire.read() << 8 | Wire.read())/131;
    yAngle = (Wire.read() << 8 | Wire.read())/131;
    zAngle = (Wire.read() << 8 | Wire.read())/131;  

    xGyError = xGyError + xAngle;
    yGyError = yGyError + yAngle;
    zGyError = zGyError + zAngle;
    
    c++;  
  }

  xGyError = xGyError/500;
  yGyError = yGyError/500;
  zGyError = zGyError/500;
  
  yawServo.attach(10);
  pitchServo.attach(8);
  rollServo.attach(6);

  yawServo.write(0);

}

void loop() {

    // Get gyro data and calculate gyro angles

    previousTime = currentTime;                      // Gyro data is outputted in degrees/second so we must
    currentTime = millis();                          // multiply by seconds to get degrees.
    passedTime = (currentTime-previousTime) / 1000;
    
    Wire.beginTransmission(0b1101000); //I2C Communication with gyro to extract data
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0b1101000,6,true);
    
    xAngle = (Wire.read() << 8 | Wire.read())/131;
    yAngle = (Wire.read() << 8 | Wire.read())/131;
    zAngle = (Wire.read() << 8 | Wire.read())/131;

    xAngle = xAngle - xGyError;  //Accounting for error calculated in the setup
    yAngle = yAngle - yGyError;
    zAngle = zAngle - zGyError;

    xGyAngle = xGyAngle + xAngle * passedTime; //Converting from deg/s to deg;
    yGyAngle = yGyAngle + yAngle * passedTime;
    yaw = yaw + zAngle * passedTime;

    rollServoValue = map(yGyAngle,-90,90,0,180);   //Mapping gyro data to a range of 0-180 degrees which
    pitchServoValue = map(xGyAngle,-90,90,0,180);  //is more desirable for the servo motors
    yawServoValue = map(yaw,-90,90,180,0);

    rollServo.write(rollServoValue);   //Writing angle values to gyro
    pitchServo.write(pitchServoValue);
    yawServo.write(yawServoValue);
   
   
//    Serial.print("xAcc ");
//    Serial.print(xAcc);
//    Serial.print(" yAcc ");
//    Serial.print(yAcc);
//    Serial.print(" zAcc ");
//    Serial.print(zAcc);
//    Serial.print("   X Error ");
//    Serial.print(xError);
//    Serial.print(" Y Error ");
//    Serial.print(yError);
//    Serial.print(" Z Error ");
//    Serial.println(zError);
    
}
