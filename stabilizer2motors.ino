
// libraries 
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
 

 // initializing the values
 Servo stabilizerServo1; // for first motor,  parallax
 Servo stabilizerServo2; // for another motor, sg90
Adafruit_MPU6050 mpu;
float pitch_gyro = 0;
float roll_gyro = 0;
float filtered_pitch = 0;
float filtered_roll = 0;
unsigned long previousTime = 0;

const float SHAKE_THRESHOLD = 2.0;   // G-force (acceleration)
const float TILT_THRESHOLD = 15.0;   // Degrees per second (gyro)



bool debugMode = false;
// Part 1.A
void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for Serial Monitor (optional)
  Wire.begin();

  // Run a quick test to understand if MPU is detected
  // otherwise, program will break
  if (!mpu.begin(0x68)) {  // Confirmed I2C address
    Serial.println("MPU6050 not found!");
    while (1);
  }

    stabilizerServo1.attach(9); // D9
    stabilizerServo2.attach(10); // D10

  //set up your MPU to record values
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // acceleration sensor measure
  mpu.setGyroRange(MPU6050_RANGE_500_DEG); // sets the rotation speed range in degrees per second
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // helps to get smoother data

  previousTime = millis();
 
  
  delay(100);
}



// recording values
  void loop() {
  // getting the data from sensor
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //test
  if (abs(a.acceleration.x) > SHAKE_THRESHOLD || abs(a.acceleration.y) > SHAKE_THRESHOLD || abs(a.acceleration.z - 9.8) > SHAKE_THRESHOLD) {
   //Serial.println("Shake detected!");
  }
  if (abs(g.gyro.x) > TILT_THRESHOLD || abs(g.gyro.y) > TILT_THRESHOLD || abs(g.gyro.z) > TILT_THRESHOLD) {
    //Serial.println("Tilt detected!");
  }


  // calculating the tilt/pitch
  float pitch_acc = 180 * atan2(a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z))/PI;
  float roll_acc  = 180 * atan2(-a.acceleration.x, a.acceleration.z) / PI;

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;
  
  
  float alpha = 0.96;
 // filtered_pitch = alpha * (filtered_pitch + g.gyro.y * deltaTime * 180 / PI) + (1 - alpha) * pitch_acc;
  //filtered_roll  = alpha * (filtered_roll  + g.gyro.x * deltaTime * 180 / PI) + (1 - alpha) * roll_acc;

    filtered_roll  = alpha * (filtered_roll  + g.gyro.x * deltaTime * 180 / PI) + (1 - alpha) * roll_acc;
    filtered_pitch = alpha * (filtered_pitch + g.gyro.y * deltaTime * 180 / PI) + (1 - alpha) * pitch_acc;

  // Clamp filtered_roll to reasonable range (-45° to +45°)
  //float limitedRoll = constrain(filtered_roll, -45.0, 45.0);
  float limitedRoll = filtered_roll; // To see raw range
  float limitedPitch = filtered_pitch; 
  // Map -45 to +45 degrees roll → 0 to 180 servo angle
  int servoAngle = map(limitedRoll, -45.0, 45.0, 0, 180);
  int servoPitch = map(limitedPitch,  -45.0, 45.0, 0, 180);

  // Send to servo
  stabilizerServo1.write(servoAngle); // parallax motor
  stabilizerServo2.write(servoPitch); // sg90


  if (debugMode){
  // linear acceperation
  Serial.print(" Xacc: "); Serial.print(a.acceleration.x); Serial.print("\t");
  Serial.print(" Yacc: "); Serial.print(a.acceleration.y); Serial.print("\t");
  Serial.print(" Zacc: "); Serial.print(a.acceleration.z); Serial.print("\t");


  // angualar velocity. tells us how fast youre tilting or spinninng 
  Serial.print(" Xrotsp: ");    Serial.print(g.gyro.x);         Serial.print("\t");
  Serial.print(" Yrotsp: ");    Serial.print(g.gyro.y);         Serial.print("\t");
  Serial.print(" Zrotsp:");     Serial.println(g.gyro.z);       Serial.print("\t");


  // position
  // In order to save some space in the output message. I will short the <Filtered pitch> to FP
  // Filtered roll will be FR
  Serial.print ("FP: "); Serial.print(filtered_pitch); Serial.print("\t");
  Serial.print ("FR: "); Serial.println(filtered_roll);

  // temperature
  Serial.print(" Temp: "); Serial.println(temp.temperature);

  } else {
   
    Serial.print("Filtered Pitch: "); Serial.print(filtered_pitch); Serial.print("\t");
    Serial.print("Filtered Roll: "); Serial.print(filtered_roll); Serial.print("\t");
    Serial.print("Temp: "); Serial.println(temp.temperature);
    
    Serial.print("Servo1 (Roll): "); Serial.print(servoAngle);
    Serial.print("\tServo2 (Pitch): "); Serial.println(servoPitch);
  }



  
  
  


  delay(20); // ~20 Hz sampling rate
}
