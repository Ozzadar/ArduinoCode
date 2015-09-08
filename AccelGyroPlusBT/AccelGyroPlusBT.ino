#include <LinkedList.h>

//For BT HC-05
#include <SoftwareSerial.h>

//For MPU6050
#include <I2Cdev.h>
#include <AccelGyro.h>
#include <Motor.h>
#include <BoolPin.h>
#include <Scheduler.h>
#include <Integrator.h>

#include <helper_3dmath.h>




/* ARDUINO AND MPU6050 6 AXIS MOTION SENSOR */







//Arduino Wire library is required if 
#if IC2DEV_IMPLEMENTATION == IC2DEV_ARDUINO_WIRE
    #include <Wire.h>
#endif

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

#define LED_PIN 13



//Devices and variables

//MPU6050
AccelGyro accelgyro;
SoftwareSerial mySerial(0,1);

//LED Blink State
bool blinkstate = false;

double ax, ay, az;
double gx, gy, gz;

int aax = 0;
int aay = 0;
int aaz = 0;
int agx = 0;
int agy = 0;
int agz = 0;

int numLoops = 1;

void setup() {
  
  Serial.begin(9600);
  
  while (!Serial) {
      ;//wait for serial port to connect
  }  
  
  
  accelgyro.setup();
  accelgyro.calibrate();
  Serial.println("Setup Complete");
  mySerial.begin(9600);
  mySerial.println("BT activated");
  
  //verify connection
  

  //configure Arduino LED for
   
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  
    digitalWrite(13, LOW);
    // Keep reading from HC-05 and send to Arduino Serial Monitor

    if (mySerial.available())
      Serial.write(mySerial.read());
    if (Serial.available())
      mySerial.write(Serial.read());
      
    accelgyro.compute();
    //read raw accel/gyro measurments from device

    accelgyro.getMotion6Normalized(&ax, &ay, &az, &gx, &gy, &gz);

    
    
    #ifdef OUTPUT_READABLE_ACCELGYRO
 
      Serial.print (ax); Serial.print(",");
      Serial.print (ay); Serial.print(",");
      Serial.print (az); Serial.print(",");
      Serial.print (gx); Serial.print(",");
      Serial.print (gy); Serial.print(",");
      Serial.println(gz);

    
    #endif
    
    
    #ifdef OUTPUT_BINARY_ACCELGRYRO
    
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif  
    
    blinkstate = !blinkstate;
    
    digitalWrite(LED_PIN, blinkstate);
}
