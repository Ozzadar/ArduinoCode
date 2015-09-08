#include <LinkedList.h>



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
MPU6050 accelgyro;
//LED Blink State
bool blinkstate = false;

int ax, ay, az;
int gx, gy, gz;

int aax = 0;
int aay = 0;
int aaz = 0;
int agx = 0;
int agy = 0;
int agz = 0;

int numLoops = 1;

void setup() {
  
  #if IC2DEV_IMPLEMENTATION == IC2DEV_ARDDUINO_WIRE
    Wire.begin();
  #elif IC2DEV_IMPLEMENTATION == IC2DEV_BUILTIN_FASTWIRE
    Fastwire::setup(400,true);
  #endif
  
  Serial.begin(9600);
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  
    // use the code below to change accel/gyro offset values

  Serial.println("Updating internal sensor offsets...");

  //accelgyro.setXGyroOffset(-6289);
  //accelgyro.setYGyroOffset(4868);
  //accelgyro.setZGyroOffset(269);
  //accelgyro.setXAccelOffset(616);
  //accelgyro.setYAccelOffset(1634);
  //accelgyro.setZAccelOffset(1634);
  
  
  //verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed.");
  
  //configure Arduino LED for
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
    
    //read raw accel/gyro measurments from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (numLoops < 20) {
      aax = (aax + ax) / numLoops;
      aay = (aay + ay) / numLoops;
      aaz = (aaz + az) / numLoops;
      agx = (agx + gx) / numLoops;
      agy = (agy + gy) / numLoops;
      agz = (agz + gz) / numLoops;
      numLoops++;
    }
    //these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);
    
    
    #ifdef OUTPUT_READABLE_ACCELGYRO
      /*
      Serial.print("a/g:\t");
      Serial.print (ax); Serial.print("\t");
      Serial.print (ay); Serial.print("\t");
      Serial.print (az); Serial.print("\t");
      Serial.print (gx); Serial.print("\t");
      Serial.print (gy); Serial.print("\t");
      Serial.println(gz);
      */
      Serial.print ("Average ax:  \t"); Serial.print (aax); Serial.print("\t");
      Serial.print ("Average ay: \t"); Serial.print (aay); Serial.print("\t");
      Serial.print ("Average az: \t"); Serial.print (aaz); Serial.print("\t");
      Serial.print ("Average gx: \t"); Serial.print (agx); Serial.print("\t");
      Serial.print ("Average gy: \t"); Serial.print (agy); Serial.print("\t");
      Serial.print ("Average gz: \t"); Serial.print (agz); Serial.print("\t");
      Serial.print ("NumLoops: \t"); Serial.println(numLoops);
    
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
