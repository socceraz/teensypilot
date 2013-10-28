#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
#include "kalman.h"
#include <Servo.h>
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"


#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

#define AILERON_PIN (2)
#define ELEVATOR_PIN (3) 

#define LEFT_SERVO (23)
#define RIGHT_SERVO (22)
#define BACK_SERVO (21)
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

double gx_scaled,gy_scaled,gz_scaled;
double ax_squared,ay_squared,az_squared;

double time1, time2,deltaTime;

double pitch, roll;
double pitchFiltered, rollFiltered;

double elevTime[2];
double aileTime[2];

double controller, controller_last_run;

double servoPosition[3];

Kalman pitchKalmanFilter;
Kalman rollKalmanFilter;

Servo leftAileron;
Servo rightAileron;
Servo elevator;


// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define DEBUG_IMU_SERIAL

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO


#define LED_PIN 13
bool blinkState = false;
int PinState;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    delay(1000);
    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    delay(2000);
    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);

    pinMode(AILERON_PIN, INPUT);
    pinMode(ELEVATOR_PIN, INPUT);

    attachInterrupt(AILERON_PIN,aileInt,CHANGE);
    attachInterrupt(ELEVATOR_PIN,elevInt,CHANGE);

    leftAileron.attach(LEFT_SERVO);
    rightAileron.attach(RIGHT_SERVO);
    elevator.attach(BACK_SERVO);

    for (int x=75;x<100;x++)
    {
        leftAileron.write(x);
        rightAileron.write(x);
        elevator.write(x);
        delay(20);
    }
    for (int x=100;x>75;x--)
    {
        leftAileron.write(x);
        rightAileron.write(x);
        elevator.write(x);
        delay(20);
    controller_last_run = micros();
    }
}

void loop() {

    //Need to create loop for time keeping purpose
    controller = micros();
    if ((controller-controller_last_run)>20000)
    {
        controller_last_run=micros();
        
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        ax_squared = ax*ax;
        ay_squared = ay*ay;
        az_squared = az*az;

        gx_scaled = gx/131.0;
        gy_scaled = gy/131.0;
        gz_scaled = gz/131.0;

        roll = atan2( (double)(ax),sqrt(ay_squared+az_squared))*180/3.141;
        pitch = atan2( (double)(ay), sqrt(ax_squared+az_squared))*180/3.141;

        time1 = micros();
        
        deltaTime = (time1-time2)/1000000.0;
        Serial.println(1/deltaTime);
        
        rollFiltered = rollKalmanFilter.getAngle(roll,gy_scaled,deltaTime);
        pitchFiltered = pitchKalmanFilter.getAngle(pitch,gx_scaled,deltaTime);

        time2 = time1;


        // Serial.print("Roll:\t");
        // Serial.print(roll);Serial.print("\t");
        // Serial.print(rollFiltered); Serial.println("\t");
       
        //Serial.print("Pitch\t");
        //Serial.println(pitchFiltered);
        Serial.println(1/deltaTime);

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
        

        servoPosition[0] = map(aileTime[1], 1000, 2000, 75000, 100000); //Left Aileron
        servoPosition[1] = map(aileTime[1], 1000, 2000, 75000, 100000); //Right Aileron
        servoPosition[2] = map(elevTime[1], 1000, 2000, 50000, 170000); //Elevator

        //implement a simple feed back loop
        servoPosition[0] += rollFiltered*100; 
        servoPosition[1] += rollFiltered*100;
        //servoPosition[2] += pitchFiltered*500; 



        leftAileron.write(servoPosition[0]/1000.0);
        rightAileron.write(servoPosition[1]/1000.0);
        elevator.write(servoPosition[2]/1000.0);
    }
}

void aileInt()
{
    //Serial.println("Aileron");
    PinState = digitalRead(AILERON_PIN); 
    //Serial.println(PinState);
    if (PinState == 1)
    {
        aileTime[0]=micros();
    }
    else
    {
        aileTime[1]=micros()-aileTime[0];
        //Serial.println(aileTime[1]);
    }
}

void elevInt()
{
    
    PinState = digitalRead(ELEVATOR_PIN); 
    //Serial.println(PinState);
    if (PinState == 1)
    {
        elevTime[0]=micros();
    }
    else
    {
        elevTime[1]=micros()-elevTime[0];
        //Serial.println(elevTime[1]);
    }
    //PinState = digitalRead(ELEVATOR_PIN);
}
