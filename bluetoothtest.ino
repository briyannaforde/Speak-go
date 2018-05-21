#include <Servo.h>
#include <NewPing.h>

#define SONAR_NUM 5      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define baud 115200

// Declaring Sensor pins
  NewPing sonar[SONAR_NUM] = 
  {   
// Sensor object array.
// Each sensor's trigger pin, echo pin, and max distance to ping. 
    NewPing(22, 23, MAX_DISTANCE), 
    NewPing(24, 25, MAX_DISTANCE), 
    NewPing(26, 27, MAX_DISTANCE),
    NewPing(28, 29, MAX_DISTANCE),
    NewPing(30, 31, MAX_DISTANCE)
  };
  
// Analog IR prox sensors ADC channels  
  const int ProxSensorFL = A0;
  const int ProxSensorFR = A1;
  const int ProxSensorRL = A2;
  const int ProxSensorRR = A3;
  
// Data input for light control
  int inputValFL = 0;
  int inputValFR = 0;
  int inputValRL = 0;
  int inputValRR = 0;
  
  // long duration,cm, inches;
  int lightSense = A4;
  int ENA_PIN    = 33; //EnA of Buffer IC chip
  int Latch_Pin  = 32;
  
// Declaring the pins for the lights used
  int ReverseLights = 5;
  int DayLights = 6;
  int turnLeft = 7;
  int turnRight = 8;
  int HeadLights = 11;
  int BrakeLights = 9;
  int SensorLight = 10;

// Declaring window wiper motor
  #define ENB_PIN   4 // ENB of DC motor driver module attach to pin 4 of sunfounder uno board
  #define IN3 3 // left IN1 attach to pin 3 
  #define IN4 2 // left IN2 attach to pin 2
  
// Initializing Serial.available variable
  int incomingByte = 0;
  int incominglight = 0;

// Creating servo objects for the motors
  Servo servo; // Steering servo
  Servo steering; //Steering wheel servo
  Servo esc; // Brushless motor 
  #define default 90
  #define left 160
  #define right 50
  #define reverse 88
  #define forward 97

// Some statement that doesn't work.
  String startMsg = "Arduino board active";

void setup()
{
// Onboard lights and wiper
// Error Commands handling.
  pinMode(ReverseLights, OUTPUT);
  pinMode(turnRight, OUTPUT);   
  pinMode(turnLeft, OUTPUT);
  pinMode(DayLights, OUTPUT);   
  pinMode(HeadLights, OUTPUT);   
  pinMode(BrakeLights, OUTPUT);
  pinMode(SensorLight, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(Latch_Pin, OUTPUT); 

  servo.attach(34);                  // Attach servo objects to their respective pins
  steering.attach(13);               
  esc.attach(12);
  steering.write(default);                // Set their default positions
  servo.write(default);
  esc.write(default);
  delay(300);
  Serial1.begin(baud);               // Begin data communication process
}

void loop () 
{

 
if(Serial1.available()> 0) // Look for user data
  {
    incomingByte = Serial1.read();          // Store user data into variable
    Serial1.write(incomingByte);            // Display message to user
    Serial1.write(": ");
    switch( incomingByte)
    {

 // Car is turned on when a is entered     
      case 'a':
        StartIgnition();
      break;

// Turns on the head lights and state visability levels
      case 'b':
      Serial1.println("Low visablity");
      digitalWrite(HeadLights,HIGH);
      break;

  // Turns off the head lights and state visability levels
      case 'c':
      Serial1.println("High visablity");
      digitalWrite(HeadLights,LOW);
      break;

  // Turns off the brake lights, turns on the sensor light and starts the brushless motor forward
      case 'd':
        Serial1.println("Headlights are on");
        digitalWrite(BrakeLights,LOW);
        digitalWrite(SensorLight,HIGH);
        digitalWrite(ReverseLights,LOW);
        esc.write(forward);
      break;

// Turns off the brake lights and sensor lights and starts the brushless motor backward
      case 'e':
        Serial1.println("Headlights are off");
        digitalWrite(SensorLight,LOW);
        digitalWrite(BrakeLights,LOW);
        digitalWrite(ReverseLights,HIGH);
        esc.write(reverse);
        esc.write(reverse);
      break;

// Turns on the left indicator lights, turns the front wheels to the left and states direction of movement
      case 'f':
        Serial1.println("Turning left");
        digitalWrite(turnLeft,HIGH);
        delay(500); // wait for a second
        digitalWrite(turnLeft, LOW); 
        steering.write(left);
        servo.write(left);
        digitalWrite(turnLeft,HIGH);
        delay(500);                       // wait for a second
        digitalWrite(turnLeft, LOW);      // turn the LED off by making the voltage LOW
        delay(500);
        digitalWrite(turnLeft, HIGH);     // turn the LED on (HIGH is the voltage level)
        delay(500);
        digitalWrite(turnLeft, LOW);      
      break;

// Turns on the right indicator lights, turns the front wheels to the right and states direction of movement
      case 'g':     
        Serial1.println("Turning right");
        digitalWrite(turnRight,HIGH);  
        delay(500);                      // wait for half a second
        digitalWrite(turnRight, LOW);    // turn the LED off by making the voltage LOW
        steering.write(35);
        servo.write(right);
        digitalWrite(turnRight,HIGH);  
        delay(500);                      // wait for half a second
        digitalWrite(turnRight, LOW);    // turn the LED off by making the voltage LOW
        delay(500);
        digitalWrite(turnRight, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(500);
        digitalWrite(turnRight, LOW);
      break;
      
// Turns off the indicator lights, turns the front wheels straight and states direction of movement
      case 'h':     
        Serial1.println("Straightening Wheel");
        digitalWrite(turnRight, LOW);    // turn the LED off by making the voltage LOW
        digitalWrite(turnLeft, LOW);
        steering.write(default);
        servo.write(default);
        
      break;

// Turns off the sensor lights, turns on the brake lights and stops the cars movement while straightening the wheels
      case 'i':
        Serial1.println("Car is stopped");
        digitalWrite(SensorLight,LOW);
        digitalWrite(BrakeLights,HIGH);
        digitalWrite(ReverseLights,LOW);
        steering.write(default);
        servo.write(default);
        esc.write(default);
        delay(1000);
      break;

// Turns on windshield wipers and states the level of preciptation
      case 'j':
        Serial1.println("High precipitation");
        digitalWrite(ENB_PIN, HIGH); 
        digitalWrite(IN3,HIGH);
        digitalWrite(IN4,LOW);
        delay(500);
      break;

// Turns off windshield wipers and states the level of preciptation
      case 'l':
        Serial1.println("No precipitation");
        digitalWrite(ENB_PIN, LOW); 
        digitalWrite(IN3,LOW);
        digitalWrite(IN4,LOW);
        delay(500);
      break;

// Car becomes partially autonomous when l is entered         
      case 'm':
        Serial1.println("Car is now partially Autonomous");
        
      break;

// Car is turned off when k is entered      
      case 'k':
        ShutOffIgnition();
      break;
    }

  }
}
void StartIgnition()
{
  Serial1.println("Car ignition has been started");
  digitalWrite(ENA_PIN,LOW);
  digitalWrite(Latch_Pin, HIGH);
  // Set certain LEDs on
  digitalWrite(DayLights, HIGH);   // Turn the LED on (HIGH is the voltage level)
}
void ShutOffIgnition()
{
  Serial1.println("Car ignition has been turned off");
  // Set certain LEDs off
  digitalWrite(DayLights, LOW);   // Turn the LED on (HIGH is the voltage level)
  digitalWrite(SensorLight,LOW);
  digitalWrite(ENA_PIN,HIGH);
  digitalWrite(Latch_Pin, LOW);
}