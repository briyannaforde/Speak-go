#include <Servo.h>
#include <NewPing.h>

#define SONAR_NUM 5      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define baud 115200

// Declaring Sensor pins
  NewPing sonar[SONAR_NUM] = 
  {   // Sensor object array.
    NewPing(22, 23, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
    NewPing(24, 25, MAX_DISTANCE), 
    NewPing(26, 27, MAX_DISTANCE),
    NewPing(28, 29, MAX_DISTANCE),
    NewPing(30, 31, MAX_DISTANCE)
  };
  const int ProxSensorFL = A0;
  const int ProxSensorFR = A1;
  const int ProxSensorRL = A2;
  const int ProxSensorRR = A3;
  int inputValFL = 0;
  int inputValFR = 0;
  int inputValRL = 0;
  int inputValRR = 0;
  // long duration,cm, inches;
  int lightSense = A4;

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
#define ENA_PIN   33 //
// Initializing Serial.available variable
  int incomingByte = 0;

// Creating servo objects for the motors
  Servo servo; // Steering servo
  Servo esc; // Brushless motor 


// Some statement that doesn't work.
  String data="Hello From Arduino!";

void setup()
{

// Error Commands handiling.
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
 
// Flash certain LEDs on and off
  digitalWrite(DayLights, HIGH);   // Turn the LED on (HIGH is the voltage level)
  digitalWrite(turnLeft,HIGH);
  digitalWrite(SensorLight,HIGH);
  digitalWrite(turnRight, HIGH);   
  delay(300);
  digitalWrite(turnLeft, LOW);    // Turn the LED off by making the voltage LOW
  digitalWrite(turnRight, LOW);
  digitalWrite(SensorLight,LOW);
  digitalWrite(DayLights, LOW);    
  delay(300);
  servo.attach(13);               // Attach servo objects to their respective pins
  esc.attach(12);
  servo.write(90);                // Set their default positions
  esc.write(90);
  delay(300);
  Serial1.begin(baud);          // Begin data communication process
}

void loop () 
{
 /* 
    for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
    delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    Serial.print("Ping Sensor ");
    Serial.print(i);
    Serial.print("= ");
    Serial.print(sonar[i].ping_cm());
    Serial.print("cm \n");
  }

  inputValFL = analogRead(ProxSensorFL);
  Serial.print("Distance1 is: ");
  Serial.println(inputValFL);
  inputValFR = analogRead(ProxSensorFR);
  Serial.print("Distance2 is: ");
  Serial.println(inputValFR);
  inputValRL = analogRead(ProxSensorRL);
  Serial.print("Distance3 is: ");
  Serial.println(inputValRL);
  inputValRR = analogRead(ProxSensorRR);
  Serial.print("Distance4 is: ");
  Serial.println(inputValRR);
  Serial.println(" ");
  delay(1500);              // Wait for a second
  */
/*  Create a section of code for ignition
    This will enble the buffer enable pin to 'start' the
    car and turn on the day lights.
    In addition a code created to turn off the enable is
    also necessary to ensure that the lights are off
    when the care is off.
*/
 
if(Serial1.available()>0) // Look for user data
  {
    incomingByte = Serial1.read();          // Store user data into variable
    Serial1.write(incomingByte);            // Display message to user
    Serial1.write(": ");
    switch( incomingByte)
    {

// Turns on the head lights and state visability levels
    case '1':
     Serial1.println("Low visablity");
     digitalWrite(HeadLights,HIGH);
    break;
// Turns off the head lights and state visability levels
    case '2':
     Serial1.println("High visablity");
     digitalWrite(HeadLights,LOW);
    break;
// Turns off the brake lights, turns on the sensor light and starts the brushless motor forward
    case '3':
      Serial1.println("Headlights are on");
      digitalWrite(BrakeLights,LOW);
      digitalWrite(SensorLight,HIGH);
      digitalWrite(ReverseLights,LOW);
      esc.write(97);
    break;
// Turns off the brake lights and sensor lights and starts the brushless motor backward
    case '4':
      Serial1.println("Headlights are off");
      digitalWrite(SensorLight,LOW);
      digitalWrite(BrakeLights,LOW);
      digitalWrite(ReverseLights,HIGH);
      esc.write(88);
      esc.write(88);
    break;
// Turns on the left indicator lights, turns the front wheels to the left and states direction of movement
    case '5':
      Serial1.println("Turning left");
      digitalWrite(turnLeft,HIGH);
      delay(500); // wait for a second
      digitalWrite(turnLeft, LOW); 
      servo.write(160);
      digitalWrite(turnLeft,HIGH);
      delay(500);                       // wait for a second
      digitalWrite(turnLeft, LOW);    // turn the LED off by making the voltage LOW
      delay(500);
      digitalWrite(turnLeft, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(500);
      digitalWrite(turnLeft, LOW);      
    break;
// Turns on the right indicator lights, turns the front wheels to the right and states direction of movement
    case '6':     
      Serial1.println("Turning right");
      digitalWrite(turnRight,HIGH);  
      delay(500);       // wait for a second
      digitalWrite(turnRight, LOW);    // turn the LED off by making the voltage LOW
      servo.write(30);
      digitalWrite(turnRight,HIGH);  
      delay(500);       // wait for a second
      digitalWrite(turnRight, LOW);    // turn the LED off by making the voltage LOW
      delay(500);
      digitalWrite(turnRight, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(500);
      digitalWrite(turnRight, LOW);
    break;
// Turns off the sensor lights, turns on the brake lights and stops the cars movement while straightening the wheels
    case '7':
      Serial1.println("Car is stopped");
      digitalWrite(SensorLight,LOW);
      digitalWrite(BrakeLights,HIGH);
      digitalWrite(ReverseLights,LOW);
      servo.write(90);
      esc.write(90);
      delay(1000);
    break;
// Turns on windshield wipers and states the level of preciptation
    case '8':
      Serial1.println("High precipitation");
      digitalWrite(ENB_PIN, HIGH); 
      digitalWrite(IN3,HIGH);
      digitalWrite(IN4,LOW);
      delay(500);
    break;
// Turns off windshield wipers and states the level of preciptation
    case '9':
      Serial1.println("No precipitation");
      digitalWrite(ENB_PIN, LOW); 
      digitalWrite(IN3,LOW);
      digitalWrite(IN4,LOW);
      delay(500);
    break;
    
    case 'A':
      Serial1.println("Car is now partially Autonomous");
      
    break;
    
   case 'B':
     Serial1.println("Car is off");
     digitalWrite(ENA_PIN,HIGH);
     // Set certain LEDs off
     digitalWrite(DayLights, LOW);   // Turn the LED on (HIGH is the voltage level)
     digitalWrite(SensorLight,LOW);
   break;
   
   case 'C':
     Serial1.println("Car is on");
     digitalWrite(ENA_PIN,LOW);
      // Set certain LEDs on
     digitalWrite(DayLights, HIGH);   // Turn the LED on (HIGH is the voltage level)
     digitalWrite(SensorLight,HIGH);
   break;
  }
//  if (inches <= 10)
//  {
//    esc.write(90);
//    digitalWrite(BrakeLights, HIGH);
  }
}
