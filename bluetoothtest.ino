// Author: Briyanna Forde
// Date: 05/28/2018

#include <Servo.h>
#include <NewPing.h>

#define SONAR_NUM 5      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define baud 115200
#define PAUSE 500 // 500 ms pause

// Declaring Sensor pins
NewPing sonar[SONAR_NUM] =
{ // Sensor object array.
  // Each sensor's pins trigger, echo, along with max distance.
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
int ENA_PIN    = 33; // EnA of Buffer 74LS573
int Latch_Pin  = 32; // Latch of Buffer 74LS573 ?

// Declaring the pins for the lights used
int ReverseLights = 5;
int DayLights = 6;
int turnLeft = 7;
int turnRight = 8;
int HeadLights = 11;
int BrakeLights = 9;
int SensorLight = 10;

// Declaring window wiper motor
#define ENB_PIN 4 // ENB for DC motor driver module
#define IN3 3 // left IN1 attach to pin 3 
#define IN4 2 // left IN2 attach to pin 2

// Initializing Serial1.available variable
char incomingByte = 0;
int incomingLight = 0;
int autoFlag = 0; // False - Car under commands

// Creating servo objects for the motors
Servo sservo; // Steering servo
Servo esc; // Brushless motor Ctrl-er

// Some statement that doesn't work.
String data = "Hello From Arduino!";
// Why not String startMsg = "Arduino board active"

// Commands function for vehicle
void carCommands(char iByte)
{
  switch (iByte)
  {
    // Turns ON the head LEDs and state visability levels
    case 'a':
      Serial1.println("Low visablity");
      digitalWrite(HeadLights, HIGH); // LEDs ON
      break;

    // Turns OFF the head lights and state visability levels
    case 'b':
      Serial1.println("High visablity");
      digitalWrite(HeadLights, LOW); // LEDs OFF
      break;

    // Turns OFF brake LEDs, go forward
    case 'c':
      Serial1.println("Headlights are on");
      Serial1.println("Sensorlight on");
      digitalWrite(SensorLight, HIGH); // Sensor ON
      digitalWrite(BrakeLights, LOW);      // Brake LEDs OFF
      digitalWrite(ReverseLights, LOW); // Reverse LEDs OFF
      esc.write(97); // Forward direction
      break;

    // Turn OFF break LEDs, Reverse LEDs ON, go backwards
    case 'd':
      Serial1.println("Headlights are off");
      digitalWrite(ReverseLights, HIGH); // Reverse LEDs ON
      digitalWrite(SensorLight, LOW); // Sensor OFF
      digitalWrite(BrakeLights, LOW); // Brake LEDs OFF
      esc.write(88); // Reverse direction
      break;

    // Turn left with indication
    case 'e':
      Serial1.println("Turning left");
      sservo.write(160); // Turn steering left
      // Flash the left turn LEDs
      for (int w = 0; w < 5; w++)
      {
        digitalWrite(turnLeft, HIGH); // LEDs ON
        delay(PAUSE); // Wait 500 ms
        digitalWrite(turnLeft, LOW); // LEDs OFF
        delay(PAUSE); // Wait 500 ms
      }
      break;

    // Turns right with indication
    case 'f':
      Serial1.println("Turning right");
      sservo.write(30); // Turn steering right
      // Flash the right turn LEDs
      for (int w = 0; w < 5; w++)
      {
        digitalWrite(turnRight, HIGH); // LEDs ON
        delay(PAUSE); // Wait 500 ms
        digitalWrite(turnRight, LOW); // LEDs OFF
        delay(PAUSE); // Wait 500 ms
      }
      break;

    // Turns brake LEDs ON, stop vehicle, steering straight
    case 'g':
      Serial1.println("Car is stopped");
      digitalWrite(BrakeLights, HIGH); // LEDs ON
      digitalWrite(SensorLight, LOW); // Sensor OFF
      digitalWrite(ReverseLights, LOW); // LEDs OFF
      sservo.write(90); // Steering straight
      esc.write(90); // Motor stopped
      delay((PAUSE * 2)); // Wait 1 sec
      break;

    // Wipers ON, preciptation level high
    case 'h':
      Serial1.println("High precipitation");
      digitalWrite(ENB_PIN, HIGH); // EN the h-bridge
      digitalWrite(IN3, HIGH); // Wipers ON
      digitalWrite(IN4, LOW); // Needs to be OFF for wipers
      delay(PAUSE); // Wait 500 ms
      break;

    // Wipers OFF, preciptation level low
    case 'i':
      Serial1.println("No precipitation");
      digitalWrite(IN3, LOW); // Wipers OFF
      digitalWrite(IN4, LOW); // Needs to be OFF for wipers
      digitalWrite(ENB_PIN, LOW); // EN for h-bridge OFF
      delay(PAUSE); // Wait 500 ms
      break;



    // Turn OFF the ignition and all other functioins
    case 'j':
      Serial1.println("Car is off");
      digitalWrite(DayLights, LOW); // Daytime running LEDs OFF
      digitalWrite(SensorLight, LOW); // Sensor OFF
      digitalWrite(ENA_PIN, HIGH); // ? OFF
      digitalWrite(Latch_Pin, LOW); // Enable buffer chip OFF
      break;

    // Turn ON the ignition for the car
    case 'k':
      Serial1.println("Car ignition is started");
      digitalWrite(Latch_Pin, HIGH); // ENable buffer chip
      digitalWrite(ENA_PIN, LOW); // ENable ?
      digitalWrite(DayLights, HIGH); // Day time running LEDs ON
      break;

    // Pre-determined sequence
    case 'l':
      Serial1.println("Car wheels are straighten");
      autoFlag = 1; // Autonomous mode enabled
      break;
    case 'm':
      Serial1.println("Car is now partially Autonomous");
      sservo.write(90);
      break;
    default: // Error Command Entered
      Serial1.println("\n Command Error!"); // Incorrect command
      sservo.write(90); // Default steering direction
      esc.write(90); // Turn OFF motor Ctrl-er
  }
  
}

// Function to automate the vehicle based on preset commands
void autoMate(char cmd, int amtOfCmds)
{
  int z = 0; // Loop variable

  // Cycle through and execute each command
  for (z = 0; z < amtOfCmds; z++)
  {
    // Execute the given command
    carCommands('k');
    delay(PAUSE);
    carCommands('a');
    delay(250);
    carCommands('e');
    delay(PAUSE);
    carCommands('c');
    delay(PAUSE);
    carCommands('f');
    carCommands('g');
    delay(PAUSE);
    carCommands('d');
    delay(PAUSE+PAUSE);
    carCommands('g');
    carCommands('j');
    delay(PAUSE);
  }
  // Reset the automate flag
  autoFlag = 0;
}

//################ Main loop ################
void setup()
{
  //Onboard lights and wiper
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
  pinMode(ENA_PIN, OUTPUT);
  pinMode(Latch_Pin, OUTPUT);

  // Flash certain LEDs - ON
  digitalWrite(DayLights, HIGH); // Daytime running lights
  digitalWrite(turnLeft, HIGH); // Left turn signal
  digitalWrite(turnRight, HIGH); // Right turn signal
  digitalWrite(SensorLight, HIGH);
  delay(PAUSE - 200); // 300 ms

  // Flash certain LEDs - OFF
  digitalWrite(DayLights, LOW);
  digitalWrite(turnLeft, LOW);
  digitalWrite(turnRight, LOW);
  digitalWrite(SensorLight, LOW);
  delay(PAUSE - 200); // 300 ms

  sservo.attach(13); // Attach sservo objects pin #13
  esc.attach(12); // Attach esc objects to pin #12
  sservo.write(90);  // Set sservo to default position
  esc.write(90); // Set esc to default - OFF
  delay(PAUSE - 200); // 300 ms
  Serial1.begin(baud); // Begin data communication process
}

// ############## Main Loop for Code Execution ##############
void loop ()
{
  // Array of commands to automate the vehicle
  char cmds[] = {'k', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'l', 'm'};
  int  numOfCmds = 13; // Hard coded, can be larger

  // Data array for ping sensors
  int pSensor[SONAR_NUM]; // 5 sensors total

  // Read each PING sensor and save its value
  for (uint8_t i = 0; i < SONAR_NUM; i++)
  { 
    // Send display message to user
    Serial1.print("Ping Sensor ");
    Serial1.print(i);
    Serial1.print("= ");
    pSensor[i] = sonar[i].ping_cm(); // Obtain the ping sensor's value
    Serial1.print(pSensor[i]); // Send distance to COMs
    Serial1.print("cm \n");
    delay((PAUSE/10));// Wait 50 ms
  }

  // Obtain the dat from the prox Sensors
  inputValFL = analogRead(ProxSensorFL);
  inputValFR = analogRead(ProxSensorFR);
  inputValRL = analogRead(ProxSensorRL);
  inputValRR = analogRead(ProxSensorRR);
  incomingLight = analogRead(lightSense);

  // Send display msg to user on COMs
  Serial1.print("Distance 1 is: ");
  Serial1.println(inputValFL);
  
  Serial1.print("Distance 2 is: ");
  Serial1.println(inputValFR);
  
  Serial1.print("Distance 3 is: ");
  Serial1.println(inputValRL);
  
  Serial1.print("Distance 4 is: ");
  Serial1.println(inputValRR);
  
  Serial1.println(" ");
  delay(PAUSE); // Wait 500 ms




  carCommands('k') ; // Turn on the car's ignition.




  // ######## Make decision based on the read values
  
  if((incomingLight < 500))
  {
    carCommands('a');
  }
  if((incomingLight > 500))
  {
    carCommands('b');
  }
  if((inputValFR > 300) & (inputValFR < 800) & (inputValFL > 300) & (inputValFL < 800)) // prox sensor FR is within
  {
    carCommands('d'); // Go backward
  }

  if((inputValRL > 300) & (inputValRL < 800) & (inputValRR > 300) & (inputValRR < 800)) // prox sensor RL is within
  {
    carCommands('c'); // Go forward
  }


  // ######## Make decision based on sensor data - pSensor[]
//  if(pSensor[0] > 200) carCommands('g'); // You can use this example - PING sensor 1
//
//  if((pSensor[0] > 100) & (pSensor[0] < 400)){
//    carCommands('g'); // You can use this example - PING sensor 1
//  }// End of checking pSensor[0], PING sensor #1
  
  // ######## Make decision based on sensor data - pSensor[0] and Prox Sensor FL
  if((inputValFL > 100) & (pSensor[0] < 30) || (pSensor[1] < 30) & (pSensor[2] > 30) & (pSensor[3] > 30))
  {
    carCommands('e'); // Turn left
  }

  if((inputValFL > 100) & (pSensor[2] < 30) || (pSensor[3] < 30) & (pSensor[0] > 30) & (pSensor[1] > 30))
  {
    carCommands('f'); // Turn right
  }
  if((inputValFL > 100) & (pSensor[2] > 30) & (pSensor[3] > 30) & (pSensor[0] > 30) & (pSensor[1] > 30))
  {
    carCommands('m'); // Car is straight
  }


  //carCommands('j'); // Turn OFF the car's ignition


  
  //######## Checks msg from Bluetooth and execute that command
  // Check for data via Bluetooth module
  if (Serial1.available() > 0) // Look for user data
  {
    incomingByte = Serial1.read(); // Store user char Data into variable
    Serial1.write(incomingByte);   // Display message to user
    Serial1.write(": ");

    // Operate the vehicle based on sent data
    carCommands(incomingByte);

    // Check if autonomous flag is set
    if (autoFlag == 1)
    {
      // Execute the automated commands
      autoMate(cmds, numOfCmds);
    }
  } // End of data checking
}
