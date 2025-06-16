// Pin definitions for motor control
const int motorLeftPin = 7;
const int motorRightPin = 8;
const int motorEnablePin = 9;

// Pin definitions for IR sensors
const int irSensor1Pin = 2;
const int irSensor2Pin = 3;
// Add more IR sensor pins as needed

void setup() {
  Serial.begin(9600);
  
  // Motor control pins
  pinMode(motorLeftPin, OUTPUT);
  pinMode(motorRightPin, OUTPUT);
  pinMode(motorEnablePin, OUTPUT);
  
  // IR sensor pins
  pinMode(irSensor1Pin, INPUT);
  pinMode(irSensor2Pin, INPUT);
  
  // Initialize motor as stopped
  digitalWrite(motorLeftPin, LOW);
  digitalWrite(motorRightPin, LOW);
  digitalWrite(motorEnablePin, LOW);
}

void loop() {
  // Check for serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("MOTOR:")) {
      String direction = command.substring(6);
      controlMotor(direction);
    }
  }
  
  // Check IR sensors
  checkIRSensors();
  
  delay(50);
}

void controlMotor(String direction) {
  if (direction == "LEFT") {
    digitalWrite(motorLeftPin, HIGH);
    digitalWrite(motorRightPin, LOW);
    digitalWrite(motorEnablePin, HIGH);
  } else if (direction == "RIGHT") {
    digitalWrite(motorLeftPin, LOW);
    digitalWrite(motorRightPin, HIGH);
    digitalWrite(motorEnablePin, HIGH);
  } else if (direction == "STOP") {
    digitalWrite(motorLeftPin, LOW);
    digitalWrite(motorRightPin, LOW);
    digitalWrite(motorEnablePin, LOW);
  }
}

void checkIRSensors() {
  bool sensor1 = digitalRead(irSensor1Pin);
  bool sensor2 = digitalRead(irSensor2Pin);
  
  // Send sensor status (LOW = object detected for most IR sensors)
  if (!sensor1 || !sensor2) {
    Serial.println("IR:BLOCKED");
    // Emergency stop motor
    digitalWrite(motorLeftPin, LOW);
    digitalWrite(motorRightPin, LOW);
    digitalWrite(motorEnablePin, LOW);
  } else {
    Serial.println("IR:CLEAR");
  }
}
