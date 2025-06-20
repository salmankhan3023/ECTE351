#define DIR_PIN 9        // Direction control pin for TB6600
#define PUL_PIN 4       // Pulse (step) control pin for TB6600

const int STEP_DELAY = 300;    // Delay in microseconds between steps
bool motorRunning = false;
bool motorDirection = HIGH;    // HIGH = forward, LOW = backward

void setup() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PUL_PIN, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("Motor control via serial started...");
  Serial.println("Send 'j' to start forward, 'l' to start backward, 's' to stop");
}

void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    if (command == 'j') {
      Serial.println("Starting continuous forward motion");
      motorDirection = HIGH;
      digitalWrite(DIR_PIN, motorDirection);
      motorRunning = true;
    }
    else if (command == 'l') {
      Serial.println("Starting continuous backward motion");
      motorDirection = LOW;
      digitalWrite(DIR_PIN, motorDirection);
      motorRunning = true;
    }
    else if (command == 'k') {
      Serial.println("Stopping motor");
      motorRunning = false;
    }
  }
  
  // Keep motor running continuously if motorRunning is true
  if (motorRunning) {
    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(STEP_DELAY);
    digitalWrite(PUL_PIN, LOW);
    delayMicroseconds(STEP_DELAY);
  }
}
