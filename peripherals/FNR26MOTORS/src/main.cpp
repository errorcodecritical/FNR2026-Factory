#include <Arduino.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// H-Bridge 1 - Motors 1 & 2 (direction only)
#define MOTOR1_IN1  2
#define MOTOR1_IN2  3
#define MOTOR2_IN1  4
#define MOTOR2_IN2  5

// H-Bridge 2 - Motors 3 & 4 (direction only)
#define MOTOR3_IN1  6
#define MOTOR3_IN2  7
#define MOTOR4_IN1  8
#define MOTOR4_IN2  9

// Encoders (moved to 18-25 to free up 10-13 for PWM)
#define MOTOR1_ENC_A  11
#define MOTOR1_ENC_B  10
#define MOTOR2_ENC_A  12
#define MOTOR2_ENC_B  13
#define MOTOR3_ENC_A  15
#define MOTOR3_ENC_B  14
#define MOTOR4_ENC_A  16
#define MOTOR4_ENC_B  17

// ============================================================================
// MOTOR CONTROL STRUCTURE
// ============================================================================

struct Motor {
  uint8_t in1_pin;
  uint8_t in2_pin;
  uint8_t enc_a_pin;
  uint8_t enc_b_pin;
  volatile long encoder_count;
  volatile int8_t last_encoded;
  int16_t speed;  // -255 to 255 (negative = reverse)
};

// Create motor instances
Motor motors[4] = {
  {MOTOR1_IN1, MOTOR1_IN2, MOTOR1_ENC_A, MOTOR1_ENC_B, 0, 0, 0},
  {MOTOR2_IN1, MOTOR2_IN2, MOTOR2_ENC_A, MOTOR2_ENC_B, 0, 0, 0},
  {MOTOR3_IN1, MOTOR3_IN2, MOTOR3_ENC_A, MOTOR3_ENC_B, 0, 0, 0},
  {MOTOR4_IN1, MOTOR4_IN2, MOTOR4_ENC_A, MOTOR4_ENC_B, 0, 0, 0}
};

// ============================================================================
// ENCODER INTERRUPT HANDLERS
// ============================================================================

void updateEncoder(uint8_t motor_id) {
  Motor* motor = &motors[motor_id];
  
  int MSB = digitalRead(motor->enc_a_pin);
  int LSB = digitalRead(motor->enc_b_pin);
  
  int encoded = (MSB << 1) | LSB;
  int sum = (motor->last_encoded << 2) | encoded;
  
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    motor->encoder_count++;
  }
  else if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    motor->encoder_count--;
  }
  
  motor->last_encoded = encoded;
}

void encoderISR_Motor1() { updateEncoder(0); }
void encoderISR_Motor2() { updateEncoder(1); }
void encoderISR_Motor3() { updateEncoder(2); }
void encoderISR_Motor4() { updateEncoder(3); }

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

void setMotorSpeed(uint8_t motor_id, int16_t speed) {
  if (motor_id >= 4) return;
  
  Motor* motor = &motors[motor_id];
  motor->speed = constrain(speed, -255, 255);
  
  if (speed > 0) {
    // Forward: IN1=HIGH, IN2=LOW, PWM = speed
    analogWrite(motor->in1_pin, speed);
    digitalWrite(motor->in2_pin, LOW);
  } 
  else if (speed < 0) {
    // Reverse: IN1=LOW, IN2=HIGH, PWM = |speed|
    digitalWrite(motor->in1_pin, LOW);
    analogWrite(motor->in2_pin, -speed);
  } 
  else {
    // Stop (brake): IN1=LOW, IN2=LOW, PWM = 0
    digitalWrite(motor->in1_pin, LOW);
    digitalWrite(motor->in2_pin, LOW);
  }
}

void stopAllMotors() {
  for (uint8_t i = 0; i < 4; i++) {
    setMotorSpeed(i, 0);
  }
}

long getEncoderCount(uint8_t motor_id) {
  if (motor_id >= 4) return 0;
  noInterrupts();
  long count = motors[motor_id].encoder_count;
  interrupts();
  return count;
}

void resetEncoder(uint8_t motor_id) {
  if (motor_id >= 4) return;
  noInterrupts();
  motors[motor_id].encoder_count = 0;
  interrupts();
}

void resetAllEncoders() {
  for (uint8_t i = 0; i < 4; i++) {
    resetEncoder(i);
  }
}

// ============================================================================
// SERIAL COMMUNICATION PROTOCOL
// ============================================================================

/*
Command Protocol (ASCII):
- Set motor speed: "M<id> <speed>\n"   (id: 0-3, speed: -255 to 255)
- Get encoder:     "E<id>\n"            (id: 0-3)
- Get all encoders: "EA\n"
- Reset encoder:   "R<id>\n"            (id: 0-3)
- Reset all encoders: "RA\n"
- Stop all motors: "S\n"
- Get status:      "?\n"
*/

String inputBuffer = "";

void processCommand(String command) {
  command.trim();
  if (command.length() == 0) return;
  
  char cmd = command.charAt(0);
  
  switch (cmd) {
    case 'M': {
      int spaceIndex = command.indexOf(' ');
      if (spaceIndex == -1) {
        Serial.println("ERROR: Invalid motor command format");
        return;
      }
      uint8_t motor_id = command.substring(1, spaceIndex).toInt();
      int16_t speed = command.substring(spaceIndex + 1).toInt();
      if (motor_id >= 4) {
        Serial.println("ERROR: Motor ID must be 0-3");
        return;
      }
      setMotorSpeed(motor_id, speed);
      Serial.print("OK: Motor ");
      Serial.print(motor_id);
      Serial.print(" set to ");
      Serial.println(speed);
      break;
    }
    
    case 'E': {
      if (command.length() == 2 && command.charAt(1) == 'A') {
        Serial.print("ENCODERS:");
        for (uint8_t i = 0; i < 4; i++) {
          Serial.print(" ");
          Serial.print(getEncoderCount(i));
        }
        Serial.println();
      } else {
        uint8_t motor_id = command.substring(1).toInt();
        if (motor_id >= 4) {
          Serial.println("ERROR: Motor ID must be 0-3");
          return;
        }
        Serial.print("ENCODER");
        Serial.print(motor_id);
        Serial.print(": ");
        Serial.println(getEncoderCount(motor_id));
      }
      break;
    }
    
    case 'R': {
      if (command.length() == 2 && command.charAt(1) == 'A') {
        resetAllEncoders();
        Serial.println("OK: All encoders reset");
      } else {
        uint8_t motor_id = command.substring(1).toInt();
        if (motor_id >= 4) {
          Serial.println("ERROR: Motor ID must be 0-3");
          return;
        }
        resetEncoder(motor_id);
        Serial.print("OK: Encoder ");
        Serial.print(motor_id);
        Serial.println(" reset");
      }
      break;
    }
    
    case 'S': {
      stopAllMotors();
      Serial.println("OK: All motors stopped");
      break;
    }
    
    case '?': {
      Serial.println("STATUS:");
      for (uint8_t i = 0; i < 4; i++) {
        Serial.print("  Motor ");
        Serial.print(i);
        Serial.print(": Speed=");
        Serial.print(motors[i].speed);
        Serial.print(" Encoder=");
        Serial.println(getEncoderCount(i));
      }
      break;
    }
    
    default:
      Serial.print("ERROR: Unknown command '");
      Serial.print(cmd);
      Serial.println("'");
      break;
  }
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("Pico 2 W Motor Controller Starting...");
  Serial.println("Pico 2 W Motor Controller v1.0");
  
  // Initialize motor direction and PWM pins
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(motors[i].in1_pin, OUTPUT);
    pinMode(motors[i].in2_pin, OUTPUT);
    digitalWrite(motors[i].in1_pin, LOW);
    digitalWrite(motors[i].in2_pin, LOW);
  }
  
  // Initialize encoder pins
  pinMode(MOTOR1_ENC_A, INPUT_PULLUP); pinMode(MOTOR1_ENC_B, INPUT_PULLUP);
  pinMode(MOTOR2_ENC_A, INPUT_PULLUP); pinMode(MOTOR2_ENC_B, INPUT_PULLUP);
  pinMode(MOTOR3_ENC_A, INPUT_PULLUP); pinMode(MOTOR3_ENC_B, INPUT_PULLUP);
  pinMode(MOTOR4_ENC_A, INPUT_PULLUP); pinMode(MOTOR4_ENC_B, INPUT_PULLUP);
  
  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(MOTOR1_ENC_A), encoderISR_Motor1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR1_ENC_B), encoderISR_Motor1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_ENC_A), encoderISR_Motor2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_ENC_B), encoderISR_Motor2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR3_ENC_A), encoderISR_Motor3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR3_ENC_B), encoderISR_Motor3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR4_ENC_A), encoderISR_Motor4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR4_ENC_B), encoderISR_Motor4, CHANGE);
  
  Serial.println("Initialization complete!");
  Serial.println("READY");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  handleSerial();
  delay(1);
}