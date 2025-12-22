#include <Servo.h>
#include <SoftwareSerial.h>

// --- PIN DEFINITIONS ---
#define LED_PIN 13

// CRITICAL: Connect NeuroSky to these pins, NOT 0/1.
// NeuroSky TX -> Arduino Pin 10
// NeuroSky RX -> Arduino Pin 11
#define EEG_RX 10 
#define EEG_TX 11 

// --- SERVO CONFIGURATION ---
#define SERVO_COUNT 5
// 0: Base (Left/Right)
// 1: Vertical (Up/Down)
// 2: Depth (Forward/Backward)
// 3: Extra/Unused
// 4: Gripper (Open/Close)
const int servoPins[SERVO_COUNT] = {2, 3, 4, 5, 6};

Servo servos[SERVO_COUNT];

#define BASE_SERVO 0     
#define VERTICAL_SERVO 1 
#define DEPTH_SERVO 2    
#define GRIPPER_SERVO 4  

// --- EEG VARIABLES ---
SoftwareSerial eegSerial(EEG_RX, EEG_TX); 
unsigned long lastPacketTime = 0;
int badSignalCount = 0;
int currentAttention = 0;

// --- STATE VARIABLES ---
bool gripperClosed = false;       
bool eegTriggerLocked = false; 
bool eyeTriggerActive = false;
bool actionTriggered = false; // From user's logic

int targetBase = 90;
int targetVert = 90;
int targetDepth = 90;

void setup() {
  // 1. PC Communication (USB)
  Serial.begin(115200); 
  
  // 2. EEG Communication (SoftwareSerial)
  // User specified 115200 is required for their NeuroSky setup.
  // WARNING: SoftwareSerial at 115200 can be unstable. 
  // Ensure short wires and good connections.
  eegSerial.begin(115200); 

  for (int i = 0; i < SERVO_COUNT; i++) {
    servos[i].attach(servoPins[i]);
  }

  resetServos();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // 3. Initialize Headset (User's Sequence)
  delay(3000);
  connectHeadset(); 
  
  Serial.println("SYSTEM_READY: EEG 115200 configured.");
}

void loop() {
  // 1. Process EEG Data
  if (readEEGPacket()) {
    lastPacketTime = millis();
    handleEEGLogic();
  }
  
  // 2. Reconnection Logic (from User's code)
  if (millis() - lastPacketTime > 5000 || badSignalCount >= 10) {
    // Attempt reconnect if signal lost
    // Note: This blocks for 3s in user code. We will make it non-blocking if possible,
    // but sticking to user's logic for reliability:
    // Serial.println("Reconnecting Headset...");
    // connectHeadset(); 
    // badSignalCount = 0;
    // lastPacketTime = millis();
    
    // Non-blocking simple reset counter
    badSignalCount = 0; 
  }

  // 3. Process PC Commands
  readSerialCommand();

  // 4. Update Servos
  updateServos();
}

// -------------------------------------------------------------
// LOGIC
// -------------------------------------------------------------
void toggleGripper() {
  gripperClosed = !gripperClosed;
  if (gripperClosed) {
    servos[GRIPPER_SERVO].write(180); 
  } else {
    servos[GRIPPER_SERVO].write(0);   
  }
}

void handleEEGLogic() {
  // User's Logic: Attention > 60 triggers action
  if (currentAttention > 60 && !eegTriggerLocked) {
     toggleGripper();
     eegTriggerLocked = true; 
  } 
  
  // Hysteresis Reset
  if (currentAttention <= 60) {
    eegTriggerLocked = false;
  }
}

void readSerialCommand() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '<') {
      int val1 = Serial.parseInt();
      int val2 = Serial.parseInt();
      int val3 = Serial.parseInt();
      int blink = Serial.parseInt(); 

      targetBase  = constrain(val1, 0, 180);
      targetVert  = constrain(val2, 0, 180);
      targetDepth = constrain(val3, 0, 180);

      // Eye Blink Logic
      if (blink == 1) {
        if (!eyeTriggerActive) { 
           toggleGripper();
           eyeTriggerActive = true; 
        }
      } else {
        eyeTriggerActive = false; 
      }
    }
  }
}

void updateServos() {
  servos[BASE_SERVO].write(targetBase);
  servos[VERTICAL_SERVO].write(targetVert);
  servos[DEPTH_SERVO].write(targetDepth);
}

// -------------------------------------------------------------
// EEG DRIVER (User's Exact Logic adapted for SoftwareSerial)
// -------------------------------------------------------------
void connectHeadset() {
  // "C de githubda arduino icin hazirlanmis koddaki konfigurasyonlar"
  eegSerial.write(0xC2); // Auto-connect
  delay(3000);
  
  byte enableStream[] = { 0xAA, 0x02, 0x01, 0x02 };
  eegSerial.write(enableStream, sizeof(enableStream));
}

bool readEEGPacket() {
  if (eegSerial.available() >= 2) {
    if (eegSerial.read() == 0xAA) {
      if (eegSerial.read() == 0xAA) {
        // Wait available? (User code does 'while(!Available)').
        // Better to check or block briefly. 
        // SoftwareSerial might drop bytes if we block too long elsewhere.
        
        byte payloadLength = eegSerial.read();
        if (payloadLength > 169) return false;

        byte payload[256];
        int bytesRead = 0;
        unsigned long startTime = millis();

        while (bytesRead < payloadLength) {
          if (eegSerial.available()) {
            payload[bytesRead++] = eegSerial.read();
          }
           // Time out safety
          if (millis() - startTime > 100) return false;
        }

        // Checksum
        byte checksum = eegSerial.read();
        byte computedChecksum = 0;
        for (int i = 0; i < payloadLength; i++) {
          computedChecksum += payload[i];
        }
        computedChecksum = 255 - computedChecksum;

        if (checksum != computedChecksum) {
          return false;
        }

        parsePayload(payload, payloadLength);
        return true;
      }
    }
  }
  return false;
}

void parsePayload(byte* data, int length) {
  int i = 0;
  bool bigPacket = false;
  
  while (i < length) {
    byte code = data[i++];
    switch (code) {
      case 0x02: { // Signal Poor
        byte poor = data[i++];
        if (poor > 200) badSignalCount++; 
        else badSignalCount = 0;
        bigPacket = true;
        break;
      }
      case 0x04: { // Attention
        currentAttention = data[i++];
        break;
      }
      case 0x05: { // Meditation
        byte med = data[i++];
        break;
      }
      case 0x80: i += 3; break;
      case 0x83: i += 25; break;
      default: {
        byte vlen = data[i++];
        i += vlen;
        break;
      }
    }
  }

  if (bigPacket) {
     digitalWrite(LED_PIN, currentAttention != 0 ? HIGH : LOW);
     // Logic handled in main loop or here
  }
}

void resetServos() {
  servos[0].write(90);
  servos[1].write(90);
  servos[2].write(90);
  servos[3].write(0);
  servos[4].write(0);
}
