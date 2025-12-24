#include <Servo.h>
#include <SoftwareSerial.h>

// --- YAPILANDIRMA ---
// Eger veri kacirma olursa bu pinleri 8 ve 9 yapip 'AltSoftSerial' kutuphanesini deneyebilirsin.
#define EEG_RX 10 
#define EEG_TX 11 

// Servo Pinleri (TOPLAM 4 ADET)
#define SERVO_PIN_BASE 2     // Sag/Sol
#define SERVO_PIN_VERT 3     // Yukari/Asagi
#define SERVO_PIN_DEPTH 4    // Ileri/Geri
#define SERVO_PIN_GRIPPER 5  // Kiskac

#define LED_PIN 13
#define RECONNECT_INTERVAL 5000

// Servo Tanımlamaları
#define SERVO_COUNT 4
const int servoPins[SERVO_COUNT] = {SERVO_PIN_BASE, SERVO_PIN_VERT, SERVO_PIN_DEPTH, SERVO_PIN_GRIPPER};

// --- NESNELER ---
SoftwareSerial eegSerial(EEG_RX, EEG_TX);
Servo servoBase;
Servo servoVert;
Servo servoDepth;
Servo servoGripper;

// --- DEGISKENLER ---
// EEG
unsigned long lastPacketTime = 0;
int badSignalCount = 0;
byte currentSignalQuality = 200; // Varsayilan: Sinyal yok
bool eegConnectionGood = false;
byte currentAttention = 0;

// Mantik Kontrolu
bool gripperState = false;       // false: Acik, true: Kapali
bool eegTriggerActive = false;   // Schmitt trigger icin
bool pcBlinkTriggerActive = false;

// PC Haberlesme
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

// Hedef Acilar (Baslangic pozisyonu)
int targetBase = 90;
int targetVert = 90;
int targetDepth = 90;

void setup() {
  // PC ile USB uzerinden haberlesme (115200 Baud - Hizli)
  Serial.begin(115200);
  
  // EEG ile SoftwareSerial uzerinden haberlesme
  // DIKKAT: Modulun 115200 ise burayi 115200 yap. 
  eegSerial.begin(115200); 

  // Servolari baslat
  servoBase.attach(SERVO_PIN_BASE);
  servoVert.attach(SERVO_PIN_VERT);
  servoDepth.attach(SERVO_PIN_DEPTH);
  servoGripper.attach(SERVO_PIN_GRIPPER);

  resetServos();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Serial.println("Sistem (4 Servo) Baslatildi. PC verisi ve EEG bekleniyor...");
  delay(1000);
  connectHeadset();
}

void loop() {
  // 1. PC'den Gelen Veriyi Oku (Non-blocking)
  readPCSerial();
  if (newData) {
    parsePCData();
    newData = false;
  }

  // 2. EEG Verisini Oku
  if (readEEGPacket()) {
    lastPacketTime = millis();
    badSignalCount = 0;
    eegConnectionGood = true;
  }

  // 3. Baglanti Kontrolu (Opsiyonel)
  checkConnection();

  // 4. Servolari Sur
  updateServos();
}

// --- PC VERI OKUMA (Non-blocking) ---
// Format: <base,vert,depth,blink> ornegin: <90,45,120,0>
void readPCSerial() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } else {
        receivedChars[ndx] = '\0'; // String sonlandir
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void parsePCData() {
  char * strtokIndx; 

  // 1. Deger: Base
  strtokIndx = strtok(receivedChars, ",");
  if (strtokIndx == NULL) return;
  targetBase = atoi(strtokIndx);

  // 2. Deger: Vertical
  strtokIndx = strtok(NULL, ",");
  if (strtokIndx == NULL) return;
  targetVert = atoi(strtokIndx);

  // 3. Deger: Depth
  strtokIndx = strtok(NULL, ",");
  if (strtokIndx == NULL) return;
  targetDepth = atoi(strtokIndx);

  // 4. Deger: Blink (Goz kirpma 1 veya 0)
  strtokIndx = strtok(NULL, ",");
  if (strtokIndx == NULL) return;
  int blinkStatus = atoi(strtokIndx);

  handleBlinkLogic(blinkStatus);
}

// --- MANTIK KONTROLU ---
void handleBlinkLogic(int blinkStatus) {
  // PC 1 gonderirse (1 sn kapali kaldi demektir) toggle yap
  if (blinkStatus == 1) {
    if (!pcBlinkTriggerActive) {
      toggleGripper();
      pcBlinkTriggerActive = true;
    }
  } else {
    pcBlinkTriggerActive = false; // Goz acildi, tetigi sifirla
  }
}

void handleEEGLogic(byte attention) {
  currentAttention = attention;
  
  // Histeresis: 60 ustu tetikle, 45 alti resetle
  if (attention > 60) {
    if (!eegTriggerActive) {
      toggleGripper();
      eegTriggerActive = true;
      digitalWrite(LED_PIN, HIGH); 
    }
  } else if (attention < 45) {
    eegTriggerActive = false;
    digitalWrite(LED_PIN, LOW);
  }
}

void toggleGripper() {
  gripperState = !gripperState;
  if (gripperState) {
    servoGripper.write(60); // Kapa (60 derece ile sinirli)
  } else {
    servoGripper.write(0);   // Ac (0 derece)
  }
}

void updateServos() {
  servoBase.write(constrain(targetBase, 0, 180));
  servoVert.write(constrain(targetVert, 0, 180));
  servoDepth.write(constrain(targetDepth, 0, 180));
}

// --- EEG OKUMA ---
bool readEEGPacket() {
  if (eegSerial.available() >= 2) {
    if (eegSerial.read() == 0xAA) {
       if (eegSerial.read() == 0xAA) {
        
        unsigned long startWait = millis();
        while(!eegSerial.available()) {
          if(millis() - startWait > 5) return false; 
        }
        
        byte payloadLength = eegSerial.read();
        if (payloadLength > 169) return false;

        byte payload[256];
        int bytesRead = 0;
        
        startWait = millis();
        while (bytesRead < payloadLength) {
          if (eegSerial.available()) {
            payload[bytesRead++] = eegSerial.read();
          }
          if (millis() - startWait > 20) return false; 
        }

        startWait = millis();
        while(!eegSerial.available()) {
           if(millis() - startWait > 5) return false;
        }
        byte checksum = eegSerial.read();

        byte computedChecksum = 0;
        for (int i = 0; i < payloadLength; i++) {
          computedChecksum += payload[i];
        }
        computedChecksum = 255 - computedChecksum;

        if (checksum == computedChecksum) {
          parseEEGPayload(payload, payloadLength);
          return true;
        }
      }
    }
  }
  return false;
}



void parseEEGPayload(byte* data, int length) {
  int i = 0;
  while (i < length) {
    byte code = data[i++];
    switch (code) {
      case 0x02: { // Signal Quality
        currentSignalQuality = data[i++]; // Ham sinyal kalitesini sakla (0: Iyi, 200: Temassiz)
        if (currentSignalQuality > 200) badSignalCount++; // Eski mantik icin
        break;
      }
      case 0x04: { // Attention
        byte att = data[i++];
        handleEEGLogic(att);
        
        // PC'ye Veri Gonder (Attention geldiyse paket tamamdir)
        Serial.print("[EEG:");
        Serial.print(att);
        Serial.print(",");
        Serial.print(currentSignalQuality);
        Serial.println("]");
        break;
      }
      case 0x80: i += 3; break;
      case 0x83: i += 25; break;
      default: i += data[i++]; break; 
    }
  }
}

void connectHeadset() {
  eegSerial.write(0xC2); 
}

void checkConnection() {
  if (millis() - lastPacketTime > RECONNECT_INTERVAL || badSignalCount >= 50) {
    lastPacketTime = millis();
    badSignalCount = 0;
  }
}

void resetServos() {
  servoBase.write(90);
  servoVert.write(90);
  servoDepth.write(90);
  servoGripper.write(0);
}