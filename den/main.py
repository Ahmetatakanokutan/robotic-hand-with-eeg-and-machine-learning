import cv2
import mediapipe as mp
import numpy as np
import time
import serial
import serial.tools.list_ports
import math
import threading

# --- AYARLAR ---
TARGET_PORT = 'COM3' 
BAUD_RATE = 115200
CAMERA_INDEX = 0

# --- HIZ & HASSASIYET ---
BASE_SPEED = 1.5
VERT_SPEED = 1.0 
DEADZONE_X = 0.015 # 0.04'ten 0.015'e dusuruldu (Daha hassas X kontrolu)
DEADZONE_Y = 0.005 

# --- DEGISKENLER ---
calibrated_center_x = 0.50 
calibrated_center_y = 0.00 

current_base = 90.0
current_vert = 90.0
current_depth = 90.0

# EEG Verileri
eeg_attention = 0
eeg_signal = 200 
serial_lock = threading.Lock() 

# --- SERI PORT BASLATMA ---
ports = serial.tools.list_ports.comports()
real_arduino_port = None
print("--- PORT TARANIYOR ---")
for port in ports:
    desc = port.description.upper()
    if "ARDUINO" in desc or "USB SERIAL" in desc or "USB-SERIAL" in desc or "CH340" in desc:
        real_arduino_port = port.device
        print(f"BULUNDU: {real_arduino_port}")

if real_arduino_port: TARGET_PORT = real_arduino_port

ser = None
try:
    ser = serial.Serial(TARGET_PORT, BAUD_RATE, timeout=0.1) 
    print(f"✅ BAGLANDI: {TARGET_PORT}")
    time.sleep(2)
except:
    print("⚠️ Arduino bulunamadi, simulasyon modu.")

# --- ARDUINO OKUMA THREAD'I ---
def read_serial_data():
    global eeg_attention, eeg_signal
    while True:
        if ser and ser.is_open:
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    # Debug icin konsola basalim
                    if "[EEG:" in line:
                         print(f"DEBUG: {line}")
                    
                    if line.startswith("[EEG:") and line.endswith("]"):
                        # [EEG:45,0]
                        content = line[5:-1]
                        parts = content.split(',')
                        if len(parts) == 2:
                            eeg_attention = int(parts[0])
                            eeg_signal = int(parts[1])
            except Exception as e: 
                # print(f"Hata: {e}")
                pass
        time.sleep(0.01)

t = threading.Thread(target=read_serial_data, daemon=True)
t.start()

# --- MEDIAPIPE ---
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(max_num_faces=1, refine_landmarks=True)

def map_range(x, in_min, in_max, out_min, out_max):
    val = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return max(min(val, max(out_min, out_max)), min(out_min, out_max))

# --- ANA DONGU ---
cap = cv2.VideoCapture(CAMERA_INDEX)
window_name = "NEURO-MECH CONTROL"

blink_start = 0
blink_triggered = False
gripper_cmd = 0

while cap.isOpened():
    success, image = cap.read()
    if not success: 
        print("Kamera okunamiyor!")
        time.sleep(0.5)
        continue

    image = cv2.flip(image, 1)
    h, w, _ = image.shape
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = face_mesh.process(rgb)
    
    msg_status = "BEKLEMEDE"
    
    # Baslangic degerleri (Hata onlemek icin)
    cur_x = calibrated_center_x
    cur_y = calibrated_center_y
    diff_y = 0
    
    if results.multi_face_landmarks:
        lms = results.multi_face_landmarks[0].landmark
        
        # Goz Hesaplamalari
        p_left, p_right = lms[468], lms[473]
        cur_x = (p_left.x + p_right.x) / 2
        cur_y = (p_left.y + p_right.y) / 2
        
        diff_y = (cur_y - calibrated_center_y) * 4.0
        diff_x = (cur_x - calibrated_center_x) * 4.0 # X ekseni icin de 4.0 kat guclendirme
        
        # Hareket Mantigi
        status_parts = []
        
        # X Hareketi (Sag/Sol)
        if diff_x < -DEADZONE_X:
            # Hiz carpani: Ne kadar cok bakarsan o kadar hizli (Max 3.0x)
            factor = min(3.0, abs(diff_x) / DEADZONE_X)
            current_base += (BASE_SPEED * factor)
            status_parts.append("SOL")
        elif diff_x > DEADZONE_X:
            factor = min(3.0, abs(diff_x) / DEADZONE_X)
            current_base -= (BASE_SPEED * factor)
            status_parts.append("SAG")
            
        # Y Hareketi (Yukari/Asagi)
        if diff_y < -DEADZONE_Y:
            factor = min(3.0, abs(diff_y) / DEADZONE_Y)
            current_vert += (VERT_SPEED * factor) 
            status_parts.append("YUKARI")
        elif diff_y > DEADZONE_Y:
            factor = min(3.0, abs(diff_y) / DEADZONE_Y)
            current_vert -= (VERT_SPEED * factor)
            status_parts.append("ASAGI")

        # Durum Mesajini Birlestir
        if not status_parts:
            msg_status = "BEKLEMEDE"
        else:
            msg_status = " + ".join(status_parts)

        # Derinlik (Ters: Yakin=180)
        f_w = math.sqrt((lms[33].x - lms[263].x)**2 + (lms[33].y - lms[263].y)**2) * w
        target_depth = map_range(f_w, 40, 150, 0, 180) 
        current_depth = (current_depth * 0.9) + (target_depth * 0.1)

        # Sinirlar
        current_base = max(0, min(180, current_base))
        current_vert = max(0, min(180, current_vert))

        # Blink (Goz Kapama)
        l_dist = abs(lms[159].y - lms[145].y)
        r_dist = abs(lms[386].y - lms[374].y)
        avg_dist = (l_dist + r_dist) / 2
        
        if avg_dist < 0.012:
            if blink_start == 0: blink_start = time.time()
            elif (time.time() - blink_start) > 0.8 and not blink_triggered:
                gripper_cmd = 1
                blink_triggered = True
        else:
            blink_start = 0
            blink_triggered = False
            gripper_cmd = 0

        # Serial Send
        if ser and ser.is_open:
            try:
                with serial_lock:
                    data = f"<{int(current_base)},{int(current_vert)},{int(current_depth)},{int(gripper_cmd)}>"
                    ser.write(data.encode())
            except: pass

        # --- EKRAN CIZIMLERI (Basit ve Guvenli) ---
        
        # 1. Bilgi Paneli (Alt Kisim)
        cv2.rectangle(image, (0, h-120), (w, h), (20,20,20), -1)
        
        # Robot Durumu
        cv2.putText(image, f"ROBOT: {msg_status}", (20, h-90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
        cv2.putText(image, f"POS: X:{int(current_base)} Y:{int(current_vert)} Z:{int(current_depth)}", (20, h-60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)
        grip_txt = "LOCKED" if gripper_cmd else "OPEN"
        cv2.putText(image, f"GRIPPER: {grip_txt}", (20, h-35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255) if gripper_cmd else (0,255,0), 1)

        # EEG Paneli (Sag Alt)
        cv2.line(image, (w-250, h-110), (w-250, h-10), (100,100,100), 1) # Ayirac
        cv2.putText(image, "BRAIN SIGNAL", (w-240, h-90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,255), 1)
        
        # Dikkat Seviyesi
        att_col = (0,255,0) if eeg_attention > 60 else ((0,0,255) if eeg_attention < 30 else (0,255,255))
        cv2.putText(image, f"ATTENTION: {eeg_attention}%", (w-240, h-60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, att_col, 2)
        
        # Sinyal Kalitesi
        sig_txt = "GOOD" if eeg_signal < 25 else ("POOR" if eeg_signal < 200 else "NO SIGNAL")
        sig_col = (0,255,0) if eeg_signal < 25 else (0,0,255)
        cv2.putText(image, f"SIGNAL: {sig_txt} ({eeg_signal})", (w-240, h-35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, sig_col, 1)

        # Debug (Hassasiyet Ayari icin)
        cv2.putText(image, f"Y_Diff: {diff_y:.4f}", (w-240, h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100,100,100), 1)

        # Nisangah
        cx, cy = w // 2, h // 2
        cv2.line(image, (cx-20, cy), (cx+20, cy), (0, 255, 0), 1)
        cv2.line(image, (cx, cy-20), (cx, cy+20), (0, 255, 0), 1)
        joy_x = int(cx + (diff_x * 2000))
        joy_y = int(cy + (diff_y * 1000))
        cv2.circle(image, (joy_x, joy_y), 5, (0, 0, 255), -1)

    # TUSLAR
    key = cv2.waitKey(1) & 0xFF
    if key == ord('c') or key == ord('C'):
        if results.multi_face_landmarks:
            calibrated_center_x = cur_x
            calibrated_center_y = cur_y
            print("✅ KALIBRE EDILDI")
    
    if key == 27: break
    
    cv2.imshow(window_name, image)

if ser: ser.close()
cap.release()
cv2.destroyAllWindows()