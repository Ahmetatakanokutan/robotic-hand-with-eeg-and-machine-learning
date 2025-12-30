import cv2
import time

# Backend'i açıkça belirtiyoruz
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

# 1. ÇÖZÜNÜRLÜĞÜ ZORLA (Çoğu kamera 640x480 veya 1280x720 sever)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# 2. FORMATI MJPG YAP (Veri akışını hızlandırır ve uyumu artırır)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

# 3. KAMERANIN UYANMASI İÇİN BEKLE
print("Kamera baslatiliyor, lutfen bekleyin...")
time.sleep(2) 

if not cap.isOpened():
    print("HATA: Kamera acilamadi!")
else:
    print("BASARILI: Görüntü aliniyor. Çıkmak için 'q'ya bas.")

while cap.isOpened():
    ret, frame = cap.read()
    
    if not ret:
        print("Kare okunamadı (Siyah Ekran Sorunu Devam Ediyor)...")
        # Bazen ilk kareler boş gelebilir, hemen pes etme
        time.sleep(0.1)
        continue
        
    cv2.imshow('Kamera Testi - MJPG Modu', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()