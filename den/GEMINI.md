# Neuro-Mech Eye Control System

## Proje Özeti
Bu proje, MediaPipe ve OpenCV kullanarak göz hareketleriyle 4 eksenli bir robot kolunu kontrol eden ve NeuroSky EEG başlığından gelen dikkat (Attention) verilerini entegre eden bir sistemdir.

## Güncel Durum (20 Aralık 2025)

### 1. Sistem Mimarisi
- **PC (Python):** Görüntü işleme, göz takibi, joystick mantığı ve GUI.
- **Arduino (C++):** Servo motor kontrolü ve EEG sensör verilerinin okunup PC'ye iletilmesi.
- **İletişim:** Çift yönlü Seri Haberleşme (115200 Baud).
  - PC -> Arduino: `<base, vert, depth, gripper>` (Motor açıları)
  - Arduino -> PC: `[EEG:Attention,Signal]` (Beyin sinyalleri)

### 2. Yapılan Son Değişiklikler
- **Hassasiyet Artırımı:** X ve Y eksenlerindeki göz hareketleri **4.0x** çarpanı ile güçlendirildi.
- **Ölü Bölge (Deadzone):** Çok hassas kontrol için düşürüldü.
  - `DEADZONE_X`: 0.015
  - `DEADZONE_Y`: 0.005
- **Analog Hızlanma:** Göz merkezen ne kadar uzaklaşırsa motor o kadar hızlı dönüyor (3.0x limite kadar).
- **Eksen Yönleri:**
  - **Y (Vert):** Yukarı bakış açıyı artırır (Tersine çevrildi).
  - **Z (Depth):** Yüze yaklaşınca kol geri çekilir (Tersine çevrildi, 0-180 arası).
- **GUI:** Gri ekran sorununu önlemek için "Güvenli Pencere" moduna geçildi. Basit ama işlevsel bilgi paneli eklendi.
- **EEG Entegrasyonu:** Arduino artık Attention ve Signal Quality verilerini anlık olarak PC'ye aktarıyor.
- **Port Tarama:** Arduino portu (örn. COM11) otomatik tespit ediliyor.

### 3. Dosya Yapısı ve Ayarlar
- **`main.py`:** Ana Python kodu.
  - `BASE_SPEED`: 1.5
  - `VERT_SPEED`: 1.0
  - Tuşlar: 'C' (Kalibrasyon), 'ESC' (Çıkış).
- **`robot_arm_eeg.ino`:** Arduino kodu.
  - Pinler: Base(2), Vert(3), Depth(4), Gripper(5), EEG RX(10), TX(11).
  - Baud Rate: 115200.

### 4. Bilinen Sorunlar ve Çözümleri
- **Gri Ekran:** Windows'ta `CAP_DSHOW` veya Fullscreen modları bazı sürücülerde görüntü vermiyor. Şu an standart `VideoCapture` ve `WINDOW_NORMAL` kullanılarak sorun çözüldü.
- **Sinyal Kalitesi 200:** EEG başlığı cilde tam temas etmediğinde sinyal kalitesi 200 (Sinyal Yok) döner ve Attention 0 olur. Kulak klipsi ve alın sensörü kontrol edilmeli.

## Nasıl Çalıştırılır
1. Arduino kodunu (`robot_arm_eeg.ino`) karta yükleyin.
2. Python kütüphanelerini kurun: `pip install opencv-python mediapipe pyserial numpy`
3. Başlatın: `python main.py`
4. Kameraya bakın ve nötr pozisyonda 'C' tuşuna basarak kalibre edin.
