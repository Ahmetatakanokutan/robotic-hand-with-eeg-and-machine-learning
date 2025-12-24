#include <iostream>
#include <string>
#include <vector>
#include <chrono>

// OpenCV kutuphaneleri (Sisteminde yuklu olmali)
// #include <opencv2/opencv.hpp>
// #include <opencv2/objdetect.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/imgproc.hpp>

// Seri port haberlesmesi icin basit bir yapi (Windows icin <windows.h> kullanilir)
// Gercek uygulamada 'SerialPort' sinifi gereklidir.

using namespace std;
// using namespace cv;

// --- ACIKLAMALAR ---
// Bu kod, C++ ve OpenCV kullanarak yapman gereken mantigi gosterir.
// Windows'ta C++ ile Serial Port acmak ve OpenCV kurmak biraz zahmetlidir.
// Asagidaki algoritmayi takip etmelisin.

/*
GERCEK KODLAMA ICIN ADIMLAR:
1. Visual Studio Community yukle.
2. OpenCV'yi indir ve projeye bagla (Include ve Linker ayarlari).
3. "Serial Class C++ Windows" diye aratip hazir bir Serial sinifi bul ve projeye ekle.
*/

int main() {
    cout << "Goz Takip ve Robot Kontrol Sistemi Baslatiliyor..." << endl;

    // 1. Seri Portu Ac (COM3, COM4 vb. Arduino'nun takili oldugu port)
    // SerialPort arduino("COM3");
    // if (!arduino.isConnected()) return -1;

    // 2. Kamera Ac
    // VideoCapture cap(0);
    // if (!cap.isOpened()) return -1;

    // 3. Yuz Tanima Modelini Yukle (Haar Cascade veya DNN)
    // CascadeClassifier faceCascade;
    // faceCascade.load("haarcascade_frontalface_alt2.xml");
    
    // Goz modelini yukle (haarcascade_eye.xml) veya MediaPipe kullan.
    // NOT: OpenCV ile hassas goz bebegi takibi zordur. 
    // "MediaPipe" C++ wrapper'i kullanmak en iyisidir ama kurulumu zordur.
    // Alternatif: "dlib" kutuphanesi ile 68 nokta yuz takibi yapabilirsin.

    /*
    ANA DONGU:
    while (true) {
        Mat frame;
        cap >> frame;
        
        // Yuz tespiti
        vector<Rect> faces;
        faceCascade.detectMultiScale(frame, faces);

        if (faces.size() > 0) {
            Rect face = faces[0];
            
            // --- 1. Kafa Hareketi (Derinlik) ---
            // Yuzun cercevedeki buyuklugu (width) yakinligi verir.
            // Yakinlasinca alan buyur -> Kol geri cekilsin
            // Uzaklasinca alan kuculur -> Kol ileri gitsin
            int faceSize = face.width;
            int depthAngle = map(faceSize, 50, 200, 180, 0); // Ters oranti

            // --- 2. Goz Takibi (Basit Yontem) ---
            // Yuzun icindeki goz bolgesini al (ROI)
            Mat faceROI = frame(face);
            
            // Goz merkezini bul (Burasi karmasiktir, basitce yuzun merkezine gore bakis tahmini)
            // Daha ileri seviye icin 'GazeTracking' kutuphanesi gerekir.
            
            int lookX = ...; // Goz bebegi X kordinati
            int lookY = ...; // Goz bebegi Y kordinati
            
            int baseAngle = map(lookX, 0, frame.cols, 180, 0); // Sag/Sol
            int vertAngle = map(lookY, 0, frame.rows, 0, 180); // Yukari/Asagi

            // --- 3. Goz Kirpma (Blink) ---
            // Goz kapaliligi icin EAR (Eye Aspect Ratio) hesaplanir.
            // Eger (EAR < 0.2) ise goz kapali sayilir.
            
            static auto blinkStartTime = chrono::steady_clock::now();
            static bool isBlinking = false;
            int blinkCommand = 0;

            if (eyesClosed) {
                if (!isBlinking) {
                    blinkStartTime = chrono::steady_clock::now();
                    isBlinking = true;
                } else {
                    auto now = chrono::steady_clock::now();
                    if (chrono::duration_cast<chrono::seconds>(now - blinkStartTime).count() >= 1) {
                        blinkCommand = 1; // 1 saniye gecti
                    }
                }
            } else {
                isBlinking = false;
                blinkCommand = 0;
            }

            // --- 4. Arduino'ya Gonder ---
            // Format: <base,vert,depth,blink>
            string data = "<" + to_string(baseAngle) + "," + 
                                to_string(vertAngle) + "," + 
                                to_string(depthAngle) + "," + 
                                to_string(blinkCommand) + ">";
            
            // arduino.writeSerialPort(data.c_str(), data.length());
        }
    }
    */

    return 0;
}
