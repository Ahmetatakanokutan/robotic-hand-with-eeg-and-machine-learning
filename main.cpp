#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <iostream>
#include <string>
#include <chrono>
// For Serial Communication on Windows
#include <windows.h>

using namespace cv;
using namespace std;

// ==========================================
// SERIAL PORT CLASS (Simple Windows Impl)
// ==========================================
class SerialPort {
private:
    HANDLE hSerial;
    bool connected;
    COMSTAT status;
    DWORD errors;

public:
    SerialPort(const char *portName) {
        connected = false;
        hSerial = CreateFileA(portName,
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            NULL);

        if (hSerial == INVALID_HANDLE_VALUE) {
            if (GetLastError() == ERROR_FILE_NOT_FOUND) {
                printf("ERROR: Port %s not available.\n", portName);
            }
            return;
        }

        DCB dcbSerialParams = { 0 };
        if (!GetCommState(hSerial, &dcbSerialParams)) {
            printf("failed to get current serial parameters!");
            return;
        }

        dcbSerialParams.BaudRate = CBR_115200;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;
        dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;

        if (!SetCommState(hSerial, &dcbSerialParams)) {
            printf("ALERT: Could not set Serial Port parameters\n");
            return;
        }

        connected = true;
        PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
        Sleep(2000); // 2s wait for Arduino reset
    }

    ~SerialPort() {
        if (connected) {
            CloseHandle(hSerial);
            connected = false;
        }
    }

    bool isConnected() { return connected; }

    bool writeSerialPort(const string &data) {
        DWORD bytesSend;
        if (!WriteFile(hSerial, (void*)data.c_str(), data.length(), &bytesSend, 0)) {
            ClearCommError(hSerial, &errors, &status);
            return false;
        }
        return true;
    }
};

// ==========================================
// MAIN APPLICATION
// ==========================================

// Global Thresholds
const int BLINK_FRAMES_THRESH = 30; // ~1 second at 30fps

int mapToServo(int val, int minVal, int maxVal, bool invert = false) {
    int angle = (int)((val - minVal) * 180.0 / (maxVal - minVal));
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    return invert ? (180 - angle) : angle;
}

int main() {
    // 1. SETUP SERIAL
    // CHANGE THIS to your Arduino Port (e.g., "COM3")
    string portName = "\\\\.\\COM3"; 
    SerialPort arduino(portName.c_str());

    if (arduino.isConnected()) {
        cout << "Arduino Connected on " << portName << endl;
    } else {
        cout << "WARNING: Arduino NOT connected. Running in simulation mode." << endl;
    }

    // 2. SETUP CAMERA
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "Error: Could not open camera" << endl;
        return -1;
    }

    // Load Face Cascade (Standard OpenCV)
    // You must have haarcascade_frontalface_alt.xml in the run directory!
    CascadeClassifier faceCascade;
    if (!faceCascade.load("haarcascade_frontalface_alt.xml")) {
        cerr << "Error loading haarcascade. Make sure xml file is in local folder." << endl;
        // Proceeding anyway but detection won't work
    }

    // Create Eye Cascade
    CascadeClassifier eyeCascade;
    if (!eyeCascade.load("haarcascade_eye_tree_eyeglasses.xml")) {
        cerr << "Warning: Could not load eye cascade." << endl;
    }

    auto blinkStartTime = chrono::steady_clock::now();
    bool eyesClosed = false;
    bool blinkTrigger = false;

    while (true) {
        Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        equalizeHist(gray, gray);

        // Detect Face
        std::vector<Rect> faces;
        faceCascade.detectMultiScale(gray, faces, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(60, 60));

        int x_center = 90;
        int y_center = 90;
        int z_depth = 90;
        int blink_cmd = 0;

        if (faces.size() > 0) {
            // Track LARGEST face
            Rect face = faces[0];
            rectangle(frame, face, Scalar(255, 0, 0), 2);

            // 1. GAZE X/Y (Based on Face Position for simplicity without heavy Deep Learning)
            // Real Gaze tracking needs pupil center, but head pose is a good proxy for "looking" direction for a robot arm.
            // If user moves head left, robot moves left.
            
            // Map Frame X (0-640) to Angle (180-0) -> Inverted usually feels more natural (Mirror)
            int centerX = face.x + face.width / 2;
            int centerY = face.y + face.height / 2;
            
            x_center = mapToServo(centerX, 0, frame.cols, true);
            y_center = mapToServo(centerY, 0, frame.rows, false); // Up is Up

            // 2. DEPTH (Z) - Based on Face Size
            // Closer face = Larger Width = Robot moves back (or forward depending on preference)
            // Let's say closer face = Robot Retracts (avoids collision)
            // Range estimated: 100px (far) to 300px (close)
            z_depth = mapToServo(face.width, 100, 300, true);

            // 3. EYE BLINK (Simple heuristic)
            // ROI for eyes
            Mat faceROI = gray(face);
            std::vector<Rect> eyes;
            eyeCascade.detectMultiScale(faceROI, eyes, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));

            // Logic: If face found but 0 eyes found -> Eyes Closed?
            // This is noisy. Better approach is DLib, but for pure OpenCV XML:
            // If < 1 eye detected inside face, assume closed.
            if (eyes.size() < 1) {
                if (!eyesClosed) {
                    blinkStartTime = chrono::steady_clock::now();
                    eyesClosed = true;
                } else {
                    auto now = chrono::steady_clock::now();
                    auto duration = chrono::duration_cast<chrono::milliseconds>(now - blinkStartTime).count();
                    if (duration > 1000) { // 1 second
                         blink_cmd = 1;
                         putText(frame, "BLINK TRIGGER!", Point(50, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 3);
                    }
                }
            } else {
                eyesClosed = false;
                blink_cmd = 0;
            }

            // Draw Eyes
            for (size_t j = 0; j < eyes.size(); j++) {
                Point eyeCenter(face.x + eyes[j].x + eyes[j].width / 2, face.y + eyes[j].y + eyes[j].height / 2);
                int radius = cvRound((eyes[j].width + eyes[j].height) * 0.25);
                circle(frame, eyeCenter, radius, Scalar(255, 255, 0), 4);
            }
        }

        // Send to Arduino
        // Format: <base, vert, depth, blink>
        char encoded[64];
        sprintf_s(encoded, "<%d,%d,%d,%d>", x_center, y_center, z_depth, blink_cmd);
        
        if (arduino.isConnected()) {
            arduino.writeSerialPort(string(encoded));
        }

        // Debug Display
        putText(frame, encoded, Point(10, frame.rows - 20), FONT_HERSHEY_PLAIN, 2, Scalar(0, 255, 0), 2);
        imshow("Eye Tracking Control", frame);

        if (waitKey(10) == 27) break; // ESC
    }

    return 0;
}
