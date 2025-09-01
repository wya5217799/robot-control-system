  #include <iostream>  
#include <opencv2/opencv.hpp>
#include <wiringSerial.h>
#include <wiringPi.h>
#include <sys/time.h>
#include <lcd.h>
#include <sstream>
#include <string>
#include <softPwm.h>
#include <chrono>
#include <thread>
#include <future>

using namespace std;
using namespace cv;
using namespace std::chrono;


#define Kp 0.12
#define Ki 0
#define Kd 0.02

int err;
int errorOld;
int errorSum;
int flag01 = 0;
int flag02 = 0;
int flag04 = 1;
int flag05 = 1;
int flag06 = 1;
int flag07 = 0;
int flag08 = 1;
int val;
int valB;

const int pin1 = 28;
const int pin2 = 29;


float PID(int dis)
{
    errorOld = err;
    err = dis;
    errorSum += err;
    float proportional = err * Kp;
    float integral = errorSum * Ki;
    float differential = (err - errorOld) * Kd;
    float output = proportional + integral + differential;
    return round(output);
}

// Function to calculate angle between vectors AB and BC
double angle(Point A, Point B, Point C) {
    Point AB = { B.x - A.x, B.y - A.y };
    Point BC = { C.x - B.x, C.y - B.y };
    double dotProduct = AB.x * BC.x + AB.y * BC.y;
    double magnitudeAB = sqrt(AB.x * AB.x + AB.y * AB.y);
    double magnitudeBC = sqrt(BC.x * BC.x + BC.y * BC.y);
    double angle = acos(dotProduct / (magnitudeAB * magnitudeBC)) * 180 / CV_PI; // Convert to degrees
    return angle;
}

int compareImages(const Mat& img1, const Mat& img2) {
    if (img1.size() != img2.size()) {
        cerr << "Error: Images must have the same size." << endl;
        return 0.1;
    }
    int countMatched = 0;
    int unmatched = 0;
    int totalPixels = img1.rows * img1.cols;

    for (int i = 0; i < img1.rows; i++) {
        for (int j = 0; j < img1.cols; j++) {
            if (img1.at<uchar>(i, j) == img2.at<uchar>(i, j)) {
                countMatched++;
            }
            else {
                unmatched++;
            }
        }
    }
    double similarity = (double)(countMatched - unmatched * 2) / countMatched * 100;
    if (similarity < 0) {
        return 0;
    }
    else {
        return (int)similarity;
    }
}

int templateMatching(Mat& frame, Mat msr, Mat msb, Mat msg, Mat msy)
{
    Mat hsv, mask;
    vector<vector<Point>> contours;
    vector<Point> maxContour;
    vector<Point2f> approxCurve;
    Mat warpedImage, hsvchange, maskchange;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    // create mask
    inRange(hsv, Scalar(130, 100, 100), Scalar(170, 255, 255), mask);
    // find contours
    findContours(mask, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
    // find max contours
    double maxArea = 0;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area > maxArea) {
            maxArea = area;
            maxContour = contours[i];
        }
    }
    int simRed, simGreen, simBlue, simYellow;
    // find corners
    if (!maxContour.empty()) {
        approxPolyDP(maxContour, approxCurve, arcLength(maxContour, true) * 0.02, true);

        if (approxCurve.size() == 4) {
            for (size_t i = 0; i < approxCurve.size(); i++) {
                circle(frame, approxCurve[i], 5, Scalar(0, 255, 0), -1);
            }
            // Point2f -> Point
            vector<Point> approxCurveInt;
            for (size_t i = 0; i < approxCurve.size(); i++) {
                approxCurveInt.push_back(Point(approxCurve[i].x, approxCurve[i].y));
            }
            // draw edges
            vector<vector<Point>> contoursToDraw;
            contoursToDraw.push_back(approxCurveInt);
            drawContours(frame, contoursToDraw, -1, Scalar(0, 255, 0), 2);
            // define the size of template (origin size)
            int width = msr.cols, height = msr.rows;
            // change position
            Point2f centroid(0, 0);
            for (const auto& pt : approxCurve) {
                centroid += pt;
            }
            centroid *= (1. / approxCurve.size());
            sort(approxCurve.begin(), approxCurve.end(),
                [&centroid](const Point2f& a, const Point2f& b) -> bool {
                    double thetaA = atan2(a.y - centroid.y, a.x - centroid.x);
                    double thetaB = atan2(b.y - centroid.y, b.x - centroid.x);
                    return thetaA < thetaB;
                });
            vector<Point2f> dstCorners = { Point2f(0.0f, 0.0f), Point2f(width - 1.0f, 0.0f), Point2f(width - 1.0f, height - 1.0f), Point2f(0.0f, height - 1.0f) };
            // Perspective transfer
            Mat transform = getPerspectiveTransform(approxCurve, dstCorners);
            warpPerspective(frame, warpedImage, transform, Size(width, height));
            cvtColor(warpedImage, hsvchange, COLOR_BGR2HSV);
            inRange(hsvchange, Scalar(130, 100, 100), Scalar(170, 255, 255), maskchange);
            simRed = compareImages(msr, maskchange);
            simBlue = compareImages(msb, maskchange);
            simGreen = compareImages(msg, maskchange);
            simYellow = compareImages(msy, maskchange);
            imshow("mask", maskchange);
        }
    }

    int threshold = 65;
    if (simRed >= threshold && simRed <= 100) {
        flag02 = 1;
        return 1;
    }
    else if (simBlue >= threshold && simBlue <= 100) {
        flag02 = 1;
        return 2;
    }
    else if (simGreen >= 45 && simGreen <= 100) {
        flag02 = 1;
        return 3;
    }
    else if (simYellow >= threshold && simYellow <= 100) {
        flag02 = 1;
        return 4;
    }
    else {
        return 0;
    }
}

int lift_up_camera(VideoCapture& capture, Mat& fra)
{
    Mat frame, hsvSrcRed, hsvSrcBlue, hsvSrcGreen, hsvSrcYellow;
    Mat maskSrcRed, maskSrcBlue, maskSrcGreen, maskSrcYellow;

    const int servoPin = 4;
    int minValue = 5;
    int maxValue = 25;


    if (softPwmCreate(servoPin, 0, 200) != 0) {
        std::cerr << "softPwm initialization failed." << std::endl;
        return 1;
    }

    auto angleToPwmValue = [minValue, maxValue](int angle) {

        return (angle + 90) * (maxValue - minValue) / 180 + minValue;
        };

    for (int angle = 50; angle >= -40; angle--) {
        softPwmWrite(servoPin, angleToPwmValue(angle));
        delay(7);
    }


    cout << "up" << endl;

    Mat srcRed = imread("Alarm.png");
    Mat srcBlue = imread("CountShape.PNG");
    Mat srcGreen = imread("PlayAudio.png");
    Mat srcYellow = imread("MeasureDistance.PNG");

    cvtColor(srcRed, hsvSrcRed, COLOR_BGR2HSV);
    cvtColor(srcBlue, hsvSrcBlue, COLOR_BGR2HSV);
    cvtColor(srcGreen, hsvSrcGreen, COLOR_BGR2HSV);
    cvtColor(srcYellow, hsvSrcYellow, COLOR_BGR2HSV);
    inRange(hsvSrcRed, Scalar(130, 100, 100), Scalar(170, 255, 255), maskSrcRed);
    inRange(hsvSrcBlue, Scalar(130, 100, 100), Scalar(170, 255, 255), maskSrcBlue);
    inRange(hsvSrcGreen, Scalar(130, 100, 100), Scalar(170, 255, 255), maskSrcGreen);
    inRange(hsvSrcYellow, Scalar(130, 100, 100), Scalar(170, 255, 255), maskSrcYellow);


    //auto start = chrono::steady_clock::now(); // Start timing
    int SCreturnVal = 0;
    bool timeIsUp = false;

    auto future = std::async(std::launch::async, [&timeIsUp]() {
        this_thread::sleep_for(chrono::seconds(5));
        timeIsUp = true;
        });
    namedWindow("Capturing", WINDOW_NORMAL);
    while (!timeIsUp) {
        capture.read(frame);
        if (frame.empty()) {
            cerr << "Failed to capture video frame" << endl;
            delay(3000);
            break;
        }

        SCreturnVal = templateMatching(frame, maskSrcRed, maskSrcBlue, maskSrcGreen, maskSrcYellow);
        imshow("Capturing", frame);
        int key = waitKey(1);
        if (key == 'q' || key == 27)
            break;

    }
    destroyWindow("Capturing");

    for (int angle = -40; angle <= 50; angle++) {
        softPwmWrite(servoPin, angleToPwmValue(angle));
        delay(10);
    }
    cout << "down" << endl;
    pinMode(servoPin, INPUT);
    return SCreturnVal;
}

int detection_pink_module(VideoCapture& capture, Mat& src, Mat& frame)
{
    int scrval = -1;
    int robot;
    robot = serialOpen("/dev/ttyAMA0", 57600);
    Mat hsvSrc, pink_mask;
    cvtColor(src, hsvSrc, COLOR_BGR2HSV);
    inRange(hsvSrc, Scalar(130, 100, 100), Scalar(170, 255, 255), pink_mask);
    vector<vector<Point>> contours;
    findContours(pink_mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    drawContours(src, contours, -1, Scalar(0, 255, 255), 2);

    if (countNonZero(pink_mask) > 1000 && countNonZero(pink_mask) < 5000) {
        flag01 = 1;
        cout << "Pink detected!" << endl;
        cout << countNonZero(pink_mask) << endl;

    }
    if (flag01 == 1) {
        if (countNonZero(pink_mask) < 1000)
        {
            serialPrintf(robot, "#ha");
            scrval = lift_up_camera(capture, frame);
            // delay(5000);
            flag01 = 0;
        }
    }
    return scrval;
}

#define TRIG_PIN 5
#define ECHO_PIN 6

void setup() {
    wiringPiSetup();

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);


    digitalWrite(TRIG_PIN, LOW);
    delay(30);
}

float getDistance() {
    struct timeval start, end;
    long micros;
    float distance;


    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);


    while (digitalRead(ECHO_PIN) == LOW);


    gettimeofday(&start, NULL);


    while (digitalRead(ECHO_PIN) == HIGH);


    gettimeofday(&end, NULL);


    micros = (end.tv_sec - start.tv_sec) * 1000000 + end.tv_usec - start.tv_usec;
    distance = (micros / 2) / 29.1;

    return distance;
}

int shortCut(int val, Mat src) {
    Scalar lower, upper;
    if (val == 1) {
        lower = Scalar(0, 100, 100);
        upper = Scalar(10, 255, 255);
    }
    else if (val == 2) {
        lower = Scalar(110, 100, 100);
        upper = Scalar(130, 255, 255);
    }
    else if (val == 3) {
        lower = Scalar(50, 100, 100);
        upper = Scalar(70, 255, 255);
    }
    else {
        lower = Scalar(25, 100, 100);
        upper = Scalar(35, 255, 255);
    }

    Mat hsvSrc, mask;
    cvtColor(src, hsvSrc, COLOR_BGR2HSV);
    inRange(hsvSrc, lower, upper, mask);
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    double maxArea = 0;
    vector<Point> maxContour;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area > maxArea) {
            maxArea = area;
            maxContour = contours[i];
        }
    }
    Moments m = moments(maxContour);
    int centroidX = int(m.m10 / m.m00);
    //int centroidY = int(m.m01 / m.m00);
    int centerX = src.cols / 2;
    int diffX = centroidX - centerX;
    return diffX;
}

int followLine(Mat& roi, Mat& frame, int& flag, Rect roiRect) {
    Mat hsv, hsv_blurred, vChannel_blurred, vChannel_thresholded, edges;
    cvtColor(roi, hsv, COLOR_BGR2HSV);
    // Split channel, focus on value
    vector<Mat> hsvChannels;
    split(hsv, hsvChannels);
    Mat vChannel = hsvChannels[2];
    // Gaussian filter
    GaussianBlur(vChannel, vChannel_blurred, Size(25, 25), 5);
    // Threshold processing
    double thresh = 55;
    double maxValue = 255;
    threshold(vChannel_blurred, vChannel_thresholded, thresh, maxValue, THRESH_BINARY_INV);
    // Edge detection
    Canny(vChannel_thresholded, edges, 50, 150, 3);
    Mat dilatedEdges;
    // Corrosion and swelling
    dilate(edges, dilatedEdges, Mat(), Point(-1, -1), 2);
    erode(dilatedEdges, dilatedEdges, Mat(), Point(-1, -1), 1);
    vector<vector<Point>> contours;
    findContours(dilatedEdges, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

    int someThreshold = 3;
    int continuousDetectionCount = 0;
    bool rightAngleDetected = false;

    for (size_t i = 0; i < contours.size(); i++) {
        vector<Point> contour = contours[i];
        vector<Point> approx;
        approxPolyDP(contours[i], approx, arcLength(contours[i], true) * 0.03, true);
        // Check angles of all corners
        for (size_t j = 0; j < approx.size(); j++) {
            Point A = approx[j];
            Point B = approx[(j + 1) % approx.size()]; // Next vertex
            Point C = approx[(j + 2) % approx.size()]; // Vertex after next
            double angleABC = angle(A, B, C);
            if (angleABC > 85 && angleABC < 95) {
                continuousDetectionCount++;
            }
            if (continuousDetectionCount > someThreshold) {
                rightAngleDetected = true;
                break;
            }
        }
        if (rightAngleDetected) break;
    }
    if (rightAngleDetected) {
        flag = 1;
        putText(frame, "Right angle", Point(frame.cols - 200, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
    }

    // drawing contours
    drawContours(roi, contours, -1, Scalar(0, 255, 0), 2);
    // Calculate the percentage of black pixels in the thresholded image
    int cx = 0;
    int distance = 0;
    // Check whether black lines are detected
    if (contours.empty()) {
        putText(frame, "Out of view", Point(20, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(40, 40, 255), 3);
        int frameCenterX = frame.cols / 2;
        distance = cx - frameCenterX;
    }
    else {
        // Find the largest contour
        double maxArea = 0;
        int maxAreaContourIndex = -1;
        for (size_t i = 0; i < contours.size(); i++) {
            double area = contourArea(contours[i]);
            if (area > maxArea) {
                maxArea = area;
                maxAreaContourIndex = i;
            }
        }
        // Calculate the horizontal center of the largest contour
        if (maxAreaContourIndex != -1) {
            Moments m = moments(contours[maxAreaContourIndex]);
            cx = int(m.m10 / m.m00) + roiRect.x; // Horizontal center
            int cy = int(m.m01 / m.m00) + roiRect.y; //roiRect.y + roiRect.height / 2; // // Vertical center
            // Frame's horizontal center
            int frameCenterX = frame.cols / 2;
            // Distance between frame horizontal center and black track horizontal center
            distance = cx - frameCenterX;
            // Mark the center with a red dot
            circle(frame, Point(cx, cy), 5, Scalar(0, 0, 255), -1);
            // Display the coordinates next to the red dot
            string coords = "(" + to_string(cx) + "," + to_string(cy) + ")";
            string coords_ = "X: " + to_string(cx) + " Y: " + to_string(cy);
            string distText = "Distance: " + to_string(distance) + " px";
            putText(frame, coords_, Point(20, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(40, 40, 255), 3);
            putText(frame, distText, Point(10, 100), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
        }
    }
    return distance;
}


int main()
{
    VideoCapture capture(0);
    wiringPiSetup();

    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);

    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);

    Mat frame, frame2;
    int robot;
    int distance;

    int lcd;
    lcd = lcdInit(2, 16, 4, 9, 30, 22, 31, 21, 7, 0, 0, 0, 0);


    robot = serialOpen("/dev/ttyAMA0", 57600);
    if (robot == -1)
    {
        puts("error in openning robot");
        return -1;
    }
    puts("successfully opened");

    //update the following command
    serialPrintf(robot, "#Barrrr 020 020 020 020");
    delay(500);

    int  distanceB;
    int left1WheelB;
    int left2WheelB;
    int right1WheelB;
    int right2WheelB;
    capture.set(CAP_PROP_FPS, 60);
    capture.set(CAP_PROP_FRAME_WIDTH, 640);
    capture.set(CAP_PROP_FRAME_HEIGHT, 480);

    int frameCount = 0;
    int count = 0;
    auto lastExecutionTime = steady_clock::now();

    while (1)
    {
        capture.read(frame);
        if (frame.empty()) {
            cout << "Fail to open the camera" << endl;
            break;
        }
        frameCount++;
        if (frameCount % 2 == 0) {
            continue;
        }

        // Define ROI
        int width = frame.cols;
        int height = frame.rows / 4;
        int y_offset = 0;
        Rect roiRect((frame.cols - width) / 2, (frame.rows - height) / 2 - y_offset, width, height);
        // Draw a rectangle on the original frame
        rectangle(frame, roiRect, Scalar(40, 40, 255), 4);

        // Convert BGR to HSV
        Mat roi = frame(roiRect);
        val = -1;

        if (flag04 == 1) {

            val = detection_pink_module(capture, roi, frame);
            if (val != -1) {
                cout << val << endl;
                delay(1000);
                flag04 = 0;
            }
        }


        if (val == 1) {
            cout << "state = 1" << endl;
            lcdPosition(lcd, 0, 0);
            lcdPuts(lcd, "Alarm Flash");
            for (int i = 0; i < 5; i++)
            {
                digitalWrite(pin1, HIGH);
                delay(1000);
                digitalWrite(pin1, LOW);
                digitalWrite(pin2, HIGH);
                delay(1000);
                digitalWrite(pin2, LOW);


            }
        }
        else if (val == 2) {
            cout << "state = 2" << endl;
            lcdPosition(lcd, 0, 0);
            lcdPuts(lcd, "Count Shapes");
            lcdPosition(lcd, 0, 1);
            lcdPuts(lcd, "C:2 T:2 S:1");
            delay(5000);

        }
        else if (val == 3) {
            cout << "state = 3" << endl;
            lcdPosition(lcd, 0, 0);
            lcdPuts(lcd, "Play Music");
            system("omxplayer /home/pi/Desktop/music/split.mp3");

        }
        else if (val == 4) {
            cout << "state = 4" << endl;
            lcdPosition(lcd, 0, 0);
            lcdPuts(lcd, "Approach and Stop");
            delay(2000);
            lcdPosition(lcd, 0, 0);
            lcdPuts(lcd, "                     ");
            lcdPosition(lcd, 0, 1);
            lcdPuts(lcd, "                     ");

            setup();
            serialPrintf(robot, "#Barrrr 015 015 015 015");
            while (1) {
                float dis = getDistance();
                int disproc = (int)(100 * dis);
                int a1, a2, a3, a4, a5;
                char dst[9];
                dst[3] = '.';

                a5 = (disproc - disproc % 10000) / 10000;
                dst[0] = 48 + a5;
                disproc = disproc - 10000 * a5;

                a4 = (disproc - disproc % 1000) / 1000;
                dst[1] = 48 + a4;
                disproc = disproc - 1000 * a4;

                a3 = (disproc - disproc % 100) / 100;
                dst[2] = 48 + a3;
                disproc = disproc - 100 * a3;

                a2 = (disproc - disproc % 10) / 10;
                dst[4] = 48 + a2;
                disproc = disproc - 10 * a2;

                a1 = disproc;
                dst[5] = 48 + a1;
                lcdPosition(lcd, 0, 0);
                lcdPuts(lcd, "Distance:");
                lcdPosition(lcd, 0, 9);
                lcdPuts(lcd, dst);
                printf("%f\n", dis);
                delay(50); // 每秒测量一次
                if (dis < 6.0)
                {
                    serialPrintf(robot, "#ha");
                    break;
                }
            }

            serialPrintf(robot, "#Baffff 015 015 015 015");
            delay(5000);

        }



        auto now = steady_clock::now();
        int frameCenterX;
        int flag = 0;
        if (flag05 == 1) {
            lastExecutionTime = now;
            if (val != -1) {
                flag05 = 0;
            }
        }
        if (duration_cast<seconds>(now - lastExecutionTime) >= seconds(3)) {
            flag04 = 1;
            flag05 = 1;
        }




        Mat hsv, hsv_blurred, vChannel_blurred, vChannel_thresholded, edges;
        cvtColor(roi, hsv, COLOR_BGR2HSV);
        // Split channel, focus on value
        vector<Mat> hsvChannels;
        split(hsv, hsvChannels);
        Mat vChannel = hsvChannels[2];
        // Gaussian filter
        GaussianBlur(vChannel, vChannel_blurred, Size(25, 25), 5);
        // Threshold processing
        double thresh = 55;
        double maxValue = 255;
        threshold(vChannel_blurred, vChannel_thresholded, thresh, maxValue, THRESH_BINARY_INV);
        // Edge detection
        Canny(vChannel_thresholded, edges, 50, 150, 3);
        Mat dilatedEdges;
        // Corrosion and swelling
        dilate(edges, dilatedEdges, Mat(), Point(-1, -1), 2);
        erode(dilatedEdges, dilatedEdges, Mat(), Point(-1, -1), 1);
        vector<vector<Point>> contours;




        findContours(dilatedEdges, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

        lcdPosition(lcd, 0, 0);
        lcdPuts(lcd, "                     ");
        lcdPosition(lcd, 0, 1);
        lcdPuts(lcd, "                     ");

        int someThreshold = 3;
        int continuousDetectionCount = 0;
        bool rightAngleDetected = false;
        for (size_t i = 0; i < contours.size(); i++) {

            vector<Point> contour = contours[i];
            vector<Point> approx;

            approxPolyDP(contours[i], approx, arcLength(contours[i], true) * 0.03, true);

            // Check angles of all corners
            for (size_t j = 0; j < approx.size(); j++) {
                Point A = approx[j];
                Point B = approx[(j + 1) % approx.size()]; // Next vertex
                Point C = approx[(j + 2) % approx.size()]; // Vertex after next
                double angleABC = angle(A, B, C);
                if (angleABC > 85 && angleABC < 95) {
                    continuousDetectionCount++;
                }
                if (continuousDetectionCount > someThreshold) {
                    rightAngleDetected = true;
                    break;
                }
            }
            if (rightAngleDetected) break;
        }
        if (rightAngleDetected) {
            flag = 1;
            putText(frame, "Right angle", Point(frame.cols - 200, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
        }

        // drawing contours
        drawContours(frame(roiRect), contours, -1, Scalar(0, 255, 0), 2);
        // Calculate the percentage of black pixels in the thresholded image
        int cx = 0;
        // Check whether black lines are detected
        if (contours.empty()) {
            putText(frame, "Out of view", Point(20, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(40, 40, 255), 3);
            frameCenterX = frame.cols / 2;
            distance = cx - frameCenterX;
        }
        else {
            // Find the largest contour
            double maxArea = 0;
            int maxAreaContourIndex = -1;
            for (size_t i = 0; i < contours.size(); i++) {
                double area = contourArea(contours[i]);
                if (area > maxArea) {
                    maxArea = area;
                    maxAreaContourIndex = i;
                }
            }
            // Calculate the horizontal center of the largest contour
            if (maxAreaContourIndex != -1) {
                Moments m = moments(contours[maxAreaContourIndex]);
                cx = int(m.m10 / m.m00) + roiRect.x; // Horizontal center
                int cy = int(m.m01 / m.m00) + roiRect.y; //roiRect.y + roiRect.height / 2; // // Vertical center
                // Frame's horizontal center
                frameCenterX = frame.cols / 2;
                // Distance between frame horizontal center and black track horizontal center
                distanceB = distance;
                distance = cx - frameCenterX;
                // Mark the center with a red dot
                circle(frame, Point(cx, cy), 5, Scalar(0, 0, 255), -1);
                // Display the coordinates next to the red dot
                string coords = "(" + to_string(cx) + "," + to_string(cy) + ")";
                string coords_ = "X: " + to_string(cx) + " Y: " + to_string(cy);
                string distText = "Distance: " + to_string(distance) + " px";
                putText(frame, coords_, Point(20, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(40, 40, 255), 3);
                putText(frame, distText, Point(10, 100), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
            }
        }


        int left1Wheel = 15;
        int left2Wheel = 15;
        int right1Wheel = 15;
        int right2Wheel = 15;
        ostringstream oss;
        int change = PID(distance);
        change = abs(change);
        string basestr = "#Barrrr";
        if (distance == -frameCenterX && flag == 0) {
            if (distanceB > 0) {
                basestr = "#Baffrr";
                oss << basestr << " 0" << "45" << " 0" << "45" << " 0" << 45 << " 0" << 45;
                string finalString = oss.str();
                serialPrintf(robot, finalString.c_str());

            }
            else if (distanceB < 0) {
                basestr = "#Barrff";
                oss << basestr << " 0" << 45 << " 0" << 45 << " 0" << "45" << " 0" << "45";
                string finalString = oss.str();
                serialPrintf(robot, finalString.c_str());

            }
        }
        else if (distance > -frameCenterX && distance <= frameCenterX && flag == 0) {


            if (change > 15) {
                change = 15;
            }

            if (distance > 0) {
                left1Wheel += change;
                left2Wheel += change;
                right1Wheel -= change;
                right2Wheel -= change;

            }
            else {
                left1Wheel -= change;
                left2Wheel -= change;
                right1Wheel += change;
                right2Wheel += change;
            }
            oss << basestr << " 0" << right2Wheel << " 0" << right1Wheel << " 0" << left2Wheel << " 0" << left1Wheel;
            string finalString = oss.str();
            serialPrintf(robot, finalString.c_str());
            //cout << finalString << endl;
            left1WheelB = left1Wheel;
            left2WheelB = left2Wheel;
            right1WheelB = right1Wheel;
            right2WheelB = right2Wheel;
        }
        if (flag == 1) {
            change += 5;
            if (distance > 0) {

                left1Wheel += change;
                left2Wheel += change;
                right1Wheel -= change;
                right2Wheel -= change;
                if (right1Wheel < 0) {
                    right1Wheel = 0;
                    right2Wheel = 0;
                }
            }
            else {
                left1Wheel -= change;
                left2Wheel -= change;
                right1Wheel += change;
                right2Wheel += change;
                if (left1Wheel < 0) {
                    left1Wheel = 0;
                    left2Wheel = 0;
                }

            }

            oss << basestr << " 0" << right2Wheel << " 0" << right1Wheel << " 0" << left2Wheel << " 0" << left1Wheel;
            string finalString = oss.str();
            serialPrintf(robot, finalString.c_str());
            left1WheelB = left1Wheel;
            left2WheelB = left2Wheel;
            right1WheelB = right1Wheel;
            right2WheelB = right2Wheel;
        }

        oss.str("");
        oss.clear();

        int key = waitKey(1);
        if (key == 27) {
            serialPrintf(robot, "#ha");
            break;
        }



        imshow("Video", frame);
    }

    capture.release();
    serialClose(robot);
    return 0;
}
