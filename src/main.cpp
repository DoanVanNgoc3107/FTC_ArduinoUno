/**
 * @author Doan Van Ngoc - 104317 - DTD64CL
 * @brief PHAN LOAI SAN PHAM DUA TREN MAU SAC - SERVO CONTROL
 * @teacher Ths. Vu Thi Thu
 * @version 1.0
 * @date 2025-11-14
 * @copyright DOAN VAN NGOC (Don't use without permission)
 */


#include <Arduino.h>
#include <Servo.h>

// Khởi tạo đối tương Servo
Servo servo;

// ==== CONSTANTS ====
// Define Servo
#define SERVO_PIN 9 // Controller pin of servo

// Define Serial
#define BAUD 9600   // Baud rate of serial

// TCS3200
#define S0_PIN 2
#define S1_PIN 3
#define S2_PIN 4
#define S3_PIN 5
#define OUT_PIN 6

int R_value, B_value, G_value;

// ==== GLOBAL VARIABLES ====
volatile unsigned long pluseCount = 0; // GLOBAL - pluse count from TCS3200

unsigned long lastTime = 0; // GLOBAL - Last time
int lastDegree = 0; // GLOBAL - vị trí hiện tại của servo
int targetDegree = 0; // GLOBAL - vị trí mục tiêu

// ==== SERVO ====

/**
 * @brief This function use sets up a servo pin
 * @param servo_pin (int) : servo_pin
 */
void servo_setup(int const servo_pin)
{
    servo.attach(servo_pin);
    lastDegree = servo.read();
    targetDegree = lastDegree;
    Serial.println("Servo setup complete on pin :" + String(servo_pin));
    Serial.println("Enter angle (0-180) to control servo:");
}

/**
 * @brief This function uses to get current degree of the servo
 * @return degree (int) : degree of the servo
 */
int getDegCurrent()
{
    return lastDegree;
}

/**
 * @brief This function uses to set target degree of servo
 * @param degree (int) : target degree (0-180)
 */
void servo_active(int const degree)
{
    if (degree < 0 || degree > 180)
    {
        Serial.println("Degree invalid (0–180), received: " + String(degree));
        return;
    }

    // Only update if targetDegree is different
    if (targetDegree != degree)
    {
        targetDegree = degree;
        Serial.println("New target set: " + String(degree) + " degrees");
    }
}

/**
 * @brief This function moves servo gradually to the target position
 * <br>
 * Call this in loop() to enable smooth movement
 */
void servo_update()
{
    const unsigned long currentTime = millis();

    if (lastDegree != targetDegree && currentTime - lastTime > 20)
    {
        lastTime = currentTime;

        if (lastDegree < targetDegree)
        {
            lastDegree++;
        }
        else
        {
            lastDegree--;
        }

        servo.write(lastDegree);
        Serial.println("Moving servo to: " + String(lastDegree));
    }
}

// ==== SERIAL ====

/**
 * @brief This function uses to set up a serial port
 * @param baud (int): baud of serial
 * <br>
 * @if baud <= 0, print error message and return
 */
void serial_setup(int const baud)
{
    if (baud <= 0)
    {
        Serial.println("Baud of serial is invalid.!");
        return;
    }
    Serial.begin(baud);
    Serial.println("Serial sets up complete with baud: " + String(baud));
}

/**
 * @brief Check for serial input and return degree value
 * @return int: degree value from serial input, -1 if no valid input
 */
int check()
{
    if (Serial.available())
    {
        // Đọc toàn bộ input để tránh buffer còn dư
        while ((Serial.available() && Serial.peek() == '\n') || Serial.peek() == '\r')
        {
            Serial.read();
        }

        if (Serial.available())
        {
            const int input = Serial.parseInt();

            // Xóa buffer còn lại
            while (Serial.available())
            {
                Serial.read();
            }
            return input;
        }
    }
    return -1;
}

// ==== TCS3200 ====

void countPluse()
{
    pluseCount++;
}

/**
 * @brief This function initializes the TCS3200 sensor pins and sets the frequency scale.
 * @param S0_pin (int), S1_pin (int), S2_pin (int), S3_pin (int), OUT_pin (int) : TCS3200 pins
 */
void tcs_setup(int const S0_pin, int const S1_pin, int const S2_pin, int const S3_pin, int const OUT_pin)
{
    // 1. Khai báo chế độ chân: S0-S3 là OUTPUT, OUT là INPUT
    pinMode(S0_pin, OUTPUT);
    pinMode(S1_pin, OUTPUT);
    pinMode(S2_pin, OUTPUT);
    pinMode(S3_pin, OUTPUT);
    pinMode(OUT_pin, INPUT);

    // 2. Thiết lập hệ số chia tần số (Scale Factor)
    // Chúng ta chọn 20% (S0 = HIGH, S1 = LOW) để có dải đo ổn định và độ phân giải tốt.
    digitalWrite(S0_pin, HIGH);
    digitalWrite(S1_pin, LOW);

    Serial.println("TCS3200 setup complete with 20% scaling.");
}


void readColor()
{
}

void setup()
{
    serial_setup(BAUD);
    servo_setup(SERVO_PIN);
}

void loop()
{
    const int deg = check();

    // Chỉ cập nhật target khi có input hợp lệ
    if (deg >= 0)
    {
        servo_active(deg);
    }

    // Cập nhật vị trí servo từng bước
    servo_update();

    // In thông tin vị trí hiện tại (tùy chọn)
    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime > 5000)
    {
        // In mỗi 5 giây
        lastPrintTime = millis();
        Serial.println("Status - Current: " + String(getDegCurrent()) +
            "°, Target: " + String(targetDegree) + "°");
    }
}
