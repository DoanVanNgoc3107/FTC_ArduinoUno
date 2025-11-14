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
#define BAUD 9600 // Baud rate of serial

// TCS3200
#define S0_PIN 2
#define S1_PIN 3
#define S2_PIN 4
#define S3_PIN 5
#define OUT_PIN 6

#define TIME_PLUSE_COUNT 500 // Thời gian đếm xung (ms)
#define DELAY_READ_COLOR 100 // Thời gian chờ giữa các lần đọc màu (ms)
#define THRESHOLD_COLOR 0.9  // Ngưỡng để phân biệt màu sắc (giá trị tùy chỉnh)

int R_value, B_value, G_value;

// ==== GLOBAL VARIABLES ====
volatile unsigned long pluse = 0; // GLOBAL - pluse count from TCS3200

unsigned long lastTime = 0; // GLOBAL - Last time
int lastDegree = 0;         // GLOBAL - vị trí hiện tại của servo
int targetDegree = 0;       // GLOBAL - vị trí mục tiêu

// ==== Function Utilities ====
/**
 * @brief This function checks if a certain delay time has passed since the last recorded time.
 * @param lastTime
 * @param delayTime
 * @return
 */
void checkTimeDelay(unsigned long &start, unsigned const long delayTime)
{
    if (millis() - start >= delayTime)
    {
        lastTime = millis();
        return;
    }
}

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
 * @brief This function uses to set target degree of servo based on color
 * @param color (char) : color string ("RED", "GREEN", "BLUE")
 * @param degree_s1 (int) : degree for RED
 * @param degree_s2 (int) : degree for GREEN
 * @param degree_s3 (int) : degree for BLUE
 */
void servo_service(char const color, int const degree_s1, int const degree_s2, int const degree_s3)
{
    switch (color)
    {
    case 'R':
        servo_active(degree_s1);
        break;
    case 'G':
        servo_active(degree_s2);
        break;
    case 'B':
        servo_active(degree_s3);
        break;
    default:
        Serial.println("Color not recognized, servo not moved.");
        break;
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

// ==== TCS3200 ===

/**
 * @brief This is an interrupt service routine (ISR) to count pulses from TCS3200 sensor.
 */
void countPluse()
{
    pluse++;
}

/**
 * @brief This function initializes the TCS3200 sensor pins and sets the frequency scale.
 * @param S0_pin (int), S1_pin (int), S2_pin (int), S3_pin (int), OUT_pin (int) : TCS3200 pins
 * @param S1_pin (int) : pin S1 of TCS3200
 * @param S2_pin (int) : pin S2 of TCS3200
 * @param S3_pin (int) : pin S3 of TCS3200
 * @param OUT_pin (int) : pin OUT of TCS3200
 */
void tcs_setup(int const S0_pin, int const S1_pin, int const S2_pin, int const S3_pin, int const OUT_pin)
{
    // Configure pin modes
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

/**
 * @brief This function reads color frequency from TCS3200 sensor based on S2 and S3 states.
 * @param S2_state (bool) : state for S2 pin
 * @param S3_state (bool) : state for S3 pin
 * @return unsigned long : pulse count corresponding to the color frequency
 */
int read_color_frequency(bool const S2_state, bool const S3_state)
{
    digitalWrite(S2_PIN, S2_state);
    digitalWrite(S3_PIN, S3_state);

    pluse = 0; // Reset biến đếm
    attachInterrupt(digitalPinToInterrupt(OUT_PIN), countPluse, RISING);
    delay(TIME_PLUSE_COUNT); // Đợi 500ms để đếm
    detachInterrupt(digitalPinToInterrupt(OUT_PIN));

    return static_cast<int>(pluse);
}

/**
 * @brief This function updates the RGB color values by reading from the TCS3200 sensor.
 */
void updateTCS()
{
    unsigned long currentTime = millis();

    // RED : S2 = LOW, S3 = LOW
    R_value = read_color_frequency(LOW, LOW);
    checkTimeDelay(currentTime, DELAY_READ_COLOR);

    // GREEN : S2 = HIGH, S3 = HIGH
    G_value = read_color_frequency(HIGH, HIGH);
    checkTimeDelay(currentTime, DELAY_READ_COLOR);

    // BLUE : S2 = LOW, S3 = HIGH
    B_value = read_color_frequency(LOW, HIGH);
}

/**
 * @brief This function gets the current RGB color values read from the TCS3200 sensor.
 * @param R (int&) : reference to store Red value
 * @param G (int&) : reference to store Green value
 * @param B (int&) : reference to store Blue value
 */
char getColorString()
{
    int max_value = max(R_value, max(G_value, B_value));

    float const THREDSHOLD = max_value * THRESHOLD_COLOR;

    if (max_value > THREDSHOLD && max_value == R_value)
    {
        return 'R';
    }
    else if (max_value > THREDSHOLD && max_value == G_value)
    {
        return 'G';
    }
    else if (max_value > THREDSHOLD && max_value == B_value)
    {
        return 'B';
    }
    else
    {
        return 'U';
    }
}

void setup()
{
    // Set up Serial, Servo, TCS3200
    serial_setup(BAUD);
    servo_setup(SERVO_PIN);
    tcs_setup(S0_PIN, S1_PIN, S2_PIN, S3_PIN, OUT_PIN);
}

void loop()
{
}
