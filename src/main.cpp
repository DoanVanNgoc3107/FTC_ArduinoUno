/**
 * @author Doan Van Ngoc - 104317 - DTD64CL
 * @brief Color Detection and Servo Control
 * @teacher Ths. Vu Thi Thu
 * @version 1.0
 * @date 2025-11-14
 * @copyright DOAN VAN NGOC (Don't use without permission, thank you!)
 */

#include <Arduino.h>
#include <Servo.h>

// Khởi tạo đối tương Servo
Servo servo_checked; // Servo này sẽ chịu nhiệm vụ đưa sản phẩm vào vị trí
Servo servo_keeper; // Servo giữ sản phẩm đợi servo_move gạt thang sang vị trí đã định
Servo servo_move; // Servo gạt thang sang vị trí thùng chứa sản phầm theo màu sắc đã định

// ==== CONSTANTS ====
// Define Servo
#define SERVO_PIN 9 // Controller pin of servo

// Define Serial
#define BAUD 9600 // Baud rate of serial

// TCS3200
#define S0_PIN 6
#define S1_PIN 3
#define S2_PIN 4
#define S3_PIN 5
#define OUT_PIN 2

#define TIME_PLUSE_COUNT 500      // Thời gian đếm xung (ms)
#define DELAY_READ_COLOR 100      // Thời gian chờ giữa các lần đọc màu (ms)
#define THRESHOLD_COLOR 0.9       // Ngưỡng để phân biệt màu sắc (giá trị tùy chỉnh)
#define THRESHOLD_COLOR_LIMIT 150 // Ngưỡng giới hạn để xác định màu sắc (giá trị tùy chỉnh)
#define SERVO_STEP 3         // degrees per update when stepping
#define SERVO_STEP_DELAY 5   // ms between step updates (was 20)


int R_value, B_value, G_value;

// ==== GLOBAL VARIABLES ====
volatile unsigned long pluse = 0; // GLOBAL - pluse count from TCS3200

unsigned long lastTime = 0; // GLOBAL - Last time
int lastDegree = 0; // GLOBAL - vị trí hiện tại của servo
int targetDegree = 0; // GLOBAL - vị trí mục tiêu

// ==== Function Utilities ====
/**
 * @brief This function checks if a specified delay time has elapsed since the last recorded time.
 * If the delay time has elapsed, it updates the start time to the current time and returns true.
 * Otherwise, it returns false.
 * @param start
 * @param delayTime
 * @return bool
 */
bool checkTimeElapsed(unsigned long& start, unsigned long delayTime)
{
    unsigned long now = millis();
    if (now - start >= delayTime)
    {
        start = now;
        return true;
    }
    return false;
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
}

/**
 * @brief Hàm này dùng để lấy vị trí hiện tại của servo
 * @return lastDegree (int) : vị trí hiện tại của servo
 */
int getDegCurrent()
{
    return lastDegree;
}

/**
 * @brief Hàm này dùng để di chuyển servo đến vị trí mong muốn
  * @param degree (int) : vị trí mong muốn (0-180)
 * @param immediate (bool) : nếu true thì di chuyển ngay lập tức, nếu false thì di chuyển từ từ
 */
void servo_active(int const degree, bool const immediate = false)
{
    if (degree < 0 || degree > 180)
    {
        Serial.println("Degree invalid (0–180), received: " + String(degree));
        return;
    }

    // Nếu vị trí mục tiêu khác với vị trí hiện tại thì cập nhật
    if (targetDegree != degree)
    {
        targetDegree = degree;
        Serial.println("New target set: " + String(degree) + " degrees");

        // Nếu di chuyển ngay lập tức thì cập nhật vị trí hiện tại luôn
        if (immediate)
        {
            lastDegree = degree;
            servo.write(degree);
            Serial.println("Servo moved immediate to: " + String(degree));
            lastTime = millis(); // reset timing
        }
    }
}

/**
 * @brief This function moves servo based on color input
  'R' -> degree_s1
  'G' -> degree_s2
  'B' -> degree_s3
 * @param color - 'R', 'G', 'B', 'U' (unknown)
 * @param degree_s1 - degree for 'R'
 * @param degree_s2 - degree for 'G'
 * @param degree_s3 - degree for 'B'
 * @param degree_none - degree for 'U'
 * @param immediate - if true, move immediately; if false, move gradually
 */
void servo_service(char const color, int const degree_s1, int const degree_s2, int const degree_s3,
                   int const degree_none, bool const immediate = false)
{
    switch (color)
    {
    case 'R':
        servo_active(degree_s1, immediate);
        break;
    case 'G':
        servo_active(degree_s2, immediate);
        break;
    case 'B':
        servo_active(degree_s3, immediate);
        break;
    case 'U':
        servo_active(degree_none);
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

    if (lastDegree != targetDegree && currentTime - lastTime > SERVO_STEP_DELAY)
    {
        lastTime = currentTime;

        if (lastDegree < targetDegree)
        {
            lastDegree = min(lastDegree + SERVO_STEP, targetDegree);
        }
        else
        {
            lastDegree = max(lastDegree - SERVO_STEP, targetDegree);
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
void tcs_update()
{
    // RED : S2 = LOW, S3 = LOW
    R_value = read_color_frequency(LOW, LOW);
    delay(DELAY_READ_COLOR);

    // GREEN : S2 = HIGH, S3 = HIGH
    G_value = read_color_frequency(HIGH, HIGH);
    delay(DELAY_READ_COLOR);

    // BLUE : S2 = LOW, S3 = HIGH
    B_value = read_color_frequency(LOW, HIGH);
}

bool checkProduct()
{
    return true;
}

/**
 * @brief Hàm này dùng để xác định màu sắc dựa trên giá trị RGB đọc được từ cảm biến TCS3200
  'R' -> Red
  'G' -> Green
  'B' -> Blue
  'N' -> No color (all values below threshold)
  'U' -> Unknown / ambiguous
 * @note THRESHOLD_COLOR_LIMIT và THRESHOLD_COLOR có thể được điều chỉnh để phù hợp với điều kiện môi trường cụ thể
 * @return @note char : ký tự đại diện cho màu sắc
 */
char getColorString()
{
    // If overall signal too low => No color
    if (R_value < THRESHOLD_COLOR_LIMIT && G_value < THRESHOLD_COLOR_LIMIT && B_value < THRESHOLD_COLOR_LIMIT)
    {
        return 'N';
    }

    int maxVal = max(R_value, max(G_value, B_value));
    int secondVal = 0;

    // compute second largest
    if (maxVal == R_value) secondVal = max(G_value, B_value);
    else if (maxVal == G_value) secondVal = max(R_value, B_value);
    else secondVal = max(R_value, G_value);

    // Avoid division by zero
    if (secondVal == 0)
    {
        if (maxVal == R_value) return 'R';
        if (maxVal == G_value) return 'G';
        return 'B';
    }

    // Interpret THRESHOLD_COLOR (0 < THRESHOLD_COLOR < 1) as how strict dominance must be.
    // Require: maxVal >= secondVal / THRESHOLD_COLOR  (e.g. THRESHOLD_COLOR=0.9 => need ~1.11x)
    float required = static_cast<float>(secondVal) / max(0.0001f, static_cast<float>(THRESHOLD_COLOR));

    if (maxVal >= required)
    {
        if (maxVal == R_value) return 'R';
        if (maxVal == G_value) return 'G';
        return 'B';
    }

    return 'U'; // Unknown / ambiguous
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
    // Update TCS3200 readings
    tcs_update();

    // Get detected color
    char color = getColorString();
    Serial.println(
        "Detected Color: " + String(color) + " | R: " + String(R_value) + " G: " + String(G_value) + " B: " +
        String(B_value));

    // Ví dụ : khi đọc được màu đỏ thì servo sẽ tự gạt sản phầm sang tay trái góc 0 độ
    // khi đọc được màu xanh lá thì servo sẽ gạt sản phẩm sang tay phải góc 90 độ
    // khi đọc được màu xanh dương thì servo sẽ gạt sản phẩm sang tay phải góc 180 độ
    servo_service(color, 40, 90, 180, 0, true);
    servo_update();
}
