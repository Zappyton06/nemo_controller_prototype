#include <Arduino.h>
#include <ESP32Servo.h>

// ---------------- Pin Configuration ----------------
#define FR_MOTOR 13
#define FL_MOTOR 18
#define AR_MOTOR 15
#define AL_MOTOR 16

#define FHR_MOTOR 17
#define FHL_MOTOR 14
#define AHR_MOTOR 19
#define AHL_MOTOR 21

// ---------------- ESC Constants ----------------
#define ESC_NEUTRAL_US    1500
#define ESC_MAX_DELTA_US  400   // ±400 µs (1100–1900 µs range)
#define FAILSAFE_TIMEOUT_MS 1200  // return to neutral if no packet

#define MAX3(a,b,c) ((a) > (b) ? ((a) > (c) ? (a) : (c)) : ((b) > (c) ? (b) : (c)))

// ---------------- Globals ----------------
uint8_t thrusterList[8] = {FR_MOTOR, FL_MOTOR, AR_MOTOR, AL_MOTOR,
                           FHR_MOTOR, FHL_MOTOR, AHR_MOTOR, AHL_MOTOR};

Servo Thruster[8];

volatile int8_t surge_val = 0;
volatile int8_t sway_val  = 0;
volatile int8_t yaw_val   = 0;
volatile int8_t heave_val = 0;
volatile bool first_packet_received = false;
volatile uint32_t last_packet_time = 0;

// FreeRTOS objects
SemaphoreHandle_t controlMutex;
SemaphoreHandle_t newPacketSem;

// ---------------- Utility ----------------
int esc_map(int8_t value) {
    if (value > -5 && value < 5) value = 0;  // deadzone
    if (value > 100) value = 100;
    if (value < -100) value = -100;
    return map(value, -100, 100, -ESC_MAX_DELTA_US, ESC_MAX_DELTA_US);
}

void multiArmMotors() {
    for (int i = 0; i < 8; i++) {
        Thruster[i].attach(thrusterList[i], 1100, 1900);
        Thruster[i].writeMicroseconds(ESC_NEUTRAL_US);
    }
}

void neutralMotors() {
    for (int i = 0; i < 8; i++) {
        Thruster[i].writeMicroseconds(ESC_NEUTRAL_US);
    }
}

// ---------------- Motor Mixers ----------------
void surgeMotors(int value) {
    int pwm = esc_map(value);
    Thruster[0].writeMicroseconds(1500 + pwm);
    Thruster[1].writeMicroseconds(1500 + pwm);
    Thruster[2].writeMicroseconds(1500 - pwm);
    Thruster[3].writeMicroseconds(1500 - pwm);
}

void swayMotors(int value) {
    int pwm = esc_map(value);
    Thruster[0].writeMicroseconds(1500 - pwm);
    Thruster[1].writeMicroseconds(1500 + pwm);
    Thruster[2].writeMicroseconds(1500 - pwm);
    Thruster[3].writeMicroseconds(1500 + pwm);
}

void yawMotors(int value) {
    int pwm = esc_map(value);
    Thruster[0].writeMicroseconds(1500 - pwm);
    Thruster[1].writeMicroseconds(1500 + pwm);
    Thruster[2].writeMicroseconds(1500 + pwm);
    Thruster[3].writeMicroseconds(1500 - pwm);
}

void heaveMotors(int value) {
    int pwm = esc_map(value);
    Thruster[4].writeMicroseconds(1500 + pwm);
    Thruster[5].writeMicroseconds(1500 - pwm);
    Thruster[6].writeMicroseconds(1500 - pwm);
    Thruster[7].writeMicroseconds(1500 + pwm);
}

// ---------------- FreeRTOS Tasks ----------------

// Task 1: Read Serial and Parse Commands
void usb_serial_task(void *pvParameters) {
    for (;;) {
        if (Serial.available() >= 6) {
            uint8_t start = Serial.read();
            int8_t s = Serial.read();
            int8_t w = Serial.read();
            int8_t y = Serial.read();
            int8_t h = Serial.read();
            uint8_t stop = Serial.read();

            if (start == 0xAA && stop == 0x55) {
                if (xSemaphoreTake(controlMutex, portMAX_DELAY)) {
                    surge_val = s;
                    sway_val  = w;
                    yaw_val   = y;
                    heave_val = h;
                    last_packet_time = millis();
                    first_packet_received = true;
                    xSemaphoreGive(controlMutex);
                }
                xSemaphoreGive(newPacketSem);  // Signal new data
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // small delay to yield CPU
    }
}

// Task 2: Motor Control (triggered by newPacketSem)
void pwm_update_task(void *pvParameters) {
    for (;;) {
        // Wait for new command or timeout
        if (xSemaphoreTake(newPacketSem, pdMS_TO_TICKS(FAILSAFE_TIMEOUT_MS)) == pdTRUE) {
            // Got new packet
            int8_t surge, sway, yaw, heave;
            uint32_t last_time;
            bool valid;

            if (xSemaphoreTake(controlMutex, portMAX_DELAY)) {
                surge = surge_val;
                sway = sway_val;
                yaw = yaw_val;
                heave = heave_val;
                last_time = last_packet_time;
                valid = first_packet_received;
                xSemaphoreGive(controlMutex);
            }

            if (valid && (millis() - last_time <= FAILSAFE_TIMEOUT_MS)) {
                heaveMotors(heave);
                int max_val = MAX3(abs(surge), abs(sway), abs(yaw));
                if (max_val == abs(surge))
                    surgeMotors(surge);
                else if (max_val == abs(sway))
                    swayMotors(sway);
                else if (max_val == abs(yaw))
                    yawMotors(yaw);
            } else {
                neutralMotors();
            }
        } else {
            // Timeout → no command received
            neutralMotors();
        }
    }
}

// ---------------- Setup ----------------
void setup() {
    Serial.begin(115200);
    delay(1000);
    multiArmMotors();

    controlMutex = xSemaphoreCreateMutex();
    newPacketSem = xSemaphoreCreateBinary();

    // Create tasks
    xTaskCreatePinnedToCore(usb_serial_task, "usb_serial_task", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(pwm_update_task, "pwm_update_task", 4096, NULL, 10, NULL, 0);
}

void loop() {
    // Unused; tasks handle everything
}
