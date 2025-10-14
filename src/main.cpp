#include <Arduino.h>
#include <ESP32Servo.h>

// ---------------- ESC Config ----------------
#define SURGE_PIN 41
#define SWAY_PIN 40

#define ESC_NEUTRAL_US   1500
#define ESC_MAX_DELTA_US 500   // 1000–2000 µs range

#define FAILSAFE_TIMEOUT_MS 1000  // return to neutral if no packet

// Servo objects
Servo surgeEsc;
Servo swayEsc;

// ---------------- Variables ----------------
volatile int8_t surge_val = 0;
volatile int8_t sway_val  = 0;
volatile uint32_t last_packet_time = 0;
volatile bool first_packet_received = false;

// ---------------- ESC Functions ----------------
int esc_map(int8_t value) {
    // Map -127..127 to microseconds
    if (value > 127) value = 127;
    if (value < -127) value = -127;
    return ESC_NEUTRAL_US + ((int32_t)value * ESC_MAX_DELTA_US) / 127;
}

// ---------------- ESC Arming & Calibration ----------------
void arm_esc() {
    Serial.println("⏳ Arming ESCs...");

    // Optional: send max → min → neutral to calibrate some ESCs
    surgeEsc.writeMicroseconds(ESC_NEUTRAL_US + ESC_MAX_DELTA_US);
    swayEsc.writeMicroseconds(ESC_NEUTRAL_US + ESC_MAX_DELTA_US);
    delay(1000);

    surgeEsc.writeMicroseconds(ESC_NEUTRAL_US - ESC_MAX_DELTA_US);
    swayEsc.writeMicroseconds(ESC_NEUTRAL_US - ESC_MAX_DELTA_US);
    delay(1000);

    surgeEsc.writeMicroseconds(ESC_NEUTRAL_US);
    swayEsc.writeMicroseconds(ESC_NEUTRAL_US);
    delay(2000);

    Serial.println("✅ ESCs armed");
}

// ---------------- USB Serial Task ----------------
void usb_serial_task(void *pvParameters) {
    for (;;) {
        while (Serial.available() >= 4) {
            uint8_t start = Serial.read();
            int8_t s = Serial.read();
            int8_t w = Serial.read();
            uint8_t stop = Serial.read();

            if (start == 0xAA && stop == 0x55) {
                surge_val = s;
                sway_val  = w;
                last_packet_time = millis();
                first_packet_received = true;

                Serial.printf("➡️ Packet received: Surge=%d, Sway=%d\n", surge_val, sway_val);
            } else {
                Serial.println("⚠️ Invalid packet");
            }
        }

        // Check failsafe
        if (!first_packet_received || (millis() - last_packet_time > FAILSAFE_TIMEOUT_MS)) {
            surgeEsc.writeMicroseconds(ESC_NEUTRAL_US);
            swayEsc.writeMicroseconds(ESC_NEUTRAL_US);
        } else {
            surgeEsc.writeMicroseconds(esc_map(surge_val));
            swayEsc.writeMicroseconds(esc_map(sway_val));
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // 20ms loop → 50Hz PWM update
    }
}

// ---------------- Setup ----------------
void setup() {
    Serial.begin(115200);
    delay(1000); // small delay for USB Serial

    // Attach ESCs to Servo objects
    surgeEsc.attach(SURGE_PIN, 1000, 2000);
    swayEsc.attach(SWAY_PIN, 1000, 2000);

    // Arm ESCs
    arm_esc();

    // Start USB serial FreeRTOS task
    xTaskCreate(usb_serial_task, "usb_serial_task", 4096, NULL, 10, NULL);
}

// ---------------- Loop ----------------
void loop() {
    // Main loop free; FreeRTOS task handles thrusters
}
