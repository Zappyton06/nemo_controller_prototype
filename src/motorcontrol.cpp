#include <Arduino.h>
#include <ESP32Servo.h>

#define FR_MOTOR 13
#define FL_MOTOR 18
#define AR_MOTOR 16
#define AL_MOTOR 19

#define FHR_MOTOR 17
#define FHL_MOTOR 14
#define AHR_MOTOR 14
#define AHL_MOTOR 21

#define ESC_NEUTRAL_US   1500
#define ESC_MAX_DELTA_US 400   // 1000–2000 µs range
#define FAILSAFE_TIMEOUT_MS 1200  // return to neutral if no packet

uint8_t thrusterList[8] = {FR_MOTOR,FL_MOTOR,AR_MOTOR,AL_MOTOR,FHR_MOTOR,FHL_MOTOR,AHR_MOTOR,AHL_MOTOR};

Servo Thruster[8];

volatile int8_t surge_val = 0;
volatile int8_t heave_val  = 0;
volatile uint32_t last_packet_time = 0;
volatile bool first_packet_received = false;

int esc_map(int8_t value) {
    if (value > -5 && value < 5) value = 0;
    if (value > 100) value = 100;
    if (value < -100) value = -100;
    value = value * 4;

    return value;
}

void multiArmMotors(){
    for(int i = 0; i < 8; i++){
        Thruster[i].attach(thrusterList[i],1100,1900);
        Thruster[i].writeMicroseconds(ESC_NEUTRAL_US);
    }
}

void neutralMotors(){
    for(int i = 0; i < 8; i++){
        Thruster[i].writeMicroseconds(ESC_NEUTRAL_US);
    }
}

void surgeMotors(int value){
    Thruster[0].writeMicroseconds(1500 + esc_map(value));
    Thruster[1].writeMicroseconds(1500 + esc_map(value));
    Thruster[2].writeMicroseconds(1500 - esc_map(value));
    Thruster[3].writeMicroseconds(1500 - esc_map(value));
}

void heaveMotors(int value){
    Thruster[4].writeMicroseconds(1500 + esc_map(value));
    Thruster[5].writeMicroseconds(1500 - esc_map(value));
    Thruster[6].writeMicroseconds(1500 - esc_map(value));
    Thruster[7].writeMicroseconds(1500 + esc_map(value));
}

void usb_serial_task(void *pvParameters) {
    for (;;) {
        while (Serial.available() >= 4) {
            uint8_t start = Serial.read();
            int8_t s = Serial.read();
            int8_t w = Serial.read();
            uint8_t stop = Serial.read();

            if (start == 0xAA && stop == 0x55){
                surge_val = s;
                heave_val  = w;
                last_packet_time = millis();
                first_packet_received = true;
            }
        }

        if (!first_packet_received || (millis() - last_packet_time > FAILSAFE_TIMEOUT_MS)) {
            neutralMotors();
        } else {
            surgeMotors(surge_val);
            heaveMotors(heave_val);
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // 20ms loop → 50Hz PWM update
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    multiArmMotors();
    xTaskCreate(usb_serial_task, "usb_serial_task", 4096, NULL, 10, NULL);
}

void loop() {

}
