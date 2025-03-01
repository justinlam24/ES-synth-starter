#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <cmath>
#include <HardwareTimer.h>
#include <STM32FreeRTOS.h>   // Include FreeRTOS for STM32

// ------------------------ GLOBAL STRUCT & GLOBALS ------------------------ //

// Shared system state (used by more than one thread)
struct {
    std::bitset<32> inputs;  
} sysState;

// Global variable for the current note step size (accessed by ISR)
uint32_t currentStepSize = 0;

// Phase accumulator for audio generation
static uint32_t phaseAcc = 0;
HardwareTimer sampleTimer(TIM1);

// ------------------------- CONSTANTS & PIN DEFINITIONS ------------------------ //

constexpr uint32_t SAMPLE_RATE = 22050; // Audio sample rate (Hz)

// Pin Definitions
const int RA0_PIN  = D3;
const int RA1_PIN  = D6;
const int RA2_PIN  = D12;
const int REN_PIN  = A5;
const int C0_PIN   = A2;
const int C1_PIN   = D9;
const int C2_PIN   = A6;
const int C3_PIN   = D1;
const int OUTR_PIN = A3;

// LED indicator (ensure LED_BUILTIN is defined for your board)
#ifndef LED_BUILTIN
  #define LED_BUILTIN PC13   // Change to the correct LED pin for your board
#endif

// Display driver instance
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

// -------------------------- NOTE CALCULATION ------------------------------- //

// Calculate the phase step size for a given frequency
constexpr uint32_t calculateStepSize(float frequency) {
    return static_cast<uint32_t>((pow(2, 32) * frequency) / SAMPLE_RATE);
}

// Step sizes for the 12 semitones (C to B)
const uint32_t stepSizes[12] = {
    calculateStepSize(261.63f),  // C
    calculateStepSize(277.18f),  // C#
    calculateStepSize(293.66f),  // D
    calculateStepSize(311.13f),  // D#
    calculateStepSize(329.63f),  // E
    calculateStepSize(349.23f),  // F
    calculateStepSize(369.99f),  // F#
    calculateStepSize(392.00f),  // G
    calculateStepSize(415.30f),  // G#
    calculateStepSize(440.00f),  // A
    calculateStepSize(466.16f),  // A#
    calculateStepSize(493.88f)   // B
};

const char* noteNames[12] = {
    "C", "C#", "D", "D#", "E", "F",
    "F#", "G", "G#", "A", "A#", "B"
};

// --------------------------- HELPER FUNCTIONS ------------------------------ //

// Set the row lines on the 3-to-8 decoder based on a row number
void setRow(uint8_t row) {
    digitalWrite(REN_PIN, LOW);
    digitalWrite(RA0_PIN, (row & 0x01) ? HIGH : LOW);
    digitalWrite(RA1_PIN, (row & 0x02) ? HIGH : LOW);
    digitalWrite(RA2_PIN, (row & 0x04) ? HIGH : LOW);
    delayMicroseconds(2);
    digitalWrite(REN_PIN, HIGH);
}

// Read the column values from the 4 inputs (assuming pull-up + key press pulls LOW)
std::bitset<4> readCols() {
    std::bitset<4> cols;
    cols[0] = !digitalRead(C0_PIN);
    cols[1] = !digitalRead(C1_PIN);
    cols[2] = !digitalRead(C2_PIN);
    cols[3] = !digitalRead(C3_PIN);
    return cols;
}

// ----------------------- FREE RTOS TASKS ----------------------------------- //

// Task to scan the key matrix at a 50ms interval (priority 2)
void scanKeysTask(void * pvParameters) {
    const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    (void) pvParameters;  // Unused parameter

    while (1) {
        // Block until 50ms have passed since last execution
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Local copy for key scan result
        std::bitset<32> localInputs;
        for (uint8_t row = 0; row < 4; row++) {
            setRow(row);
            delayMicroseconds(2);
            std::bitset<4> rowInputs = readCols();
            for (uint8_t col = 0; col < 4; col++) {
                localInputs[row * 4 + col] = rowInputs[col];
            }
        }
        sysState.inputs = localInputs;

        // Determine which note (if any) is pressed
        uint32_t localStepSize = 0;
        for (uint8_t i = 0; i < 12; i++) {
            if (localInputs[i]) {
                localStepSize = stepSizes[i];
                break;
            }
        }
        // Update currentStepSize (a 32-bit write on a 32-bit MCU is atomic)
        currentStepSize = localStepSize;
    }
}

// Task to update the display and toggle the LED every 100ms (priority 1)
void displayUpdateTask(void * pvParameters) {
    const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    (void) pvParameters;  // Unused parameter

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Toggle the onboard LED as a real-time indicator
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

        // Update the display with the currently pressed note (if any)
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        for (int i = 0; i < 12; i++) {
            if (sysState.inputs[i]) {
                u8g2.drawStr(0, 12, noteNames[i]);
                break;
            }
        }
        u8g2.sendBuffer();
    }
}

// ------------------------- TIMER ISR FOR AUDIO ----------------------------- //

void sampleISR() {
    // Optionally, you could do an atomic load if needed:
    // uint32_t step = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);
    uint32_t step = currentStepSize;
    phaseAcc += step;
    // Generate a simple 8-bit sawtooth wave:
    int32_t Vout = (int((phaseAcc >> 24) - 128)/8);
    analogWrite(OUTR_PIN, Vout + 128);
}

// ------------------------- SETUP & LOOP ------------------------------------ //

void setup() {
    Serial.begin(9600);
    Serial.println("Synth Initialized");

    // Configure pins
    pinMode(RA0_PIN, OUTPUT);
    pinMode(RA1_PIN, OUTPUT);
    pinMode(RA2_PIN, OUTPUT);
    pinMode(REN_PIN, OUTPUT);
    pinMode(OUTR_PIN, OUTPUT);
    pinMode(C0_PIN, INPUT_PULLUP);
    pinMode(C1_PIN, INPUT_PULLUP);
    pinMode(C2_PIN, INPUT_PULLUP);
    pinMode(C3_PIN, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);

    // Initialize display
    u8g2.begin();

    // Initialize audio sample timer
    sampleTimer.setOverflow(SAMPLE_RATE, HERTZ_FORMAT);
    sampleTimer.attachInterrupt(sampleISR);
    sampleTimer.resume();

    // Create the key scanning task (priority 2, 64 words stack)
    TaskHandle_t scanKeysHandle = NULL;
    xTaskCreate(
        scanKeysTask,     // Task function
        "scanKeys",       // Task name
        64,               // Stack size in words
        NULL,             // Parameter (none)
        2,                // Priority (higher)
        &scanKeysHandle   // Task handle
    );

    // Create the display update task (priority 1, 256 words stack)
    TaskHandle_t displayUpdateHandle = NULL;
    xTaskCreate(
        displayUpdateTask,  // Task function
        "displayUpdate",    // Task name
        256,                // Stack size in words
        NULL,               // Parameter (none)
        1,                  // Priority (lower)
        &displayUpdateHandle // Task handle
    );

    // Start the FreeRTOS scheduler; this function should never return.
    vTaskStartScheduler();
}

void loop() {
    // Empty. All tasks run under FreeRTOS.
}