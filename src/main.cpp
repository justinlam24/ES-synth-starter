#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <cmath>
#include <HardwareTimer.h>
#include <ES_CAN.h>
#include <STM32FreeRTOS.h>   // Include FreeRTOS for STM32


enum ModuleRole { SENDER, RECEIVER };
ModuleRole moduleRole = SENDER;   // Change to RECEIVER for receiver modules
uint8_t octave = 4;

// ------------------------ GLOBAL STRUCT & GLOBALS ------------------------ //

// Shared system state (used by more than one thread)
struct {
    std::bitset<32> inputs;
    int8_t knob3Rotation; // Store knob position (0 to 8)
    uint8_t RX_Message[8];
    SemaphoreHandle_t mutex;    
} sysState;

// Global variable for the current note step size (accessed by ISR)
uint32_t currentStepSize = 0;

// Phase accumulator for audio generation
static uint32_t phaseAcc = 0;
HardwareTimer sampleTimer(TIM1);

volatile uint8_t TX_Message[8] = {0}; 

// CAN message queues
QueueHandle_t msgInQ;  // Queue for received CAN messages
QueueHandle_t msgOutQ; // Queue for outgoing CAN messages

SemaphoreHandle_t CAN_TX_Semaphore;

// ------------------------- CONSTANTS & PIN DEFINITIONS ------------------------ //

constexpr uint32_t SAMPLE_RATE = 22050; // Audio sample rate (Hz)

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

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

void setOutMuxBit(const uint8_t bitIdx, const bool value) {
    digitalWrite(REN_PIN,LOW);
    digitalWrite(RA0_PIN, bitIdx & 0x01);
    digitalWrite(RA1_PIN, bitIdx & 0x02);
    digitalWrite(RA2_PIN, bitIdx & 0x04);
    digitalWrite(OUT_PIN,value);
    digitalWrite(REN_PIN,HIGH);
    delayMicroseconds(2);
    digitalWrite(REN_PIN,LOW);
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

static std::bitset<16> prevInputs;

// Task to scan the key matrix at a 50ms interval (priority 2)
void scanKeysTask(void * pvParameters) {
#ifdef TEST_SCANKEYS
    for (uint8_t key = 0; key < 12; key++) {
        uint8_t TX_Message[8] = {0};
        TX_Message[0] = 'P';    // Simulate a press event.
        TX_Message[1] = 4;      // Assume octave 4.
        TX_Message[2] = key;    // The key index.
        // Send without blocking (assumes msgOutQ is large enough).
        xQueueSend(msgOutQ, TX_Message, 0);
    }
#else
    const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    (void) pvParameters;  // Unused parameter
    static uint8_t prevKnobState = 0;

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

        // Lock Mutex before modifying sysState
        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        sysState.inputs = localInputs;
        xSemaphoreGive(sysState.mutex);  // Unlock Mutex

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

        // Read knob signals (row 3, columns 0 & 1)
        uint8_t knobState = (localInputs[12] << 1) | localInputs[13];
        if (knobState != prevKnobState) {
            int8_t delta = 0;

            // Decode rotation direction based on state transition
            if ((prevKnobState == 0b00 && knobState == 0b01) ||  // CW
                (prevKnobState == 0b01 && knobState == 0b11) ||
                (prevKnobState == 0b11 && knobState == 0b10) ||
                (prevKnobState == 0b10 && knobState == 0b00)) {
                delta = 1;
            }
            if ((prevKnobState == 0b00 && knobState == 0b10) ||  // CCW
                (prevKnobState == 0b10 && knobState == 0b11) ||
                (prevKnobState == 0b11 && knobState == 0b01) ||
                (prevKnobState == 0b01 && knobState == 0b00)) {
                delta = -1;
            }

            // Update knob rotation with bounds (0 - 8)
            xSemaphoreTake(sysState.mutex, portMAX_DELAY);
            sysState.knob3Rotation = constrain(sysState.knob3Rotation - delta, 0, 8);
            xSemaphoreGive(sysState.mutex);

            prevKnobState = knobState;
        }

        // Compare with previous state to detect key changes
        for (uint8_t i = 0; i < 12; i++) {
            if (localInputs[i] != prevInputs[i]) {
                uint8_t TX_Message[8] = {0};
                TX_Message[0] = localInputs[i] ? 'P' : 'R';  // 'P' for press, 'R' for release
                TX_Message[1] = octave;  // Assume Octave 4 (Modify as needed)
                TX_Message[2] = i;  // Note number (0-11)
                Serial.print("TX_Message: ");
                Serial.write(TX_Message, 8);
                Serial.println();

                xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
            }
            prevInputs[i] = localInputs[i];
        }
    }
#endif
}

void decodeTask(void *pvParameters) {
    uint8_t local_RX_Message[8];  // Local storage for received message

    while (1) {
        xQueueReceive(msgInQ, local_RX_Message, portMAX_DELAY);  // Wait for a message

        Serial.print("Decoding CAN Message: ");
        Serial.write(local_RX_Message, 8);
        Serial.println();

        // Copy to global RX_Message safely
        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        memcpy(sysState.RX_Message, local_RX_Message, sizeof(sysState.RX_Message));
        xSemaphoreGive(sysState.mutex);

        char type = local_RX_Message[0];  // 'P' for Press, 'R' for Release
        uint8_t octave = local_RX_Message[1];
        uint8_t note = local_RX_Message[2];

        if (type == 'P') {  
            uint32_t stepSize = stepSizes[note] << (octave - 4);  // Adjust for octave
            __atomic_store_n(&currentStepSize, stepSize, __ATOMIC_RELAXED);  // Atomic update
        } else if (type == 'R') {  
            __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);  // Stop note
        }
    }
}


// Task to update the display and toggle the LED every 100ms (priority 1)
void displayUpdateTask(void * pvParameters) {
    const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    (void) pvParameters;  // Unused parameter

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(2,10,"RECEIVER");

        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        uint8_t local_RX_Message[8];
        memcpy(local_RX_Message, sysState.RX_Message, sizeof(sysState.RX_Message));
        int8_t volume = sysState.knob3Rotation;
        xSemaphoreGive(sysState.mutex);

        /*u8g2.print(sysState.inputs.to_ulong(), HEX);
        xSemaphoreGive(sysState.mutex);

        u8g2.setCursor(2, 30);
        u8g2.print("Volume: ");
        // Read the knob value using the class method
        u8g2.print(sysState.knob3Rotation);

        uint32_t rxID;
        uint8_t RX_Message[8] = {0};

        while (CAN_CheckRXLevel()) {
            CAN_RX(rxID, RX_Message);
        }

        // Now display the last received CAN message.
        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        u8g2.setCursor(66,30);
        Serial.print("RX: ");
        Serial.println(sysState.RX_Message[0]);
        u8g2.print(sysState.RX_Message[0] == 'P' ? "P" : "R");
        u8g2.print(sysState.RX_Message[1]);
        u8g2.print(sysState.RX_Message[2]);
        xSemaphoreGive(sysState.mutex);

        u8g2.sendBuffer();
        digitalToggle(LED_BUILTIN);
        */

        // Display Volume Level
        char volumeText[10];
        sprintf(volumeText, "Vol: %d", volume);
        u8g2.drawStr(2, 24, volumeText);

        // Display received CAN message
        char canMessageText[20];
        sprintf(canMessageText, "RX: %c %d %d", 
                (local_RX_Message[0] == 'P') ? 'P' : 'R', 
                local_RX_Message[1], 
                local_RX_Message[2]);
        u8g2.drawStr(2, 30, canMessageText);

        u8g2.sendBuffer();
    }
}

// -------------------------------- QUEUE ------------------------------------ //

void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

void CAN_TX_ISR (void) {
    xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

void CAN_TX_Task(void * pvParameters) {
    uint8_t msgOut[8];
    while (1) {
        xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
        xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
        CAN_TX(0x123, msgOut);  // Send the message
    }
}

// ------------------------- TIMER ISR FOR AUDIO ----------------------------- //

void sampleISR() {
    // Optionally, you could do an atomic load if needed:
    // uint32_t step = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);
    uint32_t step = currentStepSize;
    phaseAcc += step;

    int8_t volume = __atomic_load_n(&sysState.knob3Rotation, __ATOMIC_RELAXED);

    // Generate a simple 8-bit sawtooth wave:
    int32_t Vout = (int((phaseAcc >> 24) - 128) >> (8 - volume));
    analogWrite(OUTR_PIN, Vout + 128);
}

// ------------------------- SETUP & LOOP ------------------------------------ //

void setup() {
    Serial.begin(9600);
    Serial.println("Synth Initialized");

    // Initialize Mutex & Default Volume
    sysState.mutex = xSemaphoreCreateMutex();
    sysState.knob3Rotation = 4;

    // Configure Pins
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

    //Initialise display
    setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
    delayMicroseconds(2);
    setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
    u8g2.begin();
    setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

    // Initialize Audio Sample Timer
    sampleTimer.setOverflow(SAMPLE_RATE, HERTZ_FORMAT);
#ifndef DISABLE_ISRS
    sampleTimer.attachInterrupt(sampleISR);
#endif
    sampleTimer.resume();

    CAN_Init(true);
    setCANFilter(0x123, 0x7ff); 
#ifndef DISABLE_ISRS
    CAN_RegisterRX_ISR(CAN_RX_ISR);
    CAN_RegisterTX_ISR(CAN_TX_ISR);
#endif
    CAN_Start();

     // Create CAN Message Queues
     msgInQ = xQueueCreate(36, 8);   // 36 messages, 8 bytes each
     msgOutQ = xQueueCreate(36, 8);
     CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

    // Start FreeRTOS Tasks
    xTaskCreate(scanKeysTask, "scanKeys", 128, NULL, 2, NULL);
    xTaskCreate(displayUpdateTask, "displayUpdate", 256, NULL, 1, NULL);
    xTaskCreate(CAN_TX_Task, "canTX", 128, NULL, 3, NULL);
    xTaskCreate(decodeTask, "decodeTask", 128, NULL, 3, NULL);

#ifdef TEST_SCANKEYS
    uint32_t startTime = micros();
    for (int iter = 0; iter < 32; iter++) {
        scanKeysTask(NULL);
    }
    uint32_t elapsed = micros() - startTime;
    Serial.print("32 iterations of scanKeysTask() took: ");
    Serial.print(elapsed);
    Serial.println(" microseconds");
    while(1);  // Halt after test measurement.
#endif

#ifndef DISABLE_THREADS
    vTaskStartScheduler();
#endif
}


void loop() {
    // Empty. All tasks run under FreeRTOS.
}