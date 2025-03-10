/*

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

        Serial.print("Pressed Keys & Knobs: ");
        for (uint8_t key = 0; key < 16; key++) {
            if (localInputs[key]) {
                Serial.print(key);
                Serial.print(" ");
            }
        }
        Serial.println();

        // Read Knob 1 and Knob 2 from Analog Pins
        int knob1Value = analogRead(JOYX_PIN);  // Knob 1 (A1)
        int knob2Value = analogRead(JOYY_PIN);  // Knob 2 (A0)

        // Convert to a range (e.g., 0-8 for Octave Control)
        uint8_t mappedKnob1 = map(knob1Value, 0, 1023, 0, 8);
        uint8_t mappedKnob2 = map(knob2Value, 0, 1023, 0, 8);

        // Debugging printout
        Serial.print("Knob 1: "); Serial.print(mappedKnob1);
        Serial.print(" | Knob 2: "); Serial.println(mappedKnob2);

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

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        uint8_t local_RX_Message[8];
        memcpy(local_RX_Message, sysState.RX_Message, sizeof(sysState.RX_Message));
        int8_t volume = sysState.knob3Rotation;

        u8g2.print(sysState.inputs.to_ulong(), HEX);
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

        // OLED Display
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(2,10,"RECEIVER");

        char volumeBuffer[12];
        sprintf(volumeBuffer, "Volume = %d", sysState.knob3Rotation);
        u8g2.setCursor(2, 20);
        u8g2.print(volumeBuffer);

        // Format the message properly
        char displayBuffer[12];  // Buffer for formatted output
        sprintf(displayBuffer, "%c %d %d", sysState.RX_Message[0], sysState.RX_Message[1], sysState.RX_Message[2]);
        u8g2.drawStr(2, 30, displayBuffer);   // Properly formatted output

        u8g2.sendBuffer();  // Update the display
        xSemaphoreGive(sysState.mutex);

       // u8g2.sendBuffer();
        digitalToggle(LED_BUILTIN);
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

    // Configure pins
    pinMode(RA0_PIN, OUTPUT);
    pinMode(RA1_PIN, OUTPUT);
    pinMode(RA2_PIN, OUTPUT);
    pinMode(REN_PIN, OUTPUT);
    pinMode(OUT_PIN, OUTPUT);
    pinMode(OUTL_PIN, OUTPUT);
    pinMode(OUTR_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(C0_PIN, INPUT);
    pinMode(C1_PIN, INPUT);
    pinMode(C2_PIN, INPUT);
    pinMode(C3_PIN, INPUT);
    pinMode(JOYX_PIN, INPUT);
    pinMode(JOYY_PIN, INPUT);

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
    xTaskCreate(CAN_TX_Task, "CAN_TX_Task", 128, NULL, 1, NULL);
    xTaskCreate(decodeTask, "decodeTask", 128, NULL, 1, NULL);

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

*/

#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <cmath>
#include <ES_CAN.h>
#include <STM32FreeRTOS.h>   // Include FreeRTOS for STM32


// Uncomment the following lines for test builds:
//#define DISABLE_THREADS
//#define DISABLE_ISRS
//#define TEST_SCANKEYS

enum ModuleRole { SENDER, RECEIVER };
ModuleRole moduleRole = SENDER;   // Change to RECEIVER for receiver modules

// Global variable for choosing the octave number for this module.
uint8_t moduleOctave = 4;         // Default octave number (can be changed at runtime)


// ------------------------- Knob Class -------------------------------------- //

class Knob {
    public:
        // Constructor: initialise limits, starting value, and internal state.
        Knob(int lower = 0, int upper = 8)
            : lowerLimit(lower), upperLimit(upper), rotation(lower), knobPrev(0), lastLegalDelta(0)
        {
        }
    
        // Update the knob's rotation value using the new quadrature state (2-bit value)
        void update(uint8_t quadratureState) {
            int delta = 0;
            uint8_t diff = knobPrev ^ quadratureState;
            if (diff == 0) {
                // No change in the state
                delta = 0;
            }
            else if (diff == 0b11) {
                // Both bits changed: an impossible transition.
                // Assume the same direction as the last legal transition.
                delta = (lastLegalDelta != 0) ? lastLegalDelta : 0;
            }
            else {
                // Legal transition (only one bit changed).
                uint8_t transition = (knobPrev << 2) | quadratureState;
                switch (transition) {
                    case 0b0001: // 00 -> 01: Clockwise => +1
                        delta = +1;
                        lastLegalDelta = +1;
                        break;
                    case 0b0100: // 01 -> 00: Anticlockwise => -1
                        delta = -1;
                        lastLegalDelta = -1;
                        break;
                    case 0b1011: // 10 -> 11: Anticlockwise => -1
                        delta = -1;
                        lastLegalDelta = -1;
                        break;
                    case 0b1110: // 11 -> 10: Clockwise => +1
                        delta = +1;
                        lastLegalDelta = +1;
                        break;
                    default:
                        // For intermediate or inconclusive transitions, make no change.
                        delta = 0;
                        break;
                }
            }
            // Save the current state for the next update.
            knobPrev = quadratureState;
    
            // Read the current rotation value atomically.
            int current = __atomic_load_n(&rotation, __ATOMIC_RELAXED);
            int newVal = current + delta;
            // Clamp to the permitted limits.
            if (newVal < lowerLimit) newVal = lowerLimit;
            if (newVal > upperLimit) newVal = upperLimit;
            // Atomically store the new rotation value.
            __atomic_store_n(&rotation, newVal, __ATOMIC_RELAXED);
        }
    
        // Get the current knob rotation (thread-safe atomic load)
        int getRotation() const {
            return __atomic_load_n(&rotation, __ATOMIC_RELAXED);
        }
    
        // Set new lower and upper limits, and adjust the current value if needed.
        void setLimits(int lower, int upper) {
            lowerLimit = lower;
            upperLimit = upper;
            int current = getRotation();
            if (current < lowerLimit) current = lowerLimit;
            if (current > upperLimit) current = upperLimit;
            __atomic_store_n(&rotation, current, __ATOMIC_RELAXED);
        }
    
    private:
        int rotation;      // Current rotation value (atomically accessed)
        int lowerLimit;    // Minimum allowed value
        int upperLimit;    // Maximum allowed value
    
        // Internal state for quadrature decoding:
        uint8_t knobPrev;      // Previous 2-bit state {B, A}
        int lastLegalDelta;    // Last legal delta (+1 or -1)
    };


// ------------------------ GLOBAL STRUCT & GLOBALS ------------------------ //

// Shared system state (used by more than one thread)
struct {
    std::bitset<32> inputs;
    SemaphoreHandle_t mutex;  
    int knob3Rotation;
    Knob knob3;
    Knob knob2;
    Knob knob1;
    uint8_t RX_Message[8];
    } sysState;


// Global variable for the current note step size (accessed by ISR)
uint32_t currentStepSize = 0;

// Phase accumulator for audio generation
static uint32_t phaseAcc = 0;
HardwareTimer sampleTimer(TIM1);

volatile uint8_t TX_Message[8] = {0}; 

QueueHandle_t msgInQ;
#ifdef TEST_SCANKEYS
  // Increase the queue size in test mode to avoid blocking.
  QueueHandle_t msgOutQ;  // We’ll create a larger queue below.
#else
  QueueHandle_t msgOutQ;
#endif

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
    cols[0] = digitalRead(C0_PIN);
    cols[1] = digitalRead(C1_PIN);
    cols[2] = digitalRead(C2_PIN);
    cols[3] = digitalRead(C3_PIN);
    return cols;
}


    



// ----------------------- FREE RTOS TASKS ----------------------------------- //

// In your global variables, change the previous state bitset to track 16 keys:
static std::bitset<16> prevKeys;  // Now tracks keys 0-15

// Task to scan the key matrix at a 20-50ms interval (priority 2)
void scanKeysTask(void * pvParameters) {
#ifdef TEST_SCANKEYS
    // In test mode, we simulate a worst-case scenario:
    // For every call, generate a key press message for each of the 12 keys.
    for (uint8_t key = 0; key < 12; key++) {
        uint8_t TX_Message[8] = {0};
        TX_Message[0] = 'P';
        TX_Message[1] = 4;
        TX_Message[2] = key;
        // Send only if in SENDER mode.
        if (moduleRole == SENDER) {
            xQueueSend(msgOutQ, TX_Message, 0);
        }
    }
#else
    const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();


    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 1) Scan the full 8x4 matrix into localInputs (16 keys)
        std::bitset<32> localInputs;
        for (uint8_t row = 0; row < 8; row++) {
            setRow(row);
            delayMicroseconds(2);
            std::bitset<4> rowInputs = readCols();
            for (uint8_t col = 0; col < 4; col++) {
                localInputs[row * 4 + col] = rowInputs[col];
            }
        }

        // 2) Update shared key state (if needed)
        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        sysState.inputs = localInputs;
        xSemaphoreGive(sysState.mutex);

        // 3) Process note keys (first 12 keys, rows 0-2) as before
        uint32_t localStepSize = 0;
        for (uint8_t i = 0; i < 12; i++) {
            if (!localInputs[i]) {
                localStepSize = stepSizes[i];
                break;
            }
        }
        currentStepSize = localStepSize;
        // Process note press/release events for keys 0-11:
        uint8_t currentOctave = moduleOctave;
        for (uint8_t key = 0; key < 12; key++) {
            bool currentState = !localInputs[key];
            bool previousState = prevKeys[key];
            if (currentState != previousState) {
                // Send messages only in SENDER mode.
                if (moduleRole == SENDER) {
                    uint8_t TX_Message[8] = {0};
                    TX_Message[0] = currentState ? 'P' : 'R';
                    TX_Message[1] = currentOctave;
                    TX_Message[2] = key;
                    xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
                }
            }
            prevKeys[key] = currentState;
        }
        

        // 4) Decode knob 3 (remains unchanged)
        uint8_t knob3A = localInputs[3 * 4 + 0];
        uint8_t knob3B = localInputs[3 * 4 + 1];
        uint8_t knob3Curr = (knob3B << 1) | knob3A;  // Quadrature state {B, A}
        sysState.knob3.update(knob3Curr);

        // 5) Decode knob 2 (remains unchanged)
        uint8_t knob2A = localInputs[3 * 4 + 2];
        uint8_t knob2B = localInputs[3 * 4 + 3];
        uint8_t knob2Curr = (knob2B << 1) | knob2A;  // Quadrature state {B, A}
        sysState.knob2.update(knob2Curr);

        // 6) Decode knob 1 (remains unchanged)
        uint8_t knob1A = localInputs[4 * 4 + 0];
        uint8_t knob1B = localInputs[4 * 4 + 1];
        uint8_t knob1Curr = (knob1B << 1) | knob1A;  // Quadrature state {B, A}
        sysState.knob1.update(knob1Curr);
        moduleRole = (sysState.knob1.getRotation() % 2) ? SENDER : RECEIVER;

        if (!localInputs[4*5 + 0]){
            Serial.println("Knob 2S pressed");
        } else if (!localInputs[4*5 + 1]){
            Serial.println("Knob 3S pressed");
        } else if (!localInputs[4*5 + 2]){
            Serial.println("Joystick S pressed");
        } else if (!localInputs[4*6 + 0]){
            Serial.println("Knob 0S pressed");
        } else if (!localInputs[4*6 + 1]){
            Serial.println("Knob 1S pressed");
        }

    }
#endif
}

// Task to update the display and poll for received CAN messages (priority 1)
void displayUpdateTask(void * pvParameters) {
    const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

        // Read sysState.knob3Rotation under the mutex
        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        int rotationCopy = sysState.knob3.getRotation();
        xSemaphoreGive(sysState.mutex);

        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(2,10,moduleRole == SENDER ? "SENDER" : "RECEIVER");

        // Display current JOYX and JOYY values in the form (12,34)
        u8g2.setCursor(75, 10);
        u8g2.print("(");
        u8g2.print(analogRead(JOYX_PIN));
        u8g2.print(",");
        u8g2.print(analogRead(JOYY_PIN));
        u8g2.print(")");
    

        // Display key matrix state
        u8g2.setCursor(2,20);
        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        u8g2.print(sysState.inputs.to_ulong(), HEX);
        xSemaphoreGive(sysState.mutex);

        u8g2.setCursor(2, 30);
        u8g2.print("Knob: ");
        // Read the knob value using the class method
        u8g2.print(rotationCopy);

        u8g2.setCursor(66, 20);
        u8g2.print("Octave: ");
        u8g2.print(moduleOctave);


        // Now display the last received CAN message.
        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        u8g2.setCursor(66,30);
        // Serial.print("RX: ");
        // for (uint8_t i = 0; i < 8; i++) {
        //     Serial.print(sysState.RX_Message[i], HEX);
        //     Serial.print(" ");
        // }
        Serial.println();
        u8g2.print(sysState.RX_Message[0] == 'P' ? "P" : "R");
        u8g2.print(sysState.RX_Message[1]);
        u8g2.print(sysState.RX_Message[2]);
        xSemaphoreGive(sysState.mutex);

        u8g2.sendBuffer();
        digitalToggle(LED_BUILTIN);
    }
}

// -------------------------- DECODE TASK  ----------------------------------- //

void decodeTask(void * pvParameters) {
    uint8_t localMsg[8];
    for (;;) {
        // Block until a message is available:
        if (xQueueReceive(msgInQ, localMsg, portMAX_DELAY) == pdPASS) {
            if (localMsg[0] == 'R') {  // Release message: set step size to 0.
                __atomic_store_n(&currentStepSize, (uint32_t)0, __ATOMIC_RELAXED);
            }
            else if (localMsg[0] == 'P') {  // Press message: convert note and octave.
                uint8_t octave = localMsg[1];
                uint8_t note = localMsg[2];
                if (note < 12) {
                    uint32_t step = stepSizes[note];
                    if (octave > 4) {
                        step = step << (octave - 4);
                    } else if (octave < 4) {
                        step = step >> (4 - octave);
                    }
                    __atomic_store_n(&currentStepSize, step, __ATOMIC_RELAXED);
                }
            }
            // Update the global RX_Message
            xSemaphoreTake(sysState.mutex, portMAX_DELAY);
            memcpy(sysState.RX_Message, localMsg, sizeof(sysState.RX_Message));
            xSemaphoreGive(sysState.mutex);
        }
    }
}



void CAN_TX_Task (void * pvParameters) {
	// If not SENDER, suspend this task.
    if (moduleRole != SENDER) {
        while (1) { vTaskDelay(portMAX_DELAY); }
    }
    uint8_t msgOut[8];
    while (1) {
        xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
        xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
        CAN_TX(0x123, msgOut);
    }
}


// ------------------------- TIMER ISR FOR AUDIO ----------------------------- //

void sampleISR() {
    if (moduleRole == SENDER) {
        // Do not generate audio in sender mode.
        return;
    }
    
    // Calculate effective step size based on moduleOctave.
    // (Assuming 4 is the base octave.)
    uint32_t effectiveStep = currentStepSize;
    if (moduleOctave > 4) {
        effectiveStep = effectiveStep << (moduleOctave - 4);
    } else if (moduleOctave < 4) {
        effectiveStep = effectiveStep >> (4 - moduleOctave);
    }
    
    phaseAcc += effectiveStep;
    int32_t Vout = (phaseAcc >> 24) - 128;

    // Optionally update moduleOctave based on knob2 (overriding moduleOctave if desired)
    int knobOctave = sysState.knob2.getRotation();
    if (knobOctave < 0) knobOctave = 0;
    if (knobOctave > 8) knobOctave = 8;
    // Decide whether to use knobOctave or moduleOctave for the frequency scaling:
    moduleOctave = knobOctave;

    // Read the volume (from knob3) and apply log taper volume control:
    int volume = sysState.knob3.getRotation();
    if (volume < 0) volume = 0;
    if (volume > 8) volume = 8;
    Vout = Vout >> (8 - volume);

    analogWrite(OUTR_PIN, Vout + 128);
}

void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL); // Send the received message to the queue
}

void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}


// ------------------------- SETUP & LOOP ------------------------------------ //

void setup() {
    Serial.begin(9600);
#ifdef TEST_SCANKEYS
    delay(3000);
#endif
    Serial.println("Synth Initialized");

    // Configure pins
    pinMode(RA0_PIN, OUTPUT);
    pinMode(RA1_PIN, OUTPUT);
    pinMode(RA2_PIN, OUTPUT);
    pinMode(REN_PIN, OUTPUT);
    pinMode(OUT_PIN, OUTPUT);
    pinMode(OUTL_PIN, OUTPUT);
    pinMode(OUTR_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(C0_PIN, INPUT);
    pinMode(C1_PIN, INPUT);
    pinMode(C2_PIN, INPUT);
    pinMode(C3_PIN, INPUT);
    pinMode(JOYX_PIN, INPUT);
    pinMode(JOYY_PIN, INPUT);

    //Initialise display
    setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
    delayMicroseconds(2);
    setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
    u8g2.begin();
    setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply
    
    
    // Initialize audio sample timer
    sampleTimer.setOverflow(SAMPLE_RATE, HERTZ_FORMAT);
//#ifndef DISABLE_ISRS
    sampleTimer.attachInterrupt(sampleISR);
//#endif
    sampleTimer.resume();
    
    CAN_Init(true);
    setCANFilter(0x123, 0x7ff); 
//#ifndef DISABLE_ISRS
    CAN_RegisterRX_ISR(CAN_RX_ISR);
    CAN_RegisterTX_ISR(CAN_TX_ISR);
//#endif
    CAN_Start();
    
    sysState.mutex = xSemaphoreCreateMutex();

    msgInQ = xQueueCreate(36, 8);
#ifdef TEST_SCANKEYS
    msgOutQ = xQueueCreate(384, 8);  // Larger queue for test iterations.
#else
    msgOutQ = xQueueCreate(36, 8);
#endif
    CAN_TX_Semaphore = xSemaphoreCreateCounting(3, 3);

#ifndef DISABLE_THREADS
    Serial.print("modulerole: ");
    Serial.println(moduleRole);
    TaskHandle_t scanKeysHandle = NULL;
    xTaskCreate(scanKeysTask, "scanKeys", 64, NULL, 2, &scanKeysHandle);
    
    TaskHandle_t displayUpdateHandle = NULL;
    xTaskCreate(displayUpdateTask, "displayUpdate", 256, NULL, 1, &displayUpdateHandle);
    
    // Always create decodeTask so that received messages are processed.
    TaskHandle_t decodeTaskHandle = NULL;
    xTaskCreate(decodeTask, "decodeTask", 128, NULL, 1, &decodeTaskHandle);

    if (moduleRole == SENDER) {
        TaskHandle_t CAN_TX_Handle = NULL;
        xTaskCreate(CAN_TX_Task, "CAN_TX_Task", 128, NULL, 1, &CAN_TX_Handle);
    }

#endif

#ifdef TEST_SCANKEYS
    // In test mode, execute the scanKeysTask 32 times (without starting the scheduler)
        // Flush the transmit queue before timing.
    xQueueReset(msgOutQ);
    uint32_t startTime = micros();
    for (int iter = 0; iter < 32; iter++) {
        scanKeysTask(NULL);
    }
    uint32_t elapsed = micros() - startTime;
    Serial.print("32 iterations of scanKeysTask() took: ");
    Serial.print(elapsed);
    Serial.println(" microseconds");
    while(1);
#endif

#ifndef DISABLE_THREADS
    // Start the FreeRTOS scheduler; this should never return in normal operation.
    vTaskStartScheduler();
#endif
}

void loop() {
    // Empty. All tasks run under FreeRTOS.    

}