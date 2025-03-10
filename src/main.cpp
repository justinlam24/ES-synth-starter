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

volatile int joyX12Val = 6;  // Default mid value (0 to 12)
volatile int joyY12Val = 6;  // Default mid value (0 to 12)



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
    Knob knob0;
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
static bool prevKnob0SPressed = false;

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

    static int prevTranspose = 0;

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
        uint8_t knob3A = localInputs[12];
        uint8_t knob3B = localInputs[13];
        uint8_t knob3Curr = (knob3B << 1) | knob3A;  // Quadrature state {B, A}
        sysState.knob3.update(knob3Curr);

        // 5) Decode knob 2 (remains unchanged)
        uint8_t knob2A = localInputs[14];
        uint8_t knob2B = localInputs[15];
        uint8_t knob2Curr = (knob2B << 1) | knob2A;  // Quadrature state {B, A}
        sysState.knob2.update(knob2Curr);

        // 6) Decode knob 1 (remains unchanged)
        uint8_t knob1A = localInputs[16];
        uint8_t knob1B = localInputs[17];
        uint8_t knob1Curr = (knob1B << 1) | knob1A;  // Quadrature state {B, A}
        sysState.knob1.update(knob1Curr);

        // 7) Decode knob 0 (remains unchanged)
        uint8_t knob0A = localInputs[18];
        uint8_t knob0B = localInputs[19];
        uint8_t knob0Curr = (knob0B << 1) | knob0A;  // Quadrature state {B, A}
        prevTranspose = sysState.knob0.getRotation();
        sysState.knob0.update(knob0Curr);
        int newTranspose = sysState.knob0.getRotation(); 

        if (newTranspose > prevTranspose) {
            Serial.println("Transposing Up");

        } else if (newTranspose < prevTranspose) {
            Serial.println("Transposing Down");
        }

        bool knob0SPressed = !localInputs[25];

        if (!localInputs[20]){
            Serial.println("Knob 2S pressed");
        } else if (!localInputs[21]){
            Serial.println("Knob 3S pressed");
        } else if (!localInputs[22]){
            Serial.println("Joystick S pressed");
        } else if (!localInputs[24]){
            Serial.println("Knob 0S pressed");
        } else if (knob0SPressed && !prevKnob0SPressed){
            // Knob 1 S (!localInputs[25])
            xSemaphoreTake(sysState.mutex, portMAX_DELAY);
            moduleRole = (moduleRole == SENDER) ? RECEIVER : SENDER;
            xSemaphoreGive(sysState.mutex);
            Serial.println("Role changed");        
        }
        prevKnob0SPressed = knob0SPressed;

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

        // Update joystick values outside of the ISR
        int rawJoyX = analogRead(JOYX_PIN);
        int rawJoyY = analogRead(JOYY_PIN);
        // Use the same ymin and ymax as in sampleISR (adjust if needed)
        joyX12Val = map(rawJoyX, 800, 119, 0, 12);
        joyY12Val = map(rawJoyY, 800, 119, 0, 12);


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
        u8g2.print(joyX12Val);
        u8g2.print(",");
        u8g2.print(joyY12Val);
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
        Serial.println(sysState.RX_Message[0]);
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

    int ymax = 119;
    int ymin = 800;
    
    // static int prevTranspose = 0;
    // int newTranspose = sysState.knob0.getRotation();

    // Calculate effective step size based on moduleOctave.
    // (Assuming 4 is the base octave.)
    
    uint32_t effectiveStep = currentStepSize;

    /*if (newTranspose > prevTranspose) {
        effectiveStep *= 1.05946;
    }
    else if (newTranspose < prevTranspose) {
        effectiveStep /= 1.05946;
    }
    else {
        Serial.println("Max/Min Transpose reached");
    }

    prevTranspose = newTranspose;*/

    effectiveStep += ((int32_t)(joyY12Val - 6) * (effectiveStep / 100));

    if (moduleOctave > 4) {
        effectiveStep = effectiveStep << (moduleOctave - 4);
    } else if (moduleOctave < 4) {
        effectiveStep = effectiveStep >> (4 - moduleOctave);
    }

    //effectiveStep = effectiveStep*(analogRead(JOYY_PIN)/472);
    
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