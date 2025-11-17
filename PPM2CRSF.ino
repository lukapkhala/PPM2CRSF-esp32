#include <Arduino.h>
#include <HardwareSerial.h>

// ---------------- PPM CONFIG ----------------
#define PPM_PIN 4
#define PPM_CHANNELS 8
#define FRAME_GAP 3000     // µs gap = frame boundary

volatile uint16_t ppmBuf[16];
volatile uint8_t ppmIndex = 0;
volatile uint32_t lastMicrosPPM = 0;
volatile bool newFrame = false;

// ---------------- CRSF CONFIG ----------------
HardwareSerial CRSF(1);

#define CRSF_TX_PIN 15
#define CRSF_RX_PIN 16
#define CRSF_BAUD   420000

#define CRSF_ADDRESS        0xC8
#define CRSF_TYPE_RC        0x16
#define CRSF_CHANNEL_COUNT  16
#define CRSF_PAYLOAD_SIZE   22
#define CRSF_FRAME_SIZE     (3 + CRSF_PAYLOAD_SIZE + 1)

// ------------- CRC8 Dallas/Maxim ----------------
uint8_t crc8_d5(uint8_t crc, uint8_t data)
{
    crc ^= data;
    for (int i = 0; i < 8; i++)
        crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
    return crc;
}

// ------------- Map PPM µs → CRSF ---------------
uint16_t usToCrsf(uint16_t us)
{
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    return 173 + (us - 1000) * (1811 - 173) / 1000;
}

// ------------- Pack 16×11-bit channels ----------
void packChannels(uint16_t *ch, uint8_t *buf)
{
    uint32_t acc = 0;
    uint8_t bits = 0;
    uint8_t out = 0;

    for (int i = 0; i < 22; i++) buf[i] = 0;

    for (int i = 0; i < 16; i++)
    {
        acc |= (uint32_t)(ch[i] & 0x7FF) << bits;
        bits += 11;

        while (bits >= 8)
        {
            buf[out++] = acc & 0xFF;
            acc >>= 8;
            bits -= 8;
        }
    }
}

// ------------- CRSF SEND -----------------------
void sendCRSF(uint16_t *ch)
{
    uint8_t payload[CRSF_PAYLOAD_SIZE];
    uint8_t frame[CRSF_FRAME_SIZE];

    // CRSF header
    frame[0] = CRSF_ADDRESS;
    frame[1] = CRSF_PAYLOAD_SIZE + 2;  // type + payload + crc
    frame[2] = CRSF_TYPE_RC;

    // Convert channel values
    uint16_t crsfCh[16];
    for (int i = 0; i < 16; i++)
        crsfCh[i] = usToCrsf(ch[i]);

    packChannels(crsfCh, payload);

    // copy payload
    for (int i = 0; i < CRSF_PAYLOAD_SIZE; i++)
        frame[3 + i] = payload[i];

    // CRC
    uint8_t crc = 0;
    for (int i = 2; i < 2 + 1 + CRSF_PAYLOAD_SIZE; i++)
        crc = crc8_d5(crc, frame[i]);

    frame[3 + CRSF_PAYLOAD_SIZE] = crc;

    // transmit
    CRSF.write(frame, CRSF_FRAME_SIZE);
}

// ------------- PPM ISR --------------------------
void IRAM_ATTR ppmISR()
{
    uint32_t now = micros();
    uint32_t pulse = now - lastMicrosPPM;
    lastMicrosPPM = now;

    if (pulse > FRAME_GAP)
    {
        ppmIndex = 0;
        newFrame = true;
        return;
    }

    if (ppmIndex < PPM_CHANNELS)
    {
        ppmBuf[ppmIndex] = pulse;
        ppmIndex++;
    }
}

// ------------------- SETUP ----------------------
void setup()
{
    Serial.begin(115200);

    // PPM setup
    pinMode(PPM_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);

    // CRSF setup
    CRSF.begin(CRSF_BAUD, SERIAL_8E2, CRSF_RX_PIN, CRSF_TX_PIN);

    Serial.println("PPM → CRSF bridge running...");
}

// ------------------- LOOP -----------------------
void loop()
{
    static uint32_t lastCRSF = 0;
    uint32_t now = millis();

    // Send CRSF at 150 Hz
    if (now - lastCRSF >= 7)
    {
        lastCRSF = now;

        uint16_t chOut[16];

        noInterrupts();
        for (int i = 0; i < PPM_CHANNELS; i++)
            chOut[i] = ppmBuf[i];

        for (int i = PPM_CHANNELS; i < 16; i++)
            chOut[i] = 1500;  // default mid values

        bool fresh = newFrame;
        newFrame = false;
        interrupts();

        sendCRSF(chOut);
    }
}
