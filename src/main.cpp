#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <Oscil.h>
#include <semphr.h>
#include "song.hpp"
#include <tables/saw2048_int8.h>
#include <tables/square_no_alias_2048_int8.h>

#include "pcm_audio.hpp"

/*

Cher(e) stagiaire de H24,
Désolé, j'ai passé tout mon stage à prototyper le PCB et j'ai manqué de
temps pour m'occuper du logiciel. Tu devrais être pas loin de pouvoir
faire jouer quelque chose sur le haut-parleur; les librairies sont là,
mais je suis nul avec FreeRTOS... Bonne chance!

- Stagiaire de A23

*/

using Sawtooth = Oscil<SAW2048_NUM_CELLS, SAMPLE_RATE>;
using SquareWv = Oscil<SQUARE_NO_ALIAS_2048_NUM_CELLS, SAMPLE_RATE>;

#define PIN_SW1 2
#define PIN_SW2 3
#define PIN_RV1 A0
#define PIN_RV2 A1
#define PIN_RV3 A2
#define PIN_RV4 A3

SquareWv squarewv_;
Sawtooth sawtooth_;

float tempoValue;
float VCADurationValue;
float VCFCutValue;
float VCFResonanceValue;
QueueHandle_t potentiometerQueue;

bool SW1;
bool SW2;

float fb;
int16_t b1;
int16_t a1;
int16_t a2;
QueueHandle_t filterCoeffQueue;

void setNoteHz(float note)
{
    squarewv_.setFreq(note);
    sawtooth_.setFreq(note);
}

int8_t processVCF(int8_t input)
{
    static int8_t y1 = 0;
    static int8_t y2 = 0;

    int8_t output = 0;

    // Reduce the time spent in the mutex
    int16_t local_b1 = 0, local_a1 = 0, local_a2 = 0;

    // Retrieve filter coefficients from the queue
    int16_t filterCoefficients[3];
    if (xQueuePeek(filterCoeffQueue, &filterCoefficients, 0) == pdPASS)
    {
        local_b1 = filterCoefficients[0];
        local_a1 = filterCoefficients[1];
        local_a2 = filterCoefficients[2];
    }

    int16_t y0 = local_b1 * input + local_a1 * y1 + local_a2 * y2;
    output = 0xFF & (y0 >> 8);

    y2 = y1;
    y1 = output;

    return output;
}

int8_t processVCA(int8_t input)
{
    static bool isVCAActive = false;
    static float envelopePosition = 0;

    int8_t outputSignal = 0;
    float envelopeDuration = 0;

    // Retrieve potentiometer values from the queue
    float potentiometerValues[4];
    if (xQueuePeek(potentiometerQueue, &potentiometerValues, 0) == pdPASS)
    {
        envelopeDuration = potentiometerValues[1] * 8000.0f; // Use the VCADurationValue from the queue
    }

    if (SW1 || SW2)
    {
        isVCAActive = true;
        envelopePosition = 0;
        outputSignal = input;
    }
    else if (isVCAActive)
    {
        if (envelopePosition < envelopeDuration)
        {
            float decayFactor = 1.0f - (envelopePosition / envelopeDuration);
            outputSignal = input * decayFactor;
            envelopePosition++;
        }
        else
        {
            outputSignal = 0;
            isVCAActive = false;
        }
    }
    else
    {
        outputSignal = 0;
    }

    return outputSignal;
}

int8_t nextSample()
{
    if (SW1)
    {
        setNoteHz(440.0f);
    }
    else if (SW2)
    {
        float local_tempo;
        float potentiometerValues[4];
        if (xQueuePeek(potentiometerQueue, &potentiometerValues, 0) == pdPASS)
        {
            local_tempo = potentiometerValues[0];
        }
        static int currentNoteIndex = 0;
        static int currentNoteDuration = 0;
        static int songLength = sizeof(song) / sizeof(song[0]);

        if (currentNoteIndex < songLength)
        {
            if (currentNoteDuration < song[currentNoteIndex].duration * 250.0f * 60.0f / (local_tempo/2))
            {
                setNoteHz(song[currentNoteIndex].freq);
                currentNoteDuration++;
            }
            else
            {
                currentNoteIndex++;
                currentNoteDuration = 0;
            }
        }
        else
        {
            currentNoteDuration = 0;
            currentNoteIndex = 0;
            setNoteHz(song[currentNoteIndex].freq);
        }
    }

    int8_t vco = sawtooth_.next() + squarewv_.next();
    int8_t vcf = processVCF(vco);
    int8_t vca = processVCA(vcf);

    int8_t output = vca;

    return output;
}

void taskAddToBuffer(void *pvParameters)
{
    for (;;)
    {
        if (pcmBufferFull() == false)
        {
            pcmAddSample(nextSample());
        }
        else
        {
            taskYIELD();
        }
    }
}

void readPotentiometerTask(void *pvParameters __attribute__((unused)))
{
    for (;;)
    {
        // Read potentiometer values before placing them in the queue to avoid holding the mutex for long
        float _tempoValue = (analogRead(PIN_RV1) / 1023.0f) * (240.0 - 60.0f) + 60.0f;
        float _VCADurationValue = (analogRead(PIN_RV2) / 1023.0f) * PI;
        float _VCFCutValue = (analogRead(PIN_RV3) / 1023.0f);
        float _VCFResonanceValue = (analogRead(PIN_RV4) / 1023.0f) * 3.0f;

        // Store potentiometer values in a temporary array
        float potentiometerValues[4] = {_tempoValue, _VCADurationValue, _VCFCutValue, _VCFResonanceValue};

        // Send potentiometer values to the potentiometer queue
        xQueueOverwrite(potentiometerQueue, &potentiometerValues);

        // Calculate the filter coefficients
        static float f = 0;
        static float q = 0;

        // Update filter coefficients based on the potentiometer values
        f = _VCFCutValue;
        q = _VCFResonanceValue;

        // Calculate filter coefficients
        fb = (q + (q / (1.0 - f)));
        b1 = f * f * 256;
        a1 = (2 - 2 * f + f * fb - f * f * fb) * 256;
        a2 = -(1 - 2 * f + f * fb + f * f - f * f * fb) * 256;

        // Store filter coefficients in an array
        int16_t filterCoefficients[3] = {b1, a1, a2};

        // Send filter coefficients to the filter queue
        xQueueOverwrite(filterCoeffQueue, &filterCoefficients);

        // Delay to control sampling rate
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void isrSW1()
{
    SW1 = digitalRead(PIN_SW1);
    SW2 = 0;
}

void isrSW2()
{
    SW2 = digitalRead(PIN_SW2);
    SW1 = 0;
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_SW1, INPUT);

    Serial.begin(9600);

    attachInterrupt(digitalPinToInterrupt(PIN_SW1), isrSW1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_SW2), isrSW2, CHANGE);

    // Oscillator.
    squarewv_ = SquareWv(SQUARE_NO_ALIAS_2048_DATA);
    sawtooth_ = SquareWv(SAW2048_DATA);
    setNoteHz(440.0);

    // QUEUES
    potentiometerQueue = xQueueCreate(1, sizeof(float[4]));
    filterCoeffQueue = xQueueCreate(1, sizeof(int16_t[3]));

    // TASKS
    xTaskCreate(readPotentiometerTask, "readPotentiometer", 256, NULL, 2, NULL);
    xTaskCreate(taskAddToBuffer, "AddToBuffer", 256, NULL, 1, NULL);

    pcmSetup();

    Serial.println("Synth prototype ready");
}

void loop()
{
}