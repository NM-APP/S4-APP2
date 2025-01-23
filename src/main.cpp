#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <Oscil.h>
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

#define PIN_SW1 2  // note fixe
#define PIN_SW2 3  // mélodie
#define PIN_RV1 A0 // tempo
#define PIN_RV2 A1 // durée VCA
#define PIN_RV3 A2 // fréquence de coupure VCF
#define PIN_RV4 A3 // fréquence de résonance VCF

SquareWv squarewv_;
Sawtooth sawtooth_;

QueueHandle_t tempoQueue;
QueueHandle_t VCADurationQueue;
QueueHandle_t VCFCutQueue;
QueueHandle_t VCFResonanceQueue;

void setNoteHz(float note)
{
    squarewv_.setFreq(note);
    sawtooth_.setFreq(note);
}

int8_t processVCF(int8_t input)
{
    float f; // Coupure
    float q; // Résonnance

    xQueuePeek(VCFCutQueue, &f, portMAX_DELAY);
    xQueuePeek(VCFResonanceQueue, &q, portMAX_DELAY);

    float fb = (q + (q / (1.0 - f)));
    int16_t b1 = f * f * 256;
    int16_t a1 = (2 - 2 * f + f * fb - f * f * fb) * 256;
    int16_t a2 = -(1 - 2 * f + f * fb + f * f - f * f * fb) * 256;

    static int8_t y1 = 0;
    static int8_t y2 = 0;

    int16_t y0 = b1 * input + a1 * y1 + a2 * y2;

    int8_t output = 0xFF & (y0 >> 8);

    y2 = y1;
    y1 = output;

    return output;
}

int8_t nextSample()
{
    // VCO
    int8_t vco = sawtooth_.next() + squarewv_.next();

    // VCF (disabled)
    int8_t vcf = processVCF(vco);

    // VCA (disabled)
    int8_t vca = vcf;

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

void readPotentiometer(void *pvParameters)
{
    (void)pvParameters;

    float tempoValue;
    float VCADurationValue;
    float VCFCutValue;
    float VCFResonanceValue;

    for (;;)
    {
        tempoValue = (analogRead(PIN_RV1) / 1023.0) * (240.0 - 60.0) + 60.0;
        VCADurationValue = (analogRead(PIN_RV2) / 1023.0) * PI;
        VCFCutValue = (analogRead(PIN_RV3) / 1023.0);
        VCFResonanceValue = (analogRead(PIN_RV4) / 1023.0) * 3.0;

        xQueueOverwrite(tempoQueue, &tempoValue);
        xQueueOverwrite(VCADurationQueue, &VCADurationValue);
        xQueueOverwrite(VCFCutQueue, &VCFCutValue);
        xQueueOverwrite(VCFResonanceQueue, &VCFResonanceValue);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void isrSW1()
{

}

void isrSW2()
{

}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_SW1, INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_SW1), isrSW1, eNoAction);
    attachInterrupt(digitalPinToInterrupt(PIN_SW2), isrSW2, eNoAction);

    Serial.begin(9600);
    tempoQueue = xQueueCreate(1, sizeof(float));
    VCADurationQueue = xQueueCreate(1, sizeof(float));
    VCFCutQueue = xQueueCreate(1, sizeof(float));
    VCFResonanceQueue = xQueueCreate(1, sizeof(float));

    // Oscillator.
    squarewv_ = SquareWv(SQUARE_NO_ALIAS_2048_DATA);
    sawtooth_ = SquareWv(SAW2048_DATA);
    setNoteHz(440.0);

    xTaskCreate(
        taskAddToBuffer, "AddToBuffer", 128, NULL, 1, NULL);

    xTaskCreate(
        readPotentiometer, "ReadPotentiometer", 128, NULL, 3, NULL);

    pcmSetup();

    Serial.println("Synth prototype ready");
}
void loop()
{
}