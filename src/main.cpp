#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <event_groups.h>
#include <Oscil.h>
#include <tables/saw2048_int8.h>
#include <tables/square_no_alias_2048_int8.h>

#include "pcm_audio.hpp"
#include "song.hpp"

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

int16_t b1;
int16_t a1;
int16_t a2;

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
QueueHandle_t SW1Queue;
QueueHandle_t SW2Queue;

EventGroupHandle_t buttonEventGroup;

void setNoteHz(float note)
{
    squarewv_.setFreq(note);
    sawtooth_.setFreq(note);
}

int8_t processVCA(int8_t input)
{
    static float amplitude_factor = 0.0f;
    static float amplitude_diminution = 0.0f;
    static float duree_VCA;
    bool isSW1;
    bool isSW2;

    xQueuePeek(SW1Queue, &isSW1, eNoAction);
    xQueuePeek(SW2Queue, &isSW2, eNoAction);
    xQueuePeek(VCADurationQueue, &duree_VCA, eNoAction);

    if (isSW1 || isSW2)
    {
        amplitude_factor = 1.0;
        amplitude_diminution = 1.0 / (duree_VCA * SAMPLE_RATE);
    }
    else if (amplitude_factor > 0.0)
    {
        amplitude_factor -= amplitude_diminution;
        if (amplitude_factor < 0.0)
        {
            amplitude_factor = 0.0;
        }
    }

    return input * amplitude_factor;
}

int8_t processVCF(int8_t input)
{
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
    int8_t vco = sawtooth_.next() + squarewv_.next();
    int8_t vcf = processVCF(vco);
    int8_t vca = processVCA(vcf);

    int8_t output = vca;

    return output;
}

void Sequencer()
{
    static int i = 0;
    static int j = 0;
    int8_t songLength = sizeof(song) / sizeof(song[0]);
    bool SW2;
    float tempo;

    xQueuePeek(SW2Queue, &SW2, eNoAction);
    xQueuePeek(tempoQueue, &tempo, eNoAction);

    if (SW2)
    {
        if (i < songLength)
        {
            if (j < song[i].duration * (60.0f * 250.0f) / tempo)
            {
                setNoteHz(song[i].freq);
                j++;
            }
            else
            {
                i++;
                j = 0;
            }
        }
        else
        {
            i = 0;
            j = 1;
            setNoteHz(song[i].freq);
        }
    }
    else
    {
        setNoteHz(400.0f);
    }
}

void taskAddToBuffer(void *pvParameters)
{
    for (;;)
    {
        Sequencer();
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

        float f = VCFCutValue;
        float q = VCFResonanceValue;

        float fb = (q + (q / (1.0 - f)));
        b1 = f * f * 256;
        a1 = (2 - 2 * f + f * fb - f * f * fb) * 256;
        a2 = -(1 - 2 * f + f * fb + f * f - f * f * fb) * 256;

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void isrSW1()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    bool sw1State = digitalRead(PIN_SW1);
    xQueueOverwriteFromISR(SW1Queue, &sw1State, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

void isrSW2()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    bool sw2State = digitalRead(PIN_SW2);
    xQueueOverwriteFromISR(SW2Queue, &sw2State, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_SW1, INPUT);
    pinMode(PIN_SW2, INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_SW1), isrSW1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_SW2), isrSW2, CHANGE);

    buttonEventGroup = xEventGroupCreate();

    Serial.begin(9600);

    tempoQueue = xQueueCreate(1, sizeof(float));
    VCADurationQueue = xQueueCreate(1, sizeof(float));
    VCFCutQueue = xQueueCreate(1, sizeof(float));
    VCFResonanceQueue = xQueueCreate(1, sizeof(float));
    SW1Queue = xQueueCreate(1, sizeof(bool));
    SW2Queue = xQueueCreate(1, sizeof(bool));

    bool initSwitches = false;

    xQueueOverwrite(SW1Queue, &initSwitches);
    xQueueOverwrite(SW2Queue, &initSwitches);

    // Oscillator.
    squarewv_ = SquareWv(SQUARE_NO_ALIAS_2048_DATA);
    sawtooth_ = SquareWv(SAW2048_DATA);
    setNoteHz(164.81);

    xTaskCreate(
        taskAddToBuffer, "AddToBuffer", 256, NULL, 1, NULL);

    xTaskCreate(
        readPotentiometer, "ReadPotentiometer", 256, NULL, 2, NULL);

    pcmSetup();

    Serial.println("Synth prototype ready");
}
void loop()
{
}