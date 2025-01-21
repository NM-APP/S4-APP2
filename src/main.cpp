#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
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

#define PIN_SW1 2
#define PIN_SW2 3
#define PIN_RV1 A0
#define PIN_RV2 A1
#define PIN_RV3 A2
#define PIN_RV4 A3

SquareWv squarewv_;
Sawtooth sawtooth_;

float f = 1.0;
float q = 0.5;
float fb = (q + (q/(1.0-f)));
int16_t b1 = f*f * 256;
int16_t a1 = (2-2*f+f*fb-f*f*fb) * 256;
int16_t a2 = -(1-2*f+f*fb+f*f-f*f*fb) * 256;

void setNoteHz(float note)
{
    squarewv_.setFreq(note);
    sawtooth_.setFreq(note);
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
    // VCO
    int8_t vco = sawtooth_.next() + squarewv_.next();
    
    // VCF (disabled)
    int8_t vcf = vco;

    // VCA (disabled)   
    int8_t vca = vcf;

    int8_t output = vca;

    return output;
}

void TaskSquareWave(void *pvParameters)
{
    const int TEST_PIN = 7;
    pinMode(TEST_PIN, OUTPUT);
    
    (void) pvParameters;

    for (;;) // A Task shall never return or exit.
    {
        digitalWrite(TEST_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
        vTaskDelay( 5 / portTICK_PERIOD_MS ); // wait for one second
        digitalWrite(TEST_PIN, LOW);    // turn the LED off by making the voltage LOW
        vTaskDelay( 5 / portTICK_PERIOD_MS ); // wait for one second
    }
}

void taskAddToBuffer(void  *pvParameters)
{
    pcmAddSample(nextSample());
}


void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_SW1, INPUT);
    
    Serial.begin(9600);

    // Oscillator.
    squarewv_ = SquareWv(SQUARE_NO_ALIAS_2048_DATA);
    sawtooth_ = SquareWv(SAW2048_DATA);
    setNoteHz(440.0);


    xTaskCreate(
        taskAddToBuffer
        ,  "AddToBuffer"
        ,  128
        ,  NULL
        ,  2 
        ,  NULL );

    xTaskCreate(
        TaskSquareWave
        ,  "SquareWave"
        ,  128
        ,  NULL
        ,  1  
        ,  NULL );

    pcmSetup();

    Serial.println("Synth prototype ready");
}
void loop()
{    

}