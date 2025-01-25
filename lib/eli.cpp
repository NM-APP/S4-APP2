#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
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

#define PIN_SW1 2
#define PIN_SW2 3
#define PIN_RV1 A0
#define PIN_RV2 A1
#define PIN_RV3 A2
#define PIN_RV4 A3

#define E5 659.26
#define G5 783.99
#define A5 880.00
#define A5s 932.33
#define G4 392.00
#define E4 329.63
#define C5 523.25
#define D5 587.33
#define F5s 739.99
#define D5s 622.25
#define G5s 830.61
#define F5 698.46

Note MARIO[] = {{G5, 4}, {F5s, 4}, {F5, 4}, {D5s, 4}, {0, 4}, {E5, 4}, {0, 4}, {G5s, 4}, {A5, 4}, {C5, 4}};

SquareWv squarewv_;
Sawtooth sawtooth_;

float f = 1.0;
float q = 0.5;
float fb = (q + (q/(1.0-f)));
int16_t b1 = f*f * 256;
int16_t a1 = (2-2*f+f*fb-f*f*fb) * 256;
int16_t a2 = -(1-2*f+f*fb+f*f-f*f*fb) * 256;

int RV1_value;
int tempo;

int RV2_value;
float duree_VCA;

int RV3_value;
float freqCoupure;

int RV4_value;
float freqResonance;

float amplitude_facteur;
float amplitude_diminution;
int8_t valeur;

TaskHandle_t taskHandle_SW1 = NULL;
uint32_t notificationValue_SW1;
float tps_16t_ms = ((60*250)/tempo);
float noteDuration = (60000.0 / tempo) / 4; // Une seizième note.

void RV1_Tempo(void *pvParameters);
void RV2_Duree_VCA(void *pvParameters);
void RV3_FreqCoupure_VCF(void *pvParameters);
void RV4_FreqResonance_VCF(void *pvParameters);
void noteContinue(void *pvParameters);
void melodie(void *pvParameters);

void coefficientsVCF(){
    f = freqCoupure;// /SAMPLE_RATE;
    q = freqResonance;
    fb = (q + (q/(1.0-f)));
    b1 = f*f * 256;
    a1 = (2-2*f+f*fb-f*f*fb) * 256;
    a2 = -(1-2*f+f*fb+f*f-f*f*fb) * 256;
    }

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

//MUTEX ajouter ou QUEUES dans FreeRTOS (pour les quatre potentiometres)
//mélodie seulement lorsque bouton enfoncé

int8_t processVCA(int8_t note, bool noteActive){
    if(noteActive){
        amplitude_facteur = 1.0;
        amplitude_diminution = 1.0 / (duree_VCA * SAMPLE_RATE);
    }
    else if (amplitude_facteur > 0.0){
        amplitude_facteur -= amplitude_diminution;
        if(amplitude_facteur < 0.0){
            amplitude_facteur = 0.0;
        }
    }

    return note * amplitude_facteur;

}

int8_t nextSample(bool noteActive)
{
    // VCO
    int8_t vco = sawtooth_.next() + squarewv_.next();
    
    // VCF
    int8_t vcf = processVCF(vco);

    // VCA
    int8_t vca = processVCA(vcf, noteActive);

    int8_t output = vca;

    return output;
}

void alimTampon(int8_t note){
    if (pcmBufferFull() == false){
        pcmAddSample(note);
    }
    else{
        taskYIELD();
    }
}

void RV1_Tempo(void *pvParameters)
{
    while(1)
    {
        RV1_value = analogRead(PIN_RV1);
        //Serial.print("Potentiometre: ");
        //Serial.println(RV1_value);
        tempo = ((float)RV1_value * 180.0 / 1023.0) + 60;
        //Serial.print("Tempo: ");
        //Serial.println(tempo);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

void RV2_Duree_VCA(void *pvParameters)
{
    while(1)
    {
        RV2_value = analogRead(PIN_RV2);
        //Serial.print("Potentiometre: ");
        //Serial.println(RV2_value);
        duree_VCA = ((float)RV2_value * 3 / 1023.0);
        //Serial.print("Duree VCA: ");
        //Serial.println(duree_VCA);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

void RV3_FreqCoupure_VCF(void *pvParameters)
{
    while(1)
    {
        RV3_value = analogRead(PIN_RV3);
        //Serial.print("Potentiometre: ");
        //Serial.println(RV3_value);
        freqCoupure = ((float)RV3_value * PI / 1023.0);
        //Serial.print("Frequence Coupure: ");
        //Serial.println(freqCoupure);
        coefficientsVCF();
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

void RV4_FreqResonance_VCF(void *pvParameters)
{
    while(1)
    {
        RV4_value = analogRead(PIN_RV4);
        //Serial.print("Potentiometre: ");
        //Serial.println(RV4_value);
        freqResonance = ((float)RV4_value * 1.0 / 1023.0);
        //Serial.print("Frequence Resonance: ");
        //Serial.println(freqResonance);
        coefficientsVCF();
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

void noteContinue(void *pvParameters)
{
    while(1)
    {
        bool noteActive = digitalRead(PIN_SW1) == HIGH;

        if(noteActive){
            setNoteHz(840.0); 
        }

        valeur = nextSample(noteActive);
        alimTampon(valeur);
    }
}

void melodie(void *pvParameters)
{
    while(1)
    {
        bool melodieActive = digitalRead(PIN_SW2) == HIGH;

        if (melodieActive)
        {
            for (int i = 0; i < sizeof(MARIO) / sizeof(Note); ++i)
            {
                // Déterminer la fréquence et la durée de la note
                float noteFreq = MARIO[i].freq;
                int noteDurationMs = MARIO[i].duration * ((60*250)/tempo);

                // Activer la note
                setNoteHz(noteFreq);

                // Jouer la note
                for (int t = 0; t < noteDurationMs * SAMPLE_RATE / 1000; ++t)
                {
                    int8_t sample = nextSample(true);
                    alimTampon(sample);

                    if(digitalRead(PIN_SW2) == LOW){
                    break;
                    }
                }

                if(digitalRead(PIN_SW2) == LOW){
                    break;
                }

            }
        }
    }
}


void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_SW1, INPUT);
    pinMode(PIN_SW2, INPUT);
    pinMode(PIN_RV1, INPUT);
    pinMode(PIN_RV2, INPUT);
    pinMode(PIN_RV3, INPUT);
    pinMode(PIN_RV4, INPUT);
    
    Serial.begin(9600);
    
    //Creation des taches
    xTaskCreate(RV1_Tempo, "RV1_Tempo", 128, NULL, 2, NULL);
    xTaskCreate(RV2_Duree_VCA, "RV2_Duree_VCA", 128, NULL, 2, NULL);
    xTaskCreate(RV3_FreqCoupure_VCF, "RV3_FreqCoupure_VCF", 128, NULL, 2, NULL);
    xTaskCreate(RV4_FreqResonance_VCF, "RV4_FreqResonance_VCF", 128, NULL, 2, NULL);
    xTaskCreate(noteContinue, "noteContinue", 128, NULL, 1, NULL);
    xTaskCreate(melodie, "melodie", 128, NULL, 1, NULL);

    // Oscillator.
    squarewv_ = SquareWv(SQUARE_NO_ALIAS_2048_DATA);
    sawtooth_ = SquareWv(SAW2048_DATA);

    pcmSetup();

    Serial.println("Synth prototype ready");
}

void loop()
{    

}

//pas de attach interrupt: lire constamment les valeurs des potentiometres
//lecture constante des boutons (prend plus de CPU) mais permet d'être plus réactif