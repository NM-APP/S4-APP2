#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <Oscil.h>
#include <tables/saw2048_int8.h>
#include <tables/square_no_alias_2048_int8.h>

#include "song.hpp"
#include "pcm_audio.hpp"
#include <event_groups.h>
#include <queue.h>

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

int16_t b1;
int16_t a1;
int16_t a2;

// Les handles des tâches
TaskHandle_t Handle_Potentio = NULL;
TaskHandle_t Handle_Buffer = NULL;

// Les queues
QueueHandle_t qCoupure;
QueueHandle_t qResonnance;
QueueHandle_t qTempo;
QueueHandle_t qDureeVca;
QueueHandle_t qSw1;
QueueHandle_t qSw2;

// Les events groups
EventGroupHandle_t Event;

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

int8_t processVCA(int8_t input)
{
    static float amplitude = 0.0;
    static float pas_amplitude = 0.0;
    static float DureeVca;
    bool Sw1;
    bool Sw2;
    // On récupère les valeurs des queues
    xQueuePeek(qDureeVca, &DureeVca, portMAX_DELAY);
    xQueuePeek(qSw1, &Sw1, portMAX_DELAY);
    xQueuePeek(qSw2, &Sw2, portMAX_DELAY);

    // Éviter la division par zéro
    if (DureeVca == 0.0)
    {
        pas_amplitude = 1.0;
    }
    else
    {
        pas_amplitude = 1.0 / (DureeVca * SAMPLE_RATE);
    }

    if (Sw1 == 1 || Sw2 == 1)
    {
        amplitude = 1.0;
    }
    else if (amplitude > 0.0)
    {
        amplitude -= pas_amplitude;
        // Si le buffer devient plus grand que le nombre de buffer,
        //  on le remet à la valeur maximale
        if (amplitude < 0.0)
        {
            amplitude = 0.0;
        }
    }

    return (int8_t)(input * amplitude);
}

int8_t nextSample()
{

    // VCO (enabled)
    int8_t vco = sawtooth_.next() + squarewv_.next();

    // VCF (enabled)
    int8_t vcf = processVCF(vco);

    // VCA (enabled)
    int8_t vca = processVCA(vcf);

    int8_t output = vca;

    return output;
}

void Potentio_Read(void *pvParameters)
{
    float Tempo;
    float DureeVca;
    float Coupure;
    float Resonnance;
    for (;;)
    {
        // Nous utilisons des queues afin de ne pas avoir de problèmes de concurrence.
        // Leur taille est de 1, car nous n'avons besoin que de la dernière valeur lue.
        // Nous utilisons xQueueOverwrite pour écraser la valeur précédente, car l'ancienne n'a pas de valeur.
        Tempo = analogRead(PIN_RV1) * 180.0f / 1023.0f + 60.0f; // Transformation de la valeur analogique en tempo
        xQueueOverwrite(qTempo, &Tempo);

        DureeVca = analogRead(PIN_RV2) * 3.0f / 1023.0f; // Transformation de la valeur analogique en durée de l'enveloppe VCA
        xQueueOverwrite(qDureeVca, &DureeVca);

        Coupure = analogRead(PIN_RV3) * PI / 1023.0f; // Transformation de la valeur analogique en fréquence de coupure
        xQueueOverwrite(qCoupure, &Coupure);

        Resonnance = analogRead(PIN_RV4) / 1023.0f; // Transformation de la valeur analogique en résonnance
        xQueueOverwrite(qResonnance, &Resonnance);

        // Test pour vérifier que les valeurs sont bien lues et  que les queues fonctionnent
        //  float value;
        //  xQueuePeek(qDureeVca, &value, portMAX_DELAY);
        //  Serial.print("VCA : ");
        //  Serial.println(value);

        float f = Coupure;
        float q = Resonnance;

        float fb = (q + (q / (1.0 - f)));
        b1 = f * f * 256;
        a1 = (2 - 2 * f + f * fb - f * f * fb) * 256;
        a2 = -(1 - 2 * f + f * fb + f * f - f * f * fb) * 256;

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

//fonction de fréquence
void frequence()
{
    static int i = 0;
    static int j = 0;
    static int longueur = sizeof(song) / sizeof(song[0]);
    float tempo;
    xQueuePeek(qTempo, &tempo, portMAX_DELAY);

    bool sw2;

    xQueuePeek(qSw2, &sw2, portMAX_DELAY);
    if (sw2)
    {
        if (i < longueur)
        {
            if (j < song[i].duration * 60.0f * 250.0f / tempo)
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
        setNoteHz(140.0);
    }
}

// Fonction de remplissage du buffer
void Full_Buffer(void *pvParameters)
{
    for (;;)
    {

        frequence();
        // On remplit le buffer
        // setfreq();
        // On vérifie si le buffer est plei
        if(pcmBufferEmpty()){
            Serial.println("Buffer vide");
        }
        if (pcmBufferFull() == false)
        {
            pcmAddSample(nextSample());
        }
        else{
            taskYIELD();
        }
    }
}



// Fonctions ISR
void sw1Isr()
{
    // In this case, eSetValueWithOverwrite means if multiple button presses occur before the task
    // processes the first one, only the most recent notification will be kept.
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    bool sw1 = digitalRead(PIN_SW1);
    xQueueOverwriteFromISR(qSw1, &sw1, &xHigherPriorityTaskWoken);
    // Si une tâche de plus haute priorité a été débloquée, demander un changement de contexte
    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

// Fonctions ISR
void sw2Isr()
{
    // In this case, eSetValueWithOverwrite means if multiple button presses occur before the task
    // processes the first one, only the most recent notification will be kept.
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    bool sw2 = digitalRead(PIN_SW2);
    xQueueOverwriteFromISR(qSw2, &sw2, &xHigherPriorityTaskWoken);
    // Si une tâche de plus haute priorité a été débloquée, demander un changement de contexte
    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_SW1, INPUT);
    pinMode(PIN_SW2, INPUT);

    Serial.begin(9600);

    // Oscillator.
    squarewv_ = SquareWv(SQUARE_NO_ALIAS_2048_DATA);
    sawtooth_ = SquareWv(SAW2048_DATA);


    pcmSetup();

    Serial.println("Synth prototype ready");

    // Créations des queues
    qCoupure = xQueueCreate(1, sizeof(float));
    qResonnance = xQueueCreate(1, sizeof(float));
    qTempo = xQueueCreate(1, sizeof(float));
    qDureeVca = xQueueCreate(1, sizeof(float));
    qSw1 = xQueueCreate(1, sizeof(bool));
    qSw2 = xQueueCreate(1, sizeof(bool));

    // interrupt en pressant le petit bouton
    attachInterrupt(digitalPinToInterrupt(PIN_SW1), sw1Isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_SW2), sw2Isr, CHANGE);

    // Lecture des potentiomètres
    xTaskCreate(
        Potentio_Read, "Lecture potentiomètres", 256 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 2 // Décider la priorité
        ,
        &Handle_Potentio);

    // Remplissage du buffer
    xTaskCreate(
        Full_Buffer, "Remplissage buffer", 512 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 1 // Décider la priorité
        ,
        NULL);
}

void loop()
{
}