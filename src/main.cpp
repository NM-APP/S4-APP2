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

bool SW1 = false;
bool SW2 = false;

bool isVCAActive = false;

/*
    Pour éviter de faire plusieurs queue correspondant à chaque coefficients et valeur de potentiomètre, nous utilisons une seule Queue contenant une struct.
*/

typedef struct
{
    float potentiometerValues[4];  // Tempo, VCA duration, VCF cut, VCF resonance
    int16_t filterCoefficients[3]; // b1, a1, a2
} AudioParams;

QueueHandle_t audioParamsQueue;

void setNoteHz(float note)
{
    squarewv_.setFreq(note);
    sawtooth_.setFreq(note);
}

int8_t processVCF(int8_t input, int16_t b1, int16_t a1, int16_t a2)
{
    static int8_t y1 = 0;
    static int8_t y2 = 0;

    int16_t y0 = b1 * input + a1 * y1 + a2 * y2;
    int8_t output = 0xFF & (y0 >> 8);

    y2 = y1;
    y1 = output;

    return output;
}

int8_t processVCA(int8_t input, bool sw1, bool sw2, float envelopeDuration)
{
    static float envelopePosition = 0.0f;
    static float decayFactor = 1.0f;
    static float envelopeStep = 0.0f;

    int8_t outputSignal = 0;

    if (sw1 || sw2)
    {
        envelopePosition = 0;
        envelopeStep = 1.0f / envelopeDuration; // Précalculer l'inverse
        outputSignal = input;
    }
    else if (isVCAActive)
    {
        if (envelopePosition < 1.0f) // Normalisation de l'enveloppe (0 à 1)
        {
            decayFactor = 1.0f - envelopePosition;
            outputSignal = input * decayFactor;
            envelopePosition += envelopeStep;
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
    static int currentNoteIndex = 0;
    static int currentNoteDuration = 0;
    static const int songLength = sizeof(song) / sizeof(song[0]);

    /*
        Pour optimiser l'ajout de notes dans le buffer, nous avons utilisé une seule queue pour faire un seul xQueuePeek. D'après nos tests,
        un xQueuePeek() est très coûteux.
    */

    AudioParams params;

    if (xQueuePeek(audioParamsQueue, &params, 0) != pdPASS)
    {
        return 0; 
    }

    float *potentiometerValues = params.potentiometerValues;
    int16_t *filterCoefficients = params.filterCoefficients;

    float localTempo = potentiometerValues[0];
    float envelopeDuration = potentiometerValues[1] * 8000.0f;

    if (SW1)
    {
        squarewv_.setFreq(440.0f);
        sawtooth_.setFreq(440.0f);
        isVCAActive = true;
    }

    /*
        Ici, on a pris une approche qui maximise le contrôle sur la mélodie, pour ce faire, on met le nombre de notes dans le buffer correspondant à 
        la durée de chaque note. Par contre, c'est un trade off de performance contre le contrôle. 
        
        On a dû optimisé le code davantage pour atteindre les requis et les performances voulu. Nous aurions pu faire une tâche qui, avec un délai correspondant à
        la durée du temps, ajouterait les fréquences dans le buffer.

        Dans l'optique où c'est un produit qui serait destiné à des musiciens, on devrait prioriser la précision des mélodies.
    */
    else if (SW2)
    {
        isVCAActive = true;
        if (currentNoteIndex < songLength)
        {
            if (currentNoteDuration < song[currentNoteIndex].duration * 250.0f * 60.0f / (localTempo/2))
            {
                squarewv_.setFreq(song[currentNoteIndex].freq);
                sawtooth_.setFreq(song[currentNoteIndex].freq);
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
        }
    }
    else
    {
        currentNoteDuration = 0;
        currentNoteIndex = 0;
    }

    /*
        Pour évité de devoir accèder à la queue encore dans les fonctions processVCF et processVCA, on passe les variable contenue dans la struct
        directement en paramètre.
    */
    int8_t vco = sawtooth_.next() + squarewv_.next();
    int8_t vcf = processVCF(vco, filterCoefficients[0], filterCoefficients[1], filterCoefficients[2]);
    int8_t vca = processVCA(vcf, SW1, SW2, envelopeDuration);

    return vca;
}

void taskAddToBuffer(void *pvParameters)
{
    for (;;)
    {
        if (!pcmBufferFull())
        {
            pcmAddSample(nextSample());
        }
        else
        {
            taskYIELD();
        }
    }
}


/*
    Cette tâche s'effectue à 10Hz, donc on mets les valeurs analogues des potentiomètres et les coefficients de filtre dans une structure
    qu'on met dans une seule xQueue.
*/
void readPotentiometerTask(void *pvParameters __attribute__((unused)))
{

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10;

    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        AudioParams params;

        params.potentiometerValues[0] = (analogRead(PIN_RV1) / 1023.0f) * (240.0 - 60.0f) + 60.0f;
        params.potentiometerValues[1] = (analogRead(PIN_RV2) / 1023.0f) * PI;
        params.potentiometerValues[2] = (analogRead(PIN_RV3) / 1023.0f);
        params.potentiometerValues[3] = (analogRead(PIN_RV4) / 1023.0f) * 3.0f;

        float fb = params.potentiometerValues[3] + (params.potentiometerValues[3] / (1.0 - params.potentiometerValues[2]));
        params.filterCoefficients[0] = params.potentiometerValues[2] * params.potentiometerValues[2] * 256;
        params.filterCoefficients[1] = (2 - 2 * params.potentiometerValues[2] + params.potentiometerValues[2] * fb - params.potentiometerValues[2] * params.potentiometerValues[2] * fb) * 256;
        params.filterCoefficients[2] = -(1 - 2 * params.potentiometerValues[2] + params.potentiometerValues[2] * fb + params.potentiometerValues[2] * params.potentiometerValues[2] - params.potentiometerValues[2] * params.potentiometerValues[2] * fb) * 256;

        xQueueOverwrite(audioParamsQueue, &params);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/*
    Nous avons utilisé des ISR pour éviter d'avoir une tâche qui lit les valeurs des boutons en permanence ce qui, évidamment,
    ne serait pas optimale.

    Comme nous programmons un synthetiseur et non une fusée qui va dans l'espace, les variables d'états des boutons
    sont des variables globales non "thread-safe". Dans notre cas, c'est assez réactif et nous facilite beaucoup l'implémentation.
*/

void isrSW1()
{
    SW1 = digitalRead(PIN_SW1);
    SW2 = false;
}

void isrSW2()
{
    SW2 = digitalRead(PIN_SW2);
    SW1 = false;
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_SW1, INPUT);
    pinMode(PIN_SW2, INPUT);

    Serial.begin(9600);

    attachInterrupt(digitalPinToInterrupt(PIN_SW1), isrSW1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_SW2), isrSW2, CHANGE);

    squarewv_ = SquareWv(SQUARE_NO_ALIAS_2048_DATA);
    sawtooth_ = Sawtooth(SAW2048_DATA);
    setNoteHz(440.0);

    audioParamsQueue = xQueueCreate(1, sizeof(AudioParams));

    /*
        Pour simplifier l'implémentation et évité le plus de task switching possible, nous avons utilisé seulement 2 tasks.
        Nous priorisons la tâche de lecture du potentiomètre comme celle-ci s'effectue à 10hz et l'autre beaucoup plus rapidement, 
        donc on s'assure que la lecture des potentiomètre se fasse à tout les 100 ms.
        
    */
    xTaskCreate(readPotentiometerTask, "readPotentiometer", 256, NULL, 3, NULL);
    xTaskCreate(taskAddToBuffer, "AddToBuffer", 256, NULL, 1, NULL);

    pcmSetup();

    Serial.println("Synth prototype ready");
}

void loop() {}
