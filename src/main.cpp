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

#define PIN_SW1 2 //Note fixe
#define PIN_SW2 3 // Mélodie
#define PIN_RV1 A0 // tempo
#define PIN_RV2 A1 // Durée VCA
#define PIN_RV3 A2 // Fréquence de coupure VCF
#define PIN_RV4 A3 // Fréquence de résonnance VCF

SquareWv squarewv_;
Sawtooth sawtooth_;

// Déclare les variables des boutons a 0 pour éviter les faux positifs au démarrage
bool SW1 = false;
bool SW2 = false;

// Variable globale pour le début du VCA
bool isVCAActive = false;

// Création d'un type de structures pour rassembler les paramètres et diminuer le nombre queues
typedef struct
{
    float potentiometerValues[4];  // Tempo, VCA duration, VCF cut, VCF resonance
    int16_t filterCoefficients[3]; // b1, a1, a2
} AudioParams;
// Utilisation d'une queue pour les paramètres audio permet d'avoir un système thread-safe
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
// Process pour diminuer l'amplitude 
int8_t processVCA(int8_t input, bool sw1, bool sw2, float envelopeDuration)
{
    static float envelopePosition = 0.0f;
    static float decayFactor = 1.0f;
    static float envelopeStep = 0.0f;

    int8_t outputSignal = 0;
// Lit les valeurs globales des switches
    if (sw1 || sw2)
    {
        //Calcul du changement de la fréquence selon la durée du VCA
        envelopePosition = 0;
        envelopeStep = 1.0f / envelopeDuration; // Précalculer l'inverse
        outputSignal = input;
    }
    else if (isVCAActive)
    {
        if (envelopePosition < 1.0f) // Normalisation de l'enveloppe (0 à 1)
        {
            // diminution de la fréquence
            decayFactor = 1.0f - envelopePosition;
            outputSignal = input * decayFactor;
            envelopePosition += envelopeStep;
        }
        // Fin du VCA
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
    // initialise les variables en static pour qu'elles ne soient pas initialisées à chaque appel
    static int currentNoteIndex = 0;
    static int currentNoteDuration = 0;
    // Calcul du nombre d'élements dans la mélodie
    static const int songLength = sizeof(song) / sizeof(song[0]);
    // Création d'une structure pour les paramètres audio, permet de réunire les paramètres et de diminuer le nombre de queues
    AudioParams params;

    
    if (xQueuePeek(audioParamsQueue, &params, 0) != pdPASS)
    {
        return 0; // Return silence if data is unavailable
    }

    float *potentiometerValues = params.potentiometerValues;
    int16_t *filterCoefficients = params.filterCoefficients;

    float localTempo = potentiometerValues[0];
    // Multiplication par 8000, car la fréquence des notes est 8kHz
    float envelopeDuration = potentiometerValues[1] * 8000.0f;

    // Si le bouton 1 est appuyé, on joue une note fixe
    if (SW1)
    {
        squarewv_.setFreq(440.0f);
        sawtooth_.setFreq(440.0f);
        isVCAActive = true;
    }
    // Si le bouton 2 est appuyé, on joue la mélodie
    else if (SW2)
    {
        isVCAActive = true;
        if (currentNoteIndex < songLength)
        {
            // On met la fréquence de la note pour la durée indiquée dans la mélodie
            if (currentNoteDuration < song[currentNoteIndex].duration * 250.0f * 60.0f / (localTempo/2))
        
        
                squarewv_.setFreq(song[currentNoteIndex].freq);
                sawtooth_.setFreq(song[currentNoteIndex].freq);
                currentNoteDuration++;
            }
            // On change de note
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
        // Si le buffer est plein, on fait une autre task pour ne pas bloquer de la puissance
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
        AudioParams params;
        // Lecture des valeurs et mapping des potentiomètres
        params.potentiometerValues[0] = (analogRead(PIN_RV1) / 1023.0f) * (240.0 - 60.0f) + 60.0f;
        params.potentiometerValues[1] = (analogRead(PIN_RV2) / 1023.0f) * PI;
        params.potentiometerValues[2] = (analogRead(PIN_RV3) / 1023.0f);
        params.potentiometerValues[3] = (analogRead(PIN_RV4) / 1023.0f) * 3.0f;
        // Calcul des coefficients du filtre
        float fb = params.potentiometerValues[3] + (params.potentiometerValues[3] / (1.0 - params.potentiometerValues[2]));
        params.filterCoefficients[0] = params.potentiometerValues[2] * params.potentiometerValues[2] * 256;
        params.filterCoefficients[1] = (2 - 2 * params.potentiometerValues[2] + params.potentiometerValues[2] * fb - params.potentiometerValues[2] * params.potentiometerValues[2] * fb) * 256;
        params.filterCoefficients[2] = -(1 - 2 * params.potentiometerValues[2] + params.potentiometerValues[2] * fb + params.potentiometerValues[2] * params.potentiometerValues[2] - params.potentiometerValues[2] * params.potentiometerValues[2] * fb) * 256;
        // Ovewrite la queue d'un éléments avec les nouveaux paramètres, cette méthode est choisit pour être thread-safe
        xQueueOverwrite(audioParamsQueue, &params);
        // On attend 100ms, car la task doit s'exucuter au 10Hz, utilisation de vtaskDelay pour ne pas bloquer le processeur
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// Fonction pour les interruptions quand SW1 est appuyé, permet d'être très réactif et de ne pas avoir de task qui regarde en permanence l'état des boutons
void isrSW1()
{
    SW1 = digitalRead(PIN_SW1);
    // Donne priorité à SW1
    SW2 = false;
}
// Fonction pour les interruptions quand SW2 est appuyé, permet d'être très réactif et de ne pas avoir de task qui regarde en permanence l'état des boutons
void isrSW2()
{
    SW2 = digitalRead(PIN_SW2);
    // Donne priorité à SW2
    SW1 = false;
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_SW1, INPUT);
    pinMode(PIN_SW2, INPUT);

    Serial.begin(9600);

    // Initialisation des interruptions pour les SW
    attachInterrupt(digitalPinToInterrupt(PIN_SW1), isrSW1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_SW2), isrSW2, CHANGE);

    squarewv_ = SquareWv(SQUARE_NO_ALIAS_2048_DATA);
    sawtooth_ = Sawtooth(SAW2048_DATA);
    setNoteHz(440.0);

    // Création de la queue pour les paramètres audio
    audioParamsQueue = xQueueCreate(1, sizeof(AudioParams));

    // Initialisation des tâches, nous avons limitité le nombre de tâches pour ne pas surcharger le processeur et un code plus lisible
    // La lecture des potiomètres est prioritaire, car nous forçons la lecture à 10Hz,
    //donc nous voulons qu'elle soit prioritaire pour ne pas qu'elle soit en retard
    xTaskCreate(readPotentiometerTask, "readPotentiometer", 256, NULL, 3, NULL);
    // Tâche qui ajoute les fréquences au buffer
    xTaskCreate(taskAddToBuffer, "AddToBuffer", 256, NULL, 1, NULL);

    pcmSetup();

    Serial.println("Synth prototype ready");
}

void loop() {}
