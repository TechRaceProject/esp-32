#include <Arduino.h>
#include "Freenove_4WD_Car_For_ESP32.h"
#include "Freenove_4WD_Race_Data.h"

// Define if the car is in race mode. Golang API will use this to update a race model
int race_id = 0;
bool race_mode = false;
bool must_end_race_api_signal = false;

float average_speed = 0;
float distance_covered = 0;
int collision_duration = 0;
int out_of_parcours = 0;

unsigned long last_distance_covered_update_time = 0;
unsigned long last_out_of_parcours_update_time = 0;
unsigned long last_collision_duration_update_time = 0;
unsigned long race_start_time = 0; // Heure de début de la course

int previous_race_id = 0;
float distance_previously_covered = 0;


// Vitesse constante m/s en fonction de la puissance des roues
float getConstantSpeed() {
    float singleContantSpeed = 0.357 / 1000;
    int power1 = Get_Wheel_1_Speed();
    int power2 = Get_Wheel_2_Speed();
    int power3 = Get_Wheel_3_Speed();
    int power4 = Get_Wheel_4_Speed();

    int power = (power1 + power2 + power3 + power4) / 4;

    return singleContantSpeed * power;
}

// Fonction pour mettre à jour la distance parcourue
void updateDistanceCovered() {
    unsigned long current_time = millis();
    
    // Vérifie si une seconde s'est écoulée
    if (current_time - last_distance_covered_update_time >= 1000) {
        float speed = getConstantSpeed(); // Vitesse en m/s
        
        // Mettre à jour la distance parcourue
        distance_covered += speed; // Ajouter la distance parcourue en une seconde
        
        // Mettre à jour le temps de la dernière mise à jour
        last_distance_covered_update_time = current_time;

        return;
    }
}

// Fonction pour calculer la vitesse moyenne
void updateAverageSpeed() {
    unsigned long current_time = millis();

    // Calculer le temps total écoulé en secondes
    float elapsed_time = (current_time - race_start_time) / 1000.0;

    // Vitesse moyenne en m/s
    if (elapsed_time > 0) { // /!\ Éviter la division par zéro
        average_speed = distance_covered / elapsed_time;
    } else {
        average_speed = 0;
    }
}

// Fonction pour terminer la course et calculer la vitesse moyenne finale
void endRace() {
    unsigned long race_end_time = millis();
    float race_duration = (race_end_time - race_start_time) / 1000.0;

    if (race_duration > 0) {
        average_speed = distance_covered / race_duration;
    } else {
        average_speed = 0;
    }

    race_mode = false;
    must_end_race_api_signal = true;
}

float getFinalAverageSpeed() {
    return average_speed;
}


// nombre de secondes durant lesquelles la voiture était hors du parcours
void updateOutOfParcours() {
    unsigned long current_time = millis();
    
    if (current_time - last_out_of_parcours_update_time >= 1000) {
        Track_Read();

        if (sensorValue[3] == 0 || sensorValue[3] == 7) {
            out_of_parcours++;
        }

        last_out_of_parcours_update_time = current_time;
    }
}

// nombre de secondes durant lesquelles la voiture était en collision
void updateCollisionDurationTime() {
        unsigned long current_time = millis();

        if (current_time - last_collision_duration_update_time >= 1000) {
            
            last_collision_duration_update_time = current_time; // mise à jour du temps de la dernière vérification

            float distance = Get_Sonar();

            if (distance < 25.0)
            {
                collision_duration++;
                
            }
        }
}

char* generateRaceTopic(char* buffer, size_t bufferSize, int raceId, const char* suffix) {
    // Générer le topic et le stocker dans le buffer fourni
    snprintf(buffer, bufferSize, "esp32/races/%d/%s", raceId, suffix);
    
    return buffer;  // Retourner le buffer
}

void resetRaceDataToDefault() {
    previous_race_id = race_id;
    race_id = 0;
    race_mode = false;
    average_speed = 0;
    distance_covered = 0;
    collision_duration = 0;
    out_of_parcours = 0;

    distance_previously_covered = 0;
    last_distance_covered_update_time = 0;
    last_out_of_parcours_update_time = 0;
    last_collision_duration_update_time = 0;

    race_start_time = 0;
}
