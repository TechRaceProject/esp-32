#include <Arduino.h>
#include "Freenove_4WD_Car_For_ESP32.h"
#include "Freenove_4WD_Race_Data.h"

// Define if the car is in race mode. Golang API will use this to update a race model
int race_id = 0;
bool race_mode = false;
float distance_covered = 0;
unsigned long last_update_time = 0;

float distance_previously_covered = 0;


// int average_speed = 0;
// int number_of_collisions = 0;
// int count_of_out_of_track = 0;

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
    if (current_time - last_update_time >= 1000) {
        float speed = getConstantSpeed(); // Vitesse en m/s
        
        // Mettre à jour la distance parcourue
        distance_covered += speed; // Ajouter la distance parcourue en une seconde
        
        // Mettre à jour le temps de la dernière mise à jour
        last_update_time = current_time;

        return;
    }
}

char* generateRaceTopic(char* buffer, size_t bufferSize, int raceId, const char* suffix) {
    // Générer le topic et le stocker dans le buffer fourni
    snprintf(buffer, bufferSize, "esp32/races/%d/%s", raceId, suffix);
    
    return buffer;  // Retourner le buffer
}

void resetRaceDataToDefault() {
    race_id = 0;
    race_mode = false;
    distance_covered = 0;
    last_update_time = 0;

    distance_previously_covered = 0;
}
