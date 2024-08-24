#ifndef Freenove_4WD_Race_Data
#define Freenove_4WD_Race_Data

#include <Arduino.h>
#include "Freenove_4WD_Car_For_ESP32.h"

extern int race_id; // ID de la course en cours
extern bool race_mode; // Indique si le mode course est actif
extern float distance_covered; // Distance parcourue en mètres
extern unsigned long last_update_time; // Temps de la dernière mise à jour de distance
extern float distance_previously_covered; // Indique si la distance parcourue a été mise à jour

float getConstantSpeed(); // Fonction pour calculer la vitesse constante
void updateDistanceCovered(); // Fonction pour mettre à jour la distance parcourue
char* generateRaceTopic(char* buffer, size_t bufferSize, int raceId, const char* suffix); // Fonction pour générer un topic
void resetRaceDataToDefault(); // Fonction pour réinitialiser les données de course

#endif // Freenove_4WD_Race_Data
