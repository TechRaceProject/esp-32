#ifndef Freenove_4WD_Race_Data
#define Freenove_4WD_Race_Data

#include <Arduino.h>
#include "Freenove_4WD_Car_For_ESP32.h"

extern int race_id; // ID de la course en cours
extern bool race_mode; // Indique si le mode course est actif
extern bool must_end_race_api_signal; // Indique si la course doit être arrêtée au niveau de l'API Golang
extern float average_speed; // Vitesse moyenne du vehicule en m/s
extern float distance_covered; // Distance parcourue en mètres
extern int collision_duration; // Nombre de collisions
extern int out_of_parcours; // Nombre de sorties de route
extern unsigned long last_distance_covered_update_time; // Temps de la dernière mise à jour de distance
extern unsigned long last_out_of_parcours_update_time; // Temps de la dernière mise à jour de sortie de route
extern unsigned long last_collision_duration_update_time; // Temps de la dernière mise à jour de collision
extern unsigned long race_start_time; // Temps de début de la course
extern float distance_previously_covered; // Indique si la distance parcourue a été mise à jour
extern int previous_race_id; // ID de la course précédente


float getConstantSpeed(); // Fonction pour calculer la vitesse constante
void updateDistanceCovered(); // Fonction pour mettre à jour la distance parcourue
void updateAverageSpeed(); // Fonction pour calculer la vitesse moyenne du vehicule
void updateOutOfParcours(); // Fonction pour calculer le nombre de sorties de route
void updateCollisionDurationTime(); // Fonction pour calculer le nombre de collisions
char* generateRaceTopic(char* buffer, size_t bufferSize, int raceId, const char* suffix); // Fonction pour générer un topic
void resetRaceDataToDefault(); // Fonction pour réinitialiser les données de course

#endif // Freenove_4WD_Race_Data
