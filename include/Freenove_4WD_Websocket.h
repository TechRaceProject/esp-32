#include <Arduino.h>

#ifndef Freenove_4WD_Websocket
#define Freenove_4WD_Websocket

void HandleWebSocketConnectionSuccess(int client_id, String client_ip);
void HandleWebSocketDisconnection(int client_id);

#endif