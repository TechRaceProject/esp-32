#include <Arduino.h>
#include "Freenove_4WD_Car_For_ESP32.h"
#include "Freenove_4WD_Car_WS2812.h"

void HandleWebSocketDisconnection(int client_id)
{
    Motor_Move(0, 0, 0, 0);

    WS2812_SetMode(3);

    WS2812_Set_Color_1(4095, 255, 0, 0);

    Serial.printf("WebSocket client #%u disconnected\n", client_id);
}
