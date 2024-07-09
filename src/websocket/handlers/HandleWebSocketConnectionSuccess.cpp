#include <Arduino.h>
#include "Freenove_4WD_Car_For_ESP32.h"
#include "Freenove_4WD_Car_WS2812.h"

void HandleWebSocketConnectionSuccess(int client_id, String client_ip)
{
    int leds_rgb_green[5] = {4095, 0, 255, 0};

    Motor_Move(0, 0, 0, 0);

    WS2812_SetMode(1);
        
    WS2812_Set_Color_1(4095, 0, 255, 0);

    Serial.printf("WebSocket client #%u connected from %s\n", client_id, client_ip.c_str());
}
