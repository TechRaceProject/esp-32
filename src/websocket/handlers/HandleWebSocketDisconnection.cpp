#include <Arduino.h>
#include "Freenove_4WD_Car_For_ESP32.h"
#include <Freenove_4WD_Led.h>

void HandleWebSocketDisconnection(int client_id)
{
    int leds_rgb_red[5] = {4095, 255, 0, 0};

    Motor_Move(0, 0, 0, 0);

    BlinkingLed(leds_rgb_red);

    Serial.printf("WebSocket client #%u disconnected\n", client_id);
}
