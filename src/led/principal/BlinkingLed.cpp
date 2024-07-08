#include <Arduino.h>
#include "Freenove_4WD_Car_WS2812.h"

void BlinkingLed(int led_colors[4]) 
{
    WS2812_Set_Color_1(led_colors[0], led_colors[1], led_colors[2], led_colors[3]);
}
