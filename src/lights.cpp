
#include "Particle.h"
#include "Arduino.h"
#include "lights.h"
#include <neopixel.h>

unsigned long breathPrevMillis = 0;
int breathBrightness = 0;
bool breathIncreasing = true;
/*
 * LIGHT STUFF
 */
Adafruit_NeoPixel ringLight(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);

void setLightRed()
{
    setLight(0xff, 0x00, 0x00);
}
void setLightBreathingRed()
{
    setLightBreathColor(LED_MIN_BRIGHT, LED_BRIGHT, 0xff, 0x00, 0x00);
}
void setLightGreen()
{
    setLight(0x00, 0xff, 0x00);
}
void setLightBreathingGreen()
{
    setLightBreathColor(LED_MIN_BRIGHT, LED_BRIGHT, 0x00, 0xff, 0x00);
}

void setLightBlue()
{
    setLight(0x00, 0x00, 0xff);
}
void setLightBreathingBlue()
{
    setLightBreathColor(LED_MIN_BRIGHT, LED_BRIGHT, 0x00, 0x00, 0xff);
}
void setLightPurple()
{
    setLight(0xff, 0x00, 0xff);
}

void turnOffLight()
{
    setLight(0x00, 0x00, 0x00);
}

void blinkCANError()
{
    // Log.info("CAN error!");
    setLightRed();
    delay(100);
    turnOffLight();
    delay(100);
}
void blinkCANBusError()
{
    setLightRed();
    delay(50);
    turnOffLight();
    delay(50);
}
void blinkIdentityError()
{
    Log.error("Identitiy error");
    for (int i = 1; i <= 10; i++)
    {
        setLightBlue();
        delay(500);
        turnOffLight();
        delay(100);
    }
}
void setLight(byte red, byte green, byte blue)
{
    for (int i = 0; i < PIXEL_COUNT; i++)
    {
        ringLight.setColorDimmed(i, red, green, blue, LED_BRIGHT);
    }
    ringLight.show();
}
void setLightBright(int bright, byte red, byte green, byte blue)
{
    for (int i = 0; i < PIXEL_COUNT; i++)
    {
        ringLight.setColorDimmed(i, red, green, blue, bright);
    }
    ringLight.show();
}

void setLightBreathColor(int min_bright, int max_bright, byte red, byte green, byte blue)
{
    unsigned long currentMillis = millis();
    const unsigned long breathInterval = 20; // ms between brightness steps

    if (currentMillis - breathPrevMillis >= breathInterval)
    {
        breathPrevMillis = currentMillis;

        // Update brightness
        if (breathIncreasing)
        {
            breathBrightness++;
            if (breathBrightness >= max_bright)
            {
                breathBrightness = max_bright;
                breathIncreasing = false; // start dimming
            }
        }
        else
        {
            breathBrightness--;
            if (breathBrightness <= min_bright)
            {
                breathBrightness = min_bright;
                breathIncreasing = true; // start brightening
            }
        }

        // Apply the brightness
        setLightBright(breathBrightness, red, green, blue);
    }
}

void beginLight()
{
    ringLight.begin();
}
