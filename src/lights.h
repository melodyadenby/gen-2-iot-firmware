#ifndef LIGHTS_H
#define LIGHTS_H
// LED Definitions
#define LED 8
#define signalLED 8
#define LED_BRIGHT 255
#define LED_MIN_BRIGHT 230

// Pixel Definitions
#define PIXEL_PIN D6
#define PIXEL_COUNT 70
#define PIXEL_TYPE WS2812B

void setLightRed();
void setLightGreen();
void setLightBlue();
void turnOffLight();
void blinkCANError();
void blinkIdentityError();
void setLight(byte red, byte green, byte blue);
void beginLight();
void blinkCANBusError();
void setLightBreathColor(int min_bright, int max_bright, byte red, byte green, byte blue);
void setLightBright(int bright, byte red, byte green, byte blue);
void setLightBreathingBlue();
void setLightBreathingRed();
void setLightBreathingGreen();
#endif // LIGHTS_H
