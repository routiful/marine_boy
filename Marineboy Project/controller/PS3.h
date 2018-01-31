#include <PS3BT.h>
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>







void PS3Begin();
void PS3TaskOn();
bool PS3Available();

uint8_t PS3GetJoy(AnalogHatEnum dir);
uint8_t PS3GetAnalogBtn(ButtonEnum btn);
uint8_t PS3GetBtn(ButtonEnum btn);

void PS3Rumble(bool onoff);
void PS3LedOn(LEDEnum led);
void PS3LedOff();

void PS3Print();
