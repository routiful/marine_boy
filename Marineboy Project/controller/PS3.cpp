# include "PS3.h"

static USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside

static BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */
//PS3BT PS3(&Btd); // This will just create the instance
// static PS3BT PS3(&Btd, 0x00, 0x19, 0x86, 0x00, 0x20, 0xDD); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch
PS3BT PS3(&Btd, 0x00, 0x19, 0x86, 0x00, 0x21, 0xF3); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch

void PS3Begin()
{
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
}

void PS3End()
{
  PS3.disconnect();
}

void PS3TaskOn()
{
  Usb.Task();
}

bool PS3Available()
{
  return (PS3.PS3Connected || PS3.PS3NavigationConnected);
}

uint8_t PS3GetJoy(AnalogHatEnum dir)
{
  return PS3.getAnalogHat(dir);
}

uint8_t PS3GetAnalogBtn(ButtonEnum btn)
{
  return PS3.getAnalogButton(btn);
}

uint8_t PS3GetBtn(ButtonEnum btn)
{
  return PS3.getButtonClick(btn);
}

void PS3Rumble(bool onoff)
{
  if (onoff)
    PS3.setRumbleOn(RumbleHigh);
  else
    PS3.setRumbleOff();
}

void PS3LedOn(LEDEnum led)
{
  PS3.setLedOn(led);
}

void PS3LedOff()
{
  PS3.setLedOff();
}

void PS3Print()
{
  Serial.println("SELECT   : "  + String(PS3GetBtn(SELECT))    + " " +
                 "LeftHatX : "  + String(PS3GetJoy(LeftHatX))  + " " +
                 "LeftHatY : "  + String(PS3GetJoy(LeftHatY))  + " " +
                 "RightHatX: "  + String(PS3GetJoy(RightHatX)) + " " +
                 "RightHatY: "  + String(PS3GetJoy(RightHatY)) + " " +
                 "TRIANGLE : "  + String(PS3GetBtn(TRIANGLE))  + " " +
                 "SQUARE   : "  + String(PS3GetBtn(SQUARE))    + " " +
                 "CROSS    : "  + String(PS3GetBtn(CROSS))     + " " +
                 "CIRCLE   : "  + String(PS3GetBtn(CIRCLE))    + " " +
                 "UP       : "  + String(PS3GetBtn(UP))        + " " +
                 "LEFT     : "  + String(PS3GetBtn(LEFT))      + " " +
                 "DOWN     : "  + String(PS3GetBtn(DOWN))      + " " +
                 "RIGHT    : "  + String(PS3GetBtn(RIGHT))     + " " +
                 "L1       : "  + String(PS3GetBtn(L1))        + " " +                     
                 "R1       : "  + String(PS3GetBtn(R1)) 
                 );
}