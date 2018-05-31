int incomingByte = 0;   // for incoming serial data

void setup() 
{
  Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
  SerialBT2.begin(9600);
}

void loop() 
{
  // send data only when you receive data:
  if (SerialBT2.available() > 0) 
  {
    // read the incoming byte:
    incomingByte = SerialBT2.read();

    // say what you got:
    Serial.print("I received: ");
    Serial.println(incomingByte, DEC);
  }
}