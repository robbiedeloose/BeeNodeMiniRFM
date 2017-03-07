#define DEBUG

void initHumidity(){
  for (size_t i = 0; i < 3; i++) {
    h = dht.readHumidity();
    t = dht.readTemperature();
      sensors.requestTemperatures(); // Send the command to get temperatures
    delay(2000);
  }
}

void readSensors() {

  //battery
  battery =  analogRead(sensorPin) * (3.3 / 1023.0);

  #ifdef DEBUG
    Serial.print("Battery: ");
    Serial.println(battery);
  #endif

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  t = dht.readTemperature();

  #ifdef DEBUG
    Serial.print("Humidity: ");
    Serial.println(h);
  #endif

  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  DEBUG_PRINT("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  DEBUG_PRINTLN("DONE");

  // After we got the temperatures, we can print them here.
  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
  temp1 = sensors.getTempCByIndex(0);
  if (temp1 == -127) {temp1 = 0;}
  temp2 = sensors.getTempCByIndex(1);
  if (temp2 == -127) {temp2 = 0;}
  temp3 = sensors.getTempCByIndex(2);
  if (temp3 == -127) {temp3 = 0;}
  temp4 = sensors.getTempCByIndex(3);
  if (temp4 == -127) {temp4 = 0;}
#ifdef DEBUG
  Serial.print("Temperature for the device 1 (index 0) is: ");
  Serial.println(temp1);
  Serial.print("Temperature for the device 2 (index 1) is: ");
  Serial.println(temp2);
  Serial.print("Temperature for the device 3 (index 2) is: ");
  Serial.println(temp3);
  Serial.print("Temperature for the device 4 (index 3) is: ");
  Serial.println(temp4);
#endif
}

void encodePayload() {

  int myVal = 0;

  myVal = temp1 * 100;
  mydata[0] = highByte(myVal);
  mydata[1] = lowByte(myVal);

  myVal = temp2 * 100;
  mydata[2] = highByte(myVal);
  mydata[3] = lowByte(myVal);

  myVal = temp3 * 100;
  mydata[4] = highByte(myVal);
  mydata[5] = lowByte(myVal);

  myVal = temp4 * 100;
  mydata[6] = highByte(myVal);
  mydata[7] = lowByte(myVal);

  myVal = h * 100;
  mydata[8] = highByte(myVal);
  mydata[9] = lowByte(myVal);

  mydata[10] = (battery * 100) - 150;

  mydata[11] = alarm;

}
