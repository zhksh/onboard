#include <Arduino.h>

#include <RCSwitch.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <util.h>
#include <debug.h>
#include <vals.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// PINS
#define TRANSMITTER_PIN 11

#define TEMPSENSOR0_PIN  4
#define TEMPSENSOR1_PIN  5


#define TESENSOR A0


RCSwitch sender = RCSwitch();

OneWire temp0Driver(TEMPSENSOR0_PIN);
DallasTemperature temp0Sensor(&temp0Driver);

OneWire temp1Driver(TEMPSENSOR1_PIN);
DallasTemperature temp1Sensor(&temp1Driver);





void transmit(long data) {

  long msg = addprefix(data, SENSOR_RC_PREFIX, SENSOR_RC_PREFIX_POS);
  Serial.print("raw:");
  Serial.print(dec2binWzerofill(msg, MSG_LENGTH));
  Serial.print(", encoded:");
  Serial.println(msg);
  sender.send(msg , MSG_LENGTH);
}


void setup() {
  Serial.begin(BAUD);
  // Serial.println("Sender setup");
  temp0Sensor.begin();
  temp1Sensor.begin();

  sender.enableTransmit(TRANSMITTER_PIN);
  sender.setRepeatTransmit(SENSOR_REPEAT_TRANSMISSION);
}

// void printToSerial(float tb, float temp0, float temp1, float aux){
//   Serial.print("DATA|");
//   Serial.print("tb");
//   Serial.print(":");
//   Serial.print(tb);
//   Serial.print(";");

//   Serial.print("tmp0");
//   Serial.print(":");
//   Serial.print(temp0);
//   Serial.print(";");

//   Serial.print("tmp1");
//   Serial.print(":");
//   Serial.print(temp1);
//   Serial.print(";");

//   Serial.print("aux");
//   Serial.print(":");
//   Serial.print(aux);

//   Serial.println("");
// }


float requestTemp(DallasTemperature sensor){
    float val;
    sensor.requestTemperatures(); // Send the command to get temperatures
    // Serial.println("DONE");
    // After we got the temperatures, we can print them here.
    // We use the function ByIndex, and as an example get the temperature from the first sensor only.
    val = sensor.getTempCByIndex(0);
    // Check if reading was successful
    // if(val != DEVICE_DISCONNECTED_C) 
    // {
    //   Serial.print("Temperature for the device 1 (index 0) is: ");
    //   Serial.println(val);
    // } 
    // else
    // {
    //   Serial.println("Error: Could not read temperature data");
    // }
    return val;
}

void loop() {

  // float sensorValue = digitalRead(TESENSOR);
  float tbval = analogRead(TESENSOR);
  float val0 = requestTemp(temp0Sensor);
  float val1 = requestTemp(temp1Sensor);
  long val0_cut = 23.3 * 10;
  long val1_cut = 23.8 * 10;
  // long val0_cut = val0 *-10;
  // long val1_cut = val1 *-10;

  // Serial.print("tbval:");
  // Serial.println(dec2binWzerofill(tbval, MSG_LENGTH));
  Serial.print("temp0:");
  Serial.println(val0_cut);
  // Serial.println(val0);
  // Serial.println(dec2binWzerofill(val0_cut, MSG_LENGTH));
  Serial.print("temp1:");
  Serial.println(val1_cut);
  // Serial.println(val1);
  // Serial.println(dec2binWzerofill(val1_cut, MSG_LENGTH));

  long raw = (long) tbval << TURB_POS;
  raw |=  val1_cut << TEMP1_POS; 
  raw |=  val0_cut << TEMP0_POS; 

  sender.setProtocol(1);

  transmit(raw);  
  delay(SENSOR_SEND_FRQ);
  
}

