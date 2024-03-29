//Libaries
#include <Servo.h>
#include <Wire.h>
#include "MS5837.h"
#include <ArduinoJson.h>

#define ANALOG_IN_PIN A8
#define ANALOG_IN_PIN A7

 // Hardware 
MS5837 sensor;
Servo AL, VRL, VRR, HRL, HRR, AR, VHR, HHR;              // Servo-Objekte für jeden ESC

 //Pins für Motoren und Funk 
int ALPin = 24, VRLPin = 25, HRLPin = 27, ARPin = 29;  // Die Pins für AL, VRL, HRL, und AR
int VHRPin = 26, HHRPin = 28;                         // Die Pins für VHR und HHR
int rcsignalPin = 5;                                 // Der Pin, an dem das RC-Signal für AL und AR angeschlossen ist
int rcsignalPin2 = 6;                               // Der Pin, an dem das RC-Signal für VHR und HHR angeschlossen ist
int rcsignalPin3 = 7;                              // Der Pin, an dem das RC-Signal für VRL angeschlossen ist
int rcsignalPin4 = 4;                             // Der Pin, an dem das RC-Signal für HRL angeschlossen ist

float LPV; 
float adc_voltage1 = 0.0;
float in_voltage1 = 0.0; // zu übertragende Spannung 1
float R1_1 = 30000.0;
float R2_1 = 7500.0; 
float ref_voltage1 = 5.0;
int adc_value1 = 0;
float adc_voltage2 = 0.0;
float in_voltage2 = 0.0; // zu übertragende Spannung 2
// Floats for resistor values in divider (in ohms)
float R1_2 = 30000.0;
float R2_2 = 7500.0; 
// Float for Reference Voltage
float ref_voltage2 = 5.0; 
// Integer for ADC value
int adc_value2 = 0;

#define SerialTX Serial1


void setup() {

  Serial.begin(115200);
  SerialTX.begin(9600);

  Serial.println("Startet");
  Wire.begin();
 

  // Initialisiere die Relaispins als OUTPUT und schalte sie aus
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);       // Auslass vorne
  pinMode(16, OUTPUT);      // Auslass Hinten
  pinMode(17, OUTPUT);     //Einlass Hinten
  pinMode(18, OUTPUT);    // Haupteinlass 
  pinMode(19, OUTPUT);   // Einlass Vorne
  pinMode(10, OUTPUT);  // Greifer ZU
  pinMode(11, OUTPUT); // Greifer AUF
   
  AL.attach(ALPin);          // ESC an Pin 24 als AL
  VRL.attach(VRLPin);       // ESC an Pin 25 als VRL
  HRL.attach(HRLPin);      // ESC an Pin 27 als HRL
  AR.attach(ARPin);       // ESC an Pin 29 als AR
  VHR.attach(VHRPin);    // ESC an Pin 26 als VHR
  HHR.attach(HHRPin);   // ESC an Pin 28 als HHR

    while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
  
  digitalWrite(14, HIGH); // Relais 14 ausschalten
  digitalWrite(15, HIGH); // Relais 15 ausschalten
  digitalWrite(16, HIGH); // Relais 16 ausschalten
  digitalWrite(17, HIGH); // Relais 17 ausschalten
  digitalWrite(18, HIGH); // Relais 18 ausschalten
  digitalWrite(19, HIGH); // Relais 19 ausschalten
  digitalWrite(10, HIGH); // Relais 20 ausschalten
  digitalWrite(11, HIGH); // Relais 21 ausschalten
}


String prepData(float temp, float depth, float pressure, float alt, float v1, float v2){
  return "<temp:" + String(temp) + "|depth" + String(depth) + ">";
};

void loop() {

  int rcSignal_AL_AR = pulseIn(rcsignalPin, HIGH, 25000);      // Lese das RC-Signal für AL und AR ein
  int rcSignal_VHR_HHR = pulseIn(rcsignalPin2, HIGH, 25000);  // Lese das RC-Signal für VHR und HHR ein
  int rcSignal_VRL = pulseIn(rcsignalPin3, HIGH, 25000);     // Lese das RC-Signal für VRL ein
  int rcSignal_HRL = pulseIn(rcsignalPin4, HIGH, 25000);    // Lese das RC-Signal für HRL ein
  
  sensor.read();
  adc_value1 = analogRead(A7);
  adc_value2 = analogRead(A8);

  
  adc_voltage1  = (adc_value1 * ref_voltage1) / 1024.0;
  in_voltage1 = adc_voltage1*(R1_1+R2_1)/R2_1;
  Serial.print("Input Voltage = ");
  Serial.println(in_voltage1, 2);

  adc_voltage2  = (adc_value2 * ref_voltage2) / 1024.0;
  in_voltage2 = adc_voltage2*(R1_2+R2_2)/R2_2;
  Serial.print("Input Voltage = ");
  Serial.println(in_voltage2, 2);

  float pressure = sensor.pressure();
  float temp = sensor.temperature();
  float depth = sensor.depth();
  float alt = sensor.altitude();

  LPV = (sensor.depth());
  
  Serial.print("Pressure: "); 
  Serial.print(pressure); 
  Serial.println(" mbar");
  Serial.print("Temperature: "); 
  Serial.print(temp); 
  Serial.println(" deg C"); 
  Serial.print("Depth: "); 
  Serial.print(depth); 
  Serial.println(" m");
  Serial.print("Altitude: "); 
  Serial.print(alt); 
  Serial.println(" m above mean sea level");

  Serial.print("LPV:");
  Serial.println (LPV );
  StaticJsonDocument<200> payload;
  payload["temp"] = temp;
  payload["altitude"] = alt;
  payload["pressure"] = pressure;
  payload["depth"] = depth;
  payload["v1"] = in_voltage1;
  payload["v2"] = in_voltage2;
  serializeJson(payload, SerialTX);


  if (rcSignal_HRL >= 600 &&  rcSignal_HRL <= 1200) {
   HRL.writeMicroseconds(1000);
  
  } else if (rcSignal_HRL > 1800) {
    HRL.writeMicroseconds(2000);
  } else {
    HRL.writeMicroseconds(1500); 
  }
  
//-----------------------------------------//-----------------------------------------//-----------------------------------------//-----------------------------------------

  // Steuere AL und AR basierend auf dem RC-Signal
  if (rcSignal_AL_AR >= 900 && rcSignal_AL_AR <= 1300) {
    AL.writeMicroseconds(1000);
    AR.writeMicroseconds(1000);
  } else if (rcSignal_AL_AR > 1800) {
    AL.writeMicroseconds(2000);
    AR.writeMicroseconds(2000);
  } else {
    AL.writeMicroseconds(1500);
    AR.writeMicroseconds(1500);
  }
  
  // Steuere VHR und HHR basierend auf dem RC-Signal
  if (rcSignal_VHR_HHR < 1200) {
    VHR.writeMicroseconds(1000);
    HHR.writeMicroseconds(1000);
  } else if (rcSignal_VHR_HHR > 1700) {
    VHR.writeMicroseconds(2000);
    HHR.writeMicroseconds(2000);
  } else {
    VHR.writeMicroseconds(1500);
    HHR.writeMicroseconds(1500);
  }
 //-----------------------------------------//-----------------------------------------//-----------------------------------------//-----------------------------------------

   if (rcSignal_VRL >= 600 && rcSignal_VRL <= 1200) {
   VRL.writeMicroseconds(1000);
  
  } else if (rcSignal_VRL > 1700) {
    VRL.writeMicroseconds(2000);
  } else {
    VRL.writeMicroseconds(1500); 
  }

//-----------------------------------------//-----------------------------------------//-----------------------------------------//-----------------------------------------

 int pulsewidths[6];

  pulsewidths[4] = pulseIn(8, HIGH); // Equipment 
  pulsewidths[5] = pulseIn(9, HIGH); // Tauchzellen
 

  int EquipmentValue = pulsewidths[4];
  int TauchzellenValue = pulsewidths[5];

     if (TauchzellenValue >= 700 && TauchzellenValue <= 1200) {
          digitalWrite(15, LOW); // Relais 15 einschalten Auslass Ventil Vorne
          digitalWrite(16, LOW); // Relais 16 einschalten Auslass Ventil hinten
    
          } else if (TauchzellenValue >= 1400 && TauchzellenValue <= 1600) {
          // Alle Relais zwischen 14 und 20 ausschalten
          for (int i = 14; i <= 19; i++) {
          digitalWrite(i, HIGH);
          }
          } else if (TauchzellenValue >= 1750 && TauchzellenValue <= 2000) {
            
          digitalWrite(17, LOW); // Relais 17 einschalten Einlass Ventil hinten 
          digitalWrite(18, LOW); // Relais 18 einschalten Haupteinlass 
          digitalWrite(19, LOW); // Relais 19 einschalten Einlass Ventil vorne
    
           } else {
           // Alle Relais ausschalten, da die Pulsbreite von Kanal 6 nicht in den definierten Bereichen liegt
             for (int i = 14; i <= 19; i++) {
           digitalWrite(i, HIGH);
            }
             }

            if (EquipmentValue >= 1000  ) {
             digitalWrite(10, LOW); // Relais 10 einschalten  Equipment 
             digitalWrite(11, HIGH); // Relais 11 einschalten  Equipment 
              digitalWrite(18, LOW); // Relais 18 einschalten Haupteinlass 
             } else {
             digitalWrite(10, HIGH); // Relais 10 ausschalten Equipment 
             digitalWrite(11, LOW); // Relais 11 ausschalten Equipment
             digitalWrite(18, LOW); // Relais 18 einschalten Haupteinlass  
       }


  delay(100 ); 

}