#include <Servo.h>
#include <Wire.h>
#include "MS5837.h"

MS5837 sensor;

Servo AL, VRL, VRR, HRL, HRR, AR, VHR, HHR;             // Servo-Objekte für jeden ESC
int ALPin = 24, VRLPin = 25, HRLPin = 27, ARPin = 29;  // Die Pins für AL, VRL, HRL, und AR
int VHRPin = 26, HHRPin = 28;                         // Die Pins für VHR und HHR
int rcsignalPin = 5;                                 // Der Pin, an dem das RC-Signal für AL und AR angeschlossen ist
int rcsignalPin2 = 6;                               // Der Pin, an dem das RC-Signal für VHR und HHR angeschlossen ist
int rcsignalPin3 = 7;                              // Der Pin, an dem das RC-Signal für VRL angeschlossen ist
int rcsignalPin4 = 4;                             // Der Pin, an dem das RC-Signal für HRL angeschlossen ist


void setup() {
 

  // Initialisiere die Relaispins als OUTPUT und schalte sie aus
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);         // Auslass vorne
  pinMode(16, OUTPUT);        // Auslass Hinten
  pinMode(17, OUTPUT);       //Einlass Hinten
  pinMode(18, OUTPUT);      // Haupteinlass 
  pinMode(19, OUTPUT);     // Einlass Vorne
  pinMode(10, OUTPUT);    // Greifer ZU
  pinMode(11, OUTPUT);   // Greifer AUF
   
  AL.attach(ALPin);          // ESC an Pin 24 als AL
  VRL.attach(VRLPin);       // ESC an Pin 25 als VRL
  HRL.attach(HRLPin);      // ESC an Pin 27 als HRL
  AR.attach(ARPin);       // ESC an Pin 29 als AR
  VHR.attach(VHRPin);    // ESC an Pin 26 als VHR
  HHR.attach(HHRPin);   // ESC an Pin 28 als HHR

    Serial.begin(9600);
  Serial.println("Starting");
  Wire.begin();

  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    }

   sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
  
 

  digitalWrite(14, HIGH);            // Relais 14 ausschalten
  digitalWrite(15, HIGH);           // Relais 15 ausschalten
  digitalWrite(16, HIGH);          // Relais 16 ausschalten
  digitalWrite(17, HIGH);         // Relais 17 ausschalten
  digitalWrite(18, HIGH);        // Relais 18 ausschalten
  digitalWrite(19, HIGH);       // Relais 19 ausschalten
  digitalWrite(10, HIGH);      // Relais 20 ausschalten
  digitalWrite(11, HIGH);     // Relais 21 ausschalten
}




void loop() {

  int rcSignal_AL_AR = pulseIn(rcsignalPin, HIGH, 25000);      // Lese das RC-Signal für AL und AR ein
  int rcSignal_VHR_HHR = pulseIn(rcsignalPin2, HIGH, 25000);  // Lese das RC-Signal für VHR und HHR ein
  int rcSignal_VRL = pulseIn(rcsignalPin3, HIGH, 25000);     // Lese das RC-Signal für VRL ein
  int rcSignal_HRL = pulseIn(rcsignalPin4, HIGH, 25000);    // Lese das RC-Signal für HRL ein
  int LPV;                                                  // abgespeicherter Druckwert, wenn die Höhensteuerung zuletzt betätigt wurde
  int differenzkleinAssistenz = 0.5 ;    // legt fest ab welcher differenz die hhr und vhr motoren angeschalten werden
  int differenzgrossAssistenz = 1;       // legt fest ab welcher differenz die tauchzellen angeschalten werden - gross mit zwei s geschrieben da scharfes s nicht kompiliert 

  sensor.read();


  if( rcSignal_VHR_HHR < 1200 && -1.25  > (sensor.depth())  ||  rcSignal_VHR_HHR > 1700  && -1  > (sensor.depth()) ){ // Wenn RC-Signal aktiv hoch oder runter  und Uboot mehr als 1 Meter Unterwasser 

LPV = (sensor.depth()) ;                // sichere den aktuellen Druckwert als LPV (Last Pressure Value)
    }
  

     if (rcSignal_HRL >= 600 &&  rcSignal_HRL <= 1200) {
   HRL.writeMicroseconds(1000);
  
  } else if (rcSignal_HRL > 1800) {
    HRL.writeMicroseconds(2000);
  } else {
    HRL.writeMicroseconds(1500); 
  }
  
  
  if (rcSignal_AL_AR >= 900 && rcSignal_AL_AR <= 1300) {    // Steuere AL und AR basierend auf dem RC-Signal
    AL.writeMicroseconds(1000);
    AR.writeMicroseconds(1000);
  } else if (rcSignal_AL_AR > 1800) {
    AL.writeMicroseconds(2000);
    AR.writeMicroseconds(2000);
  } else {
    AL.writeMicroseconds(1500);
    AR.writeMicroseconds(1500);
  }
  
                                    
  if (rcSignal_VHR_HHR < 1200) {                            // Steuere VHR und HHR basierend auf dem RC-Signal
    VHR.writeMicroseconds(1000);
    HHR.writeMicroseconds(1000);
  } else if (rcSignal_VHR_HHR > 1700) {
    VHR.writeMicroseconds(2000);
    HHR.writeMicroseconds(2000);
  } else {
    VHR.writeMicroseconds(1500);
    HHR.writeMicroseconds(1500);
  }

   if (rcSignal_VRL >= 600 && rcSignal_VRL <= 1200) {
   VRL.writeMicroseconds(1000);
  
  } else if (rcSignal_VRL > 1700) {
    VRL.writeMicroseconds(2000);
  } else {
    VRL.writeMicroseconds(1500); 
  }

 int pulsewidths[6];

  pulsewidths[4] = pulseIn(8, HIGH);      // Equipment 
  pulsewidths[5] = pulseIn(9, HIGH);     // Tauchzellen
 

  int EquipmentValue = pulsewidths[4];
  int TauchzellenValue = pulsewidths[5];

     if (TauchzellenValue >= 700 && TauchzellenValue <= 1200) {
          digitalWrite(15, LOW);        // Relais 15 einschalten Auslass Ventil Vorne
          digitalWrite(16, LOW);       // Relais 16 einschalten Auslass Ventil hinten
    
          } else if (TauchzellenValue >= 1400 && TauchzellenValue <= 1600) {
                                                                               // Alle Relais zwischen 14 und 20 ausschalten
          for (int i = 14; i <= 19; i++) {
          digitalWrite(i, HIGH);
          }
          } else if (TauchzellenValue >= 1750 && TauchzellenValue <= 2000) {
          digitalWrite(17, LOW);         // Relais 17 einschalten Einlass Ventil hinten 
          digitalWrite(18, LOW);        // Relais 18 einschalten Haupteinlass 
          digitalWrite(19, LOW);       // Relais 19 einschalten Einlass Ventil vorne
    
           } else {
           // Alle Relais ausschalten, da die Pulsbreite von Kanal 6 nicht in den definierten Bereichen liegt
             for (int i = 14; i <= 19; i++) {
           digitalWrite(i, HIGH);
            }
             }

            if (EquipmentValue >= 1000  ) {
             digitalWrite(10, LOW);       // Relais 10 einschalten  Equipment 
             digitalWrite(11, HIGH);     // Relais 11 einschalten  Equipment 
              digitalWrite(18, LOW);    // Relais 18 einschalten Haupteinlass 
             } else {
             digitalWrite(10, HIGH);      // Relais 11 ausschalten Equipment geändert auch 10 und 11-davor 21 20 wegen sda scl belegung 
             digitalWrite(11, LOW);      // Relais 11 ausschalten Equipment
             digitalWrite(18, LOW);     // Relais 18 einschalten Haupteinlass  
       }

     if (rcSignal_VHR_HHR < 1700  && rcSignal_VHR_HHR > 1200  &&( LPV + differenzkleinAssistenz ) >(sensor.depth()) ){                    
    VHR.writeMicroseconds(1000);
    HHR.writeMicroseconds(1000);
  } else if (rcSignal_VHR_HHR < 1700  && rcSignal_VHR_HHR > 1200  && ( LPV - differenzkleinAssistenz ) <(sensor.depth()) ) {
    VHR.writeMicroseconds(2000);
    HHR.writeMicroseconds(2000);
  }
  else {
    VHR.writeMicroseconds(1500);
    HHR.writeMicroseconds(1500);
  }
//-------------------------------------------------------------------------------------------------------------------
     if (rcSignal_VHR_HHR < 1700  && rcSignal_VHR_HHR > 1200  &&( LPV + differenzgrossAssistenz ) >(sensor.depth()) ){                    
      digitalWrite(15, LOW);           // Relais 15 einschalten Auslass Ventil Vorne
      digitalWrite(16, LOW);          // Relais 16 einschalten Auslass Ventil hinten
      delay(1000);
      digitalWrite(15, HIGH);           // Relais 15 ausschalten Auslass Ventil Vorne
      digitalWrite(16, HIGH);          // Relais 16 ausschalten Auslass Ventil hinten
      
  } else if (rcSignal_VHR_HHR < 1700  && rcSignal_VHR_HHR > 1200  && ( LPV - differenzgrossAssistenz ) <(sensor.depth()) ) {
    digitalWrite(17, LOW);          // Relais 17 einschalten Einlass Ventil hinten 
    digitalWrite(18, LOW);         // Relais 18 einschalten Haupteinlass 
    digitalWrite(19, LOW);        // Relais 19 einschalten Einlass Ventil vorn
    delay (1000); 
    digitalWrite(17, HIGH);          // Relais 17 ausschalten Einlass Ventil hinten 
    digitalWrite(18, HIGH);         // Relais 18 ausschalten Haupteinlass 
    digitalWrite(19, HIGH);        // Relais 19 ausschalten Einlass Ventil vorn
  }
  else {
  digitalWrite(17, HIGH);              // Relais 17 einschalten Einlass Ventil hinten 
    digitalWrite(18, LOW);            // Relais 18 einschalten Haupteinlass 
    digitalWrite(19,  HIGH);         // Relais 19 einschalten Einlass Ventil vorne
      digitalWrite(15,  HIGH);      // Relais 15 einschalten Auslass Ventil Vorne
      digitalWrite(16,  HIGH);     // Relais 16 einschalten Auslass Ventil hinten
  }
      


  delay(1000 ); //nicht unter 1000 sonst laufen die motoren unkontroliert

}