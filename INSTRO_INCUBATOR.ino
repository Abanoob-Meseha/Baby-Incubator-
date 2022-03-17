#include <LiquidCrystal.h>
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

char t;
//-----------------------------MIC INSTANCES---------------------------------------------
const int OUT_PIN = 8;
const int SAMPLE_TIME = 10;
unsigned long millisCurrent;
unsigned long millisLast = 0;
unsigned long millisElapsed = 0;
int sampleBufferValue = 0;
int timeCounter=0;
const int LED = 7;
//-----------------------------FSR INSTANCES-------------------------------------------
int fsrPin = A0;     // the FSR and 10K pulldown are connected to a0
int fsrReading;     // the analog reading from the FSR resistor divider
int fsrVoltage;     // the analog reading converted to voltage
unsigned long fsrResistance;  // The voltage converted to resistance
unsigned long fsrConductance; 
long fsrForce;       // Finally, the resistance converted to force
float babyWeight;
//------------------------------FUNCTIONS-----------------------------------------
void MIC_READ();
void WEIGHT_MEASURE();

//-------------------------------SETUP---------------------------------------
void setup() {
  pinMode(LED,OUTPUT);
  lcd.begin(16, 2);
  Serial.begin(9600);
}

//-------------------------------LOOP---------------------------------------
void loop() {

 if(Serial.available()){
  t = Serial.read();
}
 
if(t == 'M'){            //move forward(all motors rotate in forward direction)
  MIC_READ();
}
else if(t == 'W'){            //move forward(all motors rotate in forward direction)
  WEIGHT_MEASURE();
}
  
  } 
  
//--------------------------------MIC FUNCTION------------------------------------------
void MIC_READ(){
  lcd.clear();
  if(digitalRead(OUT_PIN)==0){
    lcd.print("BABY IS ASLEEP");
    }
    if(analogRead(OUT_PIN)==1){
    lcd.print("BABY IS AWAKE");
    }
  digitalWrite(LED,digitalRead(OUT_PIN));
    Serial.println(digitalRead(OUT_PIN));
    Serial.print('\t');
  
    
}
//---------------------------------FSR FUNCTION------------------------------------------------------
void WEIGHT_MEASURE(){
    fsrReading = digitalRead(fsrPin);  
  // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
  fsrVoltage = map(fsrReading, 0, 1023, 0, 5000);
   
  if (fsrVoltage == 0) {
    Serial.println("No pressure");  
  } else {
    // The voltage = Vcc * R / (R + FSR) where R = 10K and Vcc = 5V
    // so FSR = ((Vcc - V) * R) / V        yay math!
    fsrResistance = 5000 - fsrVoltage;     // fsrVoltage is in millivolts so 5V = 5000mV
    fsrResistance *= 10000;                // 10K resistor
    fsrResistance /= fsrVoltage;
    fsrConductance = 1000000;           // we measure in micromhos so 
    fsrConductance /= fsrResistance;
    // Use the two FSR guide graphs to approximate the force
    if (fsrConductance <= 1000) {
      fsrForce = fsrConductance / 80;
      babyWeight=fsrForce/9.8;
      Serial.print("Force in Newtons: ");
      Serial.println(fsrForce);
      Serial.print("BABY WEIGHT: ");
      Serial.println(babyWeight);       
    } else {
      fsrForce = fsrConductance - 1000;
      fsrForce /= 30;
      babyWeight=fsrForce/9.8;
      Serial.print("Force in Newtons: ");
      Serial.println(fsrForce);  
      Serial.print("BABY WEIGHT: ");
      Serial.println(babyWeight); 
      lcd.clear();
      lcd.print("WEIGHT:");
      lcd.setCursor(9, 0);
      lcd.print(babyWeight);   
    }
  }
  Serial.println("--------------------");
  delay(1000);
    }
