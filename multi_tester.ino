#include <LiquidCrystal.h>                      //input_pullup info   https://www.arduino.cc/reference/en/language/variables/constants/constants/
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);          //digitalRead info    https://www.arduino.cc/en/Tutorial/DigitalReadSerial
                                                //TinkerCad
int rPin = A0;
int raw = 0;
int Vin = 5;
float Vout = 0;
float R1 = 1000;
float R2 = 0;
float buffer = 0;

int cPinOut = A2;
int cPinIn = A1;
float IN_STRAY_CAP_TO_GND = 24.48;
float IN_CAP_TO_GND  = IN_STRAY_CAP_TO_GND;
float R_PULLUP = 34.8;  
int MAX_ADC_VALUE = 1023;

int ledPin = 6;
int npnPin = 7;
int pnpPin = 8;
int scrPin = 9;
int chipPin = 10;

void setup()
{
  pinMode(ledPin, INPUT_PULLUP);
  pinMode(npnPin, INPUT_PULLUP);
  pinMode(pnpPin, INPUT_PULLUP);
  pinMode(scrPin, INPUT_PULLUP);
  pinMode(chipPin, INPUT_PULLUP);
  
  pinMode(cPinOut, OUTPUT);
  pinMode(cPinIn, OUTPUT);
  lcd.begin(16, 2);
  Serial.begin(9600);
}

void loop()
{
  Serial.println(digitalRead(npnPin));
  pinMode(cPinIn, INPUT);
  digitalWrite(cPinOut, HIGH);
  int val = analogRead(cPinIn);
  digitalWrite(cPinOut, LOW);
  pinMode(cPinIn, OUTPUT);
  //Serial.println(analogRead(val));
  
  if (analogRead(rPin) == 0)
  {
  if (val < 1000)
  {
    float capacitance = (float)val * IN_CAP_TO_GND / (float)(MAX_ADC_VALUE - val);
    if (capacitance > 40.0)
    {
      lcd.setCursor(0,0);
      lcd.clear();
      lcd.print("Capacitance = ");
      
      lcd.setCursor(0,1);
      lcd.print(capacitance, 3);
      lcd.print("pF ");
      lcd.print(val);
      lcd.print("mS");
    }
    else
    {
      lcd.setCursor(0,0);
      lcd.print("Insert resistor");
      lcd.setCursor(0,1);
      lcd.print("or capacitor");
    }
  }
    
  else
  {
    delay(1);
    pinMode(cPinOut, INPUT_PULLUP);
    unsigned long u1 = micros();
    unsigned long t;
    int digVal;

    do
    {
      digVal = digitalRead(cPinOut);
      unsigned long u2 = micros();
      t = u2 > u1 ? u2 - u1 : u1 - u2;
    }
      
    while ((digVal < 1) && (t < 400000L));

    pinMode(cPinOut, INPUT);
    val = analogRead(cPinOut);
    digitalWrite(cPinIn, HIGH);
    int dischargeTime = (int)(t / 1000L) * 5;
    delay(dischargeTime);
    pinMode(cPinOut, OUTPUT);
    digitalWrite(cPinOut, LOW);
    digitalWrite(cPinIn, LOW);

    float capacitance = -(float)t / R_PULLUP / log(1.0 - (float)val / (float)MAX_ADC_VALUE);

    lcd.setCursor(0,0);
    lcd.clear();
    lcd.print("Capacitance = ");
    
    lcd.setCursor(0,1);
    lcd.print(capacitance / 1000.0, 3);
    lcd.print("uF ");
    lcd.print(val);
    lcd.print("mS");
  }
  while (millis() % 1000 != 0);
  }
  else
  {
    raw = analogRead(rPin);
    if(raw) 
    {
      buffer = raw * Vin;
      Vout = (buffer)/1024.0;
      buffer = (Vin/Vout) -1;
      R2 = R1 * buffer;

      {
        lcd.setCursor(0, 0);
        lcd.clear();
        lcd.print("Vout: ");
        lcd.print(Vout);

        lcd.setCursor(0, 1);
        lcd.print("R: ");
        lcd.print(R2);
        delay(1000);
      }
    }
  }
}
//int npnPin = 2;
//int pnpPin = 3;
//int ledPin = 4;
//int chipPin = 5;
//int state = 0;
//
//void setup()
//{
//  pinMode(npnPin, INPUT_PULLUP);
//  pinMode(pnpPin, INPUT_PULLUP);
//  pinMode(ledPin, INPUT_PULLUP);
//  pinMode(chipPin, INPUT_PULLUP);
//  Serial.begin(9600);
//}
//
//void loop()
//{
//  Serial.print("npn: ");
//  Serial.println(digitalRead(npnPin));
//  Serial.print("pnp: ");
//  Serial.println(digitalRead(pnpPin));
//  Serial.print("LED: ");
//  Serial.println(digitalRead(ledPin));
//  if (digitalRead(chipPin))
//  {
//    state = 1;
//  }
//  Serial.print("555: ");
//  Serial.println(state);
//  delay(1000);
//}
