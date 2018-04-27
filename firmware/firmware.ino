/**
 * Author:    Project-MAR
 * Created:   27 Apr. 2018
 **/

#include "DHT.h"
#include "Wire.h"
#include "LiquidCrystal_PCF8574.h"

/*
 * LCD Connection
 * Black  - GND
 * White  - Vcc
 * Gray   - SDA
 * Purpel - SCL
 */

 /*
  * Rotary Switch
  * Red    - Gnd
  * Orange - Vcc
  * Yellow - SW
  * Blue   - DT
  * Green  - Clk
  */
  
#define PushSW       5
#define RotaryA      6
#define RotaryB      7

#define DHT22_PIN    8

#define Relay4      12
#define Relay3      11
#define Relay2      10
#define Relay1       9
#define RelayON      LOW
#define RelayOFF     HIGH


#define CCW          0
#define CW           1
#define Push         2

// Connect a 4.7K resistor between VCC and the data pin (strong pullup)
DHT dht;

// set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_PCF8574 lcd(0x27);

int A;
int aState;
int RotateEvent;

float targetTemperature  = 25.0;
float currentTemperature = 25.0;

float targetHumidity  = 70.0;
float currentHumidity = 70.0;

char lcd_buffer[16];

int dht22_count = 0;
int dht22_target  = 5000;

int relayControl_count  = 0;
int relayControl_target = 500;

void setup(void)
{ 
  int lcd_error = 0;
  // start serial port
  Serial.begin(115200);

  dht.setup(DHT22_PIN); // data pin 2

  Wire.begin();
  Wire.beginTransmission(0x27);
  lcd_error = Wire.endTransmission();

  if (lcd_error == 0) 
  {
    //Serial.println(": LCD found.");
    lcd.begin(16, 2); // initialize the lcd
  } else {
    Serial.println(": LCD not found.");
  }

  // welcome message
  lcd.setBacklight(1);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("  System Start  ");
  lcd.setCursor(0, 1);
  lcd.print("   Project-MAR  ");

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Relay4, OUTPUT);
  pinMode(Relay3, OUTPUT);
  pinMode(Relay2, OUTPUT);
  pinMode(Relay1, OUTPUT);

  // Relay Module is active LOW
  digitalWrite(Relay1, HIGH);
  digitalWrite(Relay2, HIGH);
  digitalWrite(Relay3, HIGH);
  digitalWrite(Relay4, HIGH);
  
  pinMode(PushSW,  INPUT_PULLUP);
  pinMode(RotaryA, INPUT);
  pinMode(RotaryB, INPUT);

  aState = digitalRead(RotaryA);
  
  Serial.println("Status\tHumidity (%)\tTemperature (C)\t(F)");

  // The sensor can only be read from every 1-2s, and requires a minimum
  // 2s warm-up after power-on.
  delay(2000);
  lcd.clear();
}

void readDHT22()
{  
  //delay(dht.getMinimumSamplingPeriod());
  
  currentHumidity = dht.getHumidity();
  currentTemperature = dht.getTemperature();
  
  Serial.print(dht.getStatusString());
  Serial.print("\t");
  Serial.print(currentHumidity, 1);
  Serial.print("\t\t");
  Serial.print(currentTemperature, 1);
  Serial.print("\t\t");
  Serial.println(dht.toFahrenheit(currentTemperature), 1);
}

void setTargetHuminity()
{
  int a;
  //Serial.println("set Huminity");
  
  while(1)
  {
    a = Rotary();
    if(a == CW)
    {
      targetHumidity += 0.5;
      Serial.println("Increase target Humidity");
      show();
    }
    else if(a == CCW)
    {
      targetHumidity -= 0.5;
      Serial.println("Decrease target Humidity");
      show();
    }
    
    // if user push again, force exit
    if(digitalRead(PushSW) == LOW)
    {
      delay(500);
      Serial.println("Exit: set Huminity");
      break;
    }
  }
}

void show()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.setCursor(6, 0);
  lcd.print(currentTemperature,1);
  lcd.setCursor(10, 0);
  lcd.print("/");
  lcd.setCursor(11, 0);
  lcd.print(targetTemperature,1);
  lcd.setCursor(0, 1);
  
  lcd.setCursor(0, 1);
  lcd.print("Humi: ");
  lcd.setCursor(6, 1);
  lcd.print(currentHumidity,1);
  lcd.setCursor(10, 1);
  lcd.print("/");
  lcd.setCursor(11, 1);
  lcd.print(targetHumidity,1);
}

int Rotary()
{
  int result = -1;

  // Scan for Rotary Turn event
  A = digitalRead(RotaryA);
  if((aState == LOW) && (A == HIGH))
  {
    // Turn detect
    if(digitalRead(RotaryB) == LOW)
      result = CW;
    else
      result = CCW;
    
    delay(100);
  }
  aState = A;

  // Scan for Rotary push event
  if(digitalRead(PushSW) == LOW)
  {
    result = Push;
    delay(200);
  }

  return result;
}

//float targetTemperature  = 25.0;
//float currentTemperature = 25.0;

//float targetHumidity  = 70.0;
//float currentHumidity = 70.0;

void temperatureControl()
{
  static int coolingState = 0;
  
  // Fast Cooling: Turn OFF heater and Turn ON fan
  if((currentTemperature > targetTemperature) && (coolingState == 0))
  {
    //turn off heater and turn on fan
    coolingState = 1;
    digitalWrite(Relay4, RelayOFF);
    digitalWrite(Relay3, RelayON);    
  }

  // Turn off Fast Cooling if temperature is below (targetTemperature - 2)
  if((coolingState == 1) && (currentTemperature >= (targetTemperature - 2)))
  {
    digitalWrite(Relay4, RelayOFF);
    digitalWrite(Relay3, RelayOFF); 
  }

  // Start heating again when temperature drop below (targetTemperature - 5)
  if((coolingState == 1) && (currentTemperature <= (targetTemperature - 5)))
  {
    coolingState = 0;
    digitalWrite(Relay4, RelayON);
    digitalWrite(Relay3, RelayOFF); 
  }
}

void HumidityControl()
{
  
}
    
void loop(void)
{   
  
  if(relayControl_count >= relayControl_target)
  {
    relayControl_count = 0;
    temperatureControl();
    HumidityControl();
  }
  
  // Scan for Rotary turn event
  RotateEvent = Rotary();
  if(RotateEvent == CW)
  {
    Serial.println("Increase target Temperature");
    targetTemperature += 0.5;
    show();
  }
  else if(RotateEvent == CCW)
  {
    Serial.println("Decrease target Temperature");
    targetTemperature -= 0.5;
    show();
  }
  else if(RotateEvent == Push)
  {
    //Serial.println("PushSW");
    setTargetHuminity();
  }

  dht22_count += 1;
  relayControl_count += 1;
  delay(1);
}
