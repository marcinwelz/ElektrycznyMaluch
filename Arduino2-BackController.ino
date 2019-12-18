#include <AM2320.h>
#include <LiquidCrystal_I2C.h>

  unsigned long button_time;
  uint16_t  battery_voltage;     //*100  
  uint16_t  circuit_voltage;     //*100  
  //battery_current,  
  //circuit_current,  
  float  engine_temp;          
  int16_t  engine_humidity;      //*10
  int16_t  driver_temp;          //*10
  byte  engine_fan;
  bool  hamulec;                    
  bool  driver_fan;          
  bool  silnik;    


// sensor setup
TwoWire twoWire;
AM2320 sensor(&twoWire);

#define RefreshTime             1000             
#define V120_VoltageRead_Pin    A0    
#define V12_VoltageRead_Pin     A1    
#define Fan_Control_Pin1        12      //D12
#define Fan_Control_Pin2        11      //D11
#define Fan_Control_Pin3        10      //D10
#define Button_Pin              13      //D13
//#define TemperatureRead_Pin     10
#define Reference_Value         5    //You can check it at pin "REF"

LiquidCrystal_I2C lcd(0x27,20,4);   

//List of Functions 
void showDataOnLcd (void);
void meassureVoltage (void);
void fanControl (byte mode);
byte automaticMode (void);
void buttonControl(void);


void setup()
{
  pinMode(Fan_Control_Pin1,OUTPUT);
  pinMode(Fan_Control_Pin2,OUTPUT);
  pinMode(Fan_Control_Pin3,OUTPUT);
  pinMode(Button_Pin,INPUT);
  engine_fan=4;

  twoWire.begin();          //is it necessary?
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
}

void loop() 
{
  sensor.Read();
  engine_temp=sensor.cTemp;
  meassureVoltage();
  showDataOnLcd();
  fanControl(engine_fan);
  buttonControl();
  delay(RefreshTime);
}

void showDataOnLcd (void)
{
  float output;
  lcd.setCursor(0,0); 
  lcd.print("CircuitVolt: ");
  output=circuit_voltage;
  lcd.print(output/100,2);  
  lcd.setCursor(19,0);
  lcd.print("V");

  lcd.setCursor(0,1); 
  lcd.print("BatteryVolt: ");
  output=battery_voltage;
  lcd.print(output/10,1);  
  lcd.setCursor(19,1);
  lcd.print("V");
  
  lcd.setCursor(0,2); 
  lcd.print("Engine_Temp: ");
  lcd.print(engine_temp,1);  
  lcd.setCursor(19,2);
  lcd.print("C");

  lcd.setCursor(0,3); 
  lcd.print("FanMode:");
  if(engine_fan==4)
    lcd.print("AUTO");
  else if(engine_fan==0)
  {
    lcd.print("off");
  }
  else
  {
  lcd.setCursor(9,3); 
  lcd.print(engine_fan);
  }
  
  if(button_time)
  {
  lcd.setCursor(14,3);
  lcd.print("button");
  }
}

void  meassureVoltage (void)
{
  float p,P,R1,R2,X = 0;

          //  (V12) Circut Voltage
  p =analogRead(V12_VoltageRead_Pin);     // read pin (0 - 1023)
  P = (p*Reference_Value /1023);          // change value of p to voltage (0V - ~5V)
  R1 = 9850; 
  R2 = 4630; 
  X = (P* (R1+R2)/R2);                    // calculate X
  X*=100;
  circuit_voltage = X;                    // save value into circuit_voltage 
 
          //  (V120) Battery Voltage
  p =analogRead(V120_VoltageRead_Pin);
  P = (p*Reference_Value /1023); 
  R1 = 99570; 
  R2 = 3258; 
  X = (P* (R1+R2)/R2);
  X*=10;
  battery_voltage = X;
  Serial.print("  \n");
}

void fanControl (byte mode)
{
  switch(mode)
  {
    case 0:
      digitalWrite(Fan_Control_Pin1,HIGH);
      digitalWrite(Fan_Control_Pin2,HIGH);
      digitalWrite(Fan_Control_Pin3,HIGH);
      break;
    case 1:
      digitalWrite(Fan_Control_Pin1,LOW);
      digitalWrite(Fan_Control_Pin2,HIGH);
      digitalWrite(Fan_Control_Pin3,HIGH);
      break;
    case 2:
      digitalWrite(Fan_Control_Pin1,HIGH);
      digitalWrite(Fan_Control_Pin2,LOW);
      digitalWrite(Fan_Control_Pin3,HIGH);
      break;
    case 3:
      digitalWrite(Fan_Control_Pin1,HIGH);
      digitalWrite(Fan_Control_Pin2,HIGH);
      digitalWrite(Fan_Control_Pin3,LOW);
      break;
    case 4:
      fanControl (automaticMode());
      break;
  }
}

byte automaticMode()
{
  if(engine_temp>65)
    return 3;
  else if (engine_temp>45)
    return 2;
  else if (engine_temp>25)
    return 1;
  else 
    return 0; 
}

void buttonControl()
{
  switch (digitalRead(Button_Pin))
  {
  case LOW:
    if (button_time)                  // button is bening held
    {
      return;                         // do nothing  
    }                 
    else                              // button has been clicked
    {
      button_time=millis();           // check and save time
      break;
    }
  case HIGH:
    if(button_time)                   // button has been relesed
    {                                 
      if((millis()-button_time)>1500)   // check how long has been held
      {
        engine_fan=4;                 // more than 1.5 sec
      }
      else
      {
        engine_fan++;                 // less than 1.5 sec
        if(engine_fan==5)
          engine_fan=0;
      }
      
      button_time=0;                  // set button free!!!
      break;
    }
    else                              // nothing is happening
    {
      return;                         // do nothing 
    }
  }
}
