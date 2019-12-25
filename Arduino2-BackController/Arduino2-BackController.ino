#include <AM2320.h>
#include <LiquidCrystal_I2C.h>

//data
unsigned long button_click_time;
uint16_t  battery_voltage;     //*100  
uint16_t  circuit_voltage;     //*100  
//battery_current,  
//circuit_current,  
float  engine_temp;          
float  engine_humidity;      //*10
float  driver_temp;          //*10
byte  engine_fan;
bool  brake;                    
bool  driver_fan;          
bool  engine;   
bool  back_light; 

// setup
TwoWire twoWire;
AM2320 sensor(&twoWire);
LiquidCrystal_I2C lcd(0x27,20,4);   

// constants
#define RefreshTime             400            
#define V120_VoltageRead_Pin    A0    
#define V12_VoltageRead_Pin     A1    
#define Fan_Control_Pin1        12      //D12
#define Fan_Control_Pin2        10      //D11
#define Fan_Control_Pin3        9      //D10
#define Light_Control_Pin       8
#define Button_Pin              13      //D13
//#define TemperatureRead_Pin     10
#define Reference_Value         5    //You can check it at pin "REF"

//List of Functions 
void lcdSetup (void);
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
  pinMode(Light_Control_Pin,OUTPUT);
  digitalWrite(Light_Control_Pin,HIGH);
  
  back_light = 1;
  engine_fan=4;
  button_click_time=0;

  twoWire.begin();          //is it necessary?
  Serial.begin(38400);
  lcdSetup();
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

void lcdSetup (void)
{
  lcd.init();
  lcd.backlight();
  
  lcd.setCursor(0,0); 
  lcd.print("CircuitVolt: ");
  lcd.setCursor(19,0);
  lcd.print("V");

  lcd.setCursor(0,1); 
  lcd.print("BatteryVolt: ");
  lcd.setCursor(19,1);
  lcd.print("V");

  lcd.setCursor(0,2); 
  lcd.print("Engine_Temp: ");
  lcd.setCursor(19,2);
  lcd.print("C");

  lcd.setCursor(0,3); 
  lcd.print("FanMode:");
  
}

void showDataOnLcd (void)
{
  float output;

  lcd.setCursor(13,0);
  output=circuit_voltage;
  lcd.print(output/100,2);  
  

  lcd.setCursor(13,1);
  output=battery_voltage;
  lcd.print(output/10,1);  
  
  lcd.setCursor(14,2);
  lcd.print(engine_temp,1); 
  if(engine_temp<10000)
    lcd.print(" "); 
  

  lcd.setCursor(8,3); 
  if(engine_fan==4)
    lcd.print("AUTO");
  else if(engine_fan==0)
  {
    lcd.print("off ");
  }
  else
  {
    lcd.print(" ");
    lcd.print(engine_fan);
    lcd.print("  ");
  }
  
  if(button_click_time)
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
    default:
      engine_fan = 0;
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
    if (button_click_time)                  //button is released
    {
      if((millis()-button_click_time)>1500)   // more than 1.5 sec
        {
          if(back_light)
          {
            digitalWrite(Light_Control_Pin,LOW);
            back_light=true;
          }
          else
          {
            digitalWrite(Light_Control_Pin,HIGH);
            back_light=false;
          }
        }
        else                                  // less than 1.5 sec
        {
          engine_fan++;                 
          if(engine_fan==5)
            engine_fan=0;
        }
        
      button_click_time=0;                  // set button free!!!
      return;                        
    }                 
    else                                    //button is not clicked
    {
      return;
    }
    
  case HIGH:
    if(button_click_time)                   // button is held
    {                                 
      return;
    }
    else                                    // button has been clicked
    {
      button_click_time=millis();
    }
  }
}
