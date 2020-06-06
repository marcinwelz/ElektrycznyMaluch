//    Version 3.2   //

//#define K_Debug

#include <KacperDebug.h>
#include <KacperPinControl.h>
#include <AM2320.h>
#include <LiquidCrystal_I2C.h>


#define V120_VoltageRead_Pin    A0    
#define V12_VoltageRead_Pin     A1  
#define EngineTemp_R            A6
#define EngineTemp_L            A7  
#define Fan_Control_Pin1        9       //Minimum Power
#define Fan_Control_Pin2        12      //Medium  Power
#define Fan_Control_Pin3        10      //Maximum Power
#define Light_Control_Pin       8
#define Button_Pin              13      //D13
#define REF                     4920    //mV

enum fanModeName
{
  off,
  minimum,
  medium,
  maximum,
  winter,
  normal,
  summer,
  custom
};

//functionts                          
inline void displaySetup (void);
inline void fanControl (uint8_t mode);        
uint8_t automaticMode (float,float,float);    
inline void buttonControl (unsigned long &Time);
inline void displayControl(void);

//objects
AM2320 SensorA(&Wire);
Button button(Button_Pin);
LiquidCrystal_I2C lcd(0x27,20,4);

//Variables     
unsigned long loop_saved_time =2000;    
bool          BackLight=0;
uint8_t       EngineFan_Mode=normal;
uint8_t       EngineFan_State=4;
int16_t       Engine_Temperature;
float         SensorAm_Temperature;
float         SensorAm_Humidity;
uint16_t      SensorL_Temperature;
uint16_t      SensorR_Temperature;
uint16_t      Circut_Voltage;
uint16_t      Battery_Voltage;

using namespace Debug;
void setup()
{
  Serial.begin(38400);
  Wire.begin();

  pinMode(Fan_Control_Pin1,OUTPUT);
  pinMode(Fan_Control_Pin2,OUTPUT);
  pinMode(Fan_Control_Pin3,OUTPUT);
  pinMode(Button_Pin,INPUT);
  pinMode(Light_Control_Pin,OUTPUT);
  digitalWrite(Light_Control_Pin,BackLight = 1);
  
  Get(summer);
}

void loop()
{ 
  unsigned long Time=millis();
  
  LoopSpeed(5000,Time);     
  buttonControl(Time);               
  fanControl(EngineFan_Mode);   
  
  if(SensorA.Read())
  { //SensorA_Error
    SensorA.cTemp=0;
    Debug::Print("Error_SensorA--------------====--");
  }
  
  delay(10);
  if(Time>=(loop_saved_time+10000))
  {                       
    displayControl();
    Get(Time);
    Get(readVoltage(EngineTemp_R,REF)/10-50,"TempR: ");
    Get(Engine_Temperature = readVoltage(EngineTemp_L,REF)/10-50,"TempL: ");
    Engine_Temperature+= readVoltage(EngineTemp_L,REF)/10-50;
    Get(Engine_Temperature/2,"Temperatura Silnika: ");
    Get(SensorA.cTemp,"TempA: ");
    Get(SensorA.Humidity,"Humidity: ");
    Debug::Print("*******");

    Circut_Voltage=(readVoltage (V12_VoltageRead_Pin,REF));   
    Battery_Voltage=(V120_VoltageRead_Pin,REF);
    
    loop_saved_time=Time;
  }
}

void lcdSetup (void)
{
  lcd.init();
  lcd.backlight();
  
  lcd.setCursor(0,0); 
  lcd.print("12V : ");
  lcd.setCursor(10,0); 
  lcd.print("120V: ");

  lcd.setCursor(0,1); 
  lcd.print("TemA: ");
  lcd.setCursor(10,1); 
  lcd.print("TemS:");

  lcd.setCursor(0,2); 
  lcd.print("TemL: ");
  lcd.setCursor(10,2); 
  lcd.print("TemR: ");
  
  lcd.setCursor(0,3); 
  lcd.print("HumA: ");
  lcd.setCursor(10,3); 
  lcd.print("Fan : ");
}

void displayerControl()
{ 
  lcd.setCursor(6,0); 
  lcd.print(Circut_Voltage);
  lcd.setCursor(16,0); 
  lcd.print(Battery_Voltage);

  lcd.setCursor(6,1); 
  lcd.print(SensorAm_Temperature);
  lcd.setCursor(16,1); 
  lcd.print(Engine_Temperature);

  lcd.setCursor(6,2); 
  lcd.print(SensorL_Temperature);
  lcd.setCursor(16,2); 
  lcd.print(SensorR_Temperature);
  
  lcd.setCursor(6,3); 
  lcd.print(SensorAm_Humidity);
  lcd.setCursor(16,3); 
  lcd.print(EngineFan_Mode);
}

void buttonControl(unsigned long &Time)
{
  switch(button.checkSignal(Time))
  {
    case 0:
      break;
    case 1:
      Debug::Get(EngineFan_Mode,"Engine Fan Mode: ");
      EngineFan_Mode++;
      if(EngineFan_Mode==custom)
        EngineFan_Mode=off;
      break;
    case 10:
      if(BackLight)
        digitalWrite(Light_Control_Pin,BackLight=0);
      else
        digitalWrite(Light_Control_Pin,BackLight=1);
      break;
  }
}

void fanControl (uint8_t mode)
{
  if(EngineFan_Mode==EngineFan_State)
    return;
  switch(mode)    
  {
    case off: 
      digitalWrite(Fan_Control_Pin1,HIGH);
      digitalWrite(Fan_Control_Pin2,HIGH);
      digitalWrite(Fan_Control_Pin3,HIGH);
      EngineFan_State=0;
      break;
    case minimum:
      digitalWrite(Fan_Control_Pin1,LOW);
      digitalWrite(Fan_Control_Pin2,HIGH);
      digitalWrite(Fan_Control_Pin3,HIGH);
      EngineFan_State=1;
      break;
    case medium:
      digitalWrite(Fan_Control_Pin1,HIGH);
      digitalWrite(Fan_Control_Pin2,LOW);
      digitalWrite(Fan_Control_Pin3,HIGH);
      EngineFan_State=2;
      break;      
    case maximum:
      digitalWrite(Fan_Control_Pin1,HIGH);
      digitalWrite(Fan_Control_Pin2,HIGH);
      digitalWrite(Fan_Control_Pin3,LOW);
      EngineFan_State=3;
      break;
    case winter:
      fanControl (automaticMode(60,75,90));
      break;
    case normal:
      fanControl (automaticMode(50,70,90));
      break;
    case summer:
      fanControl (automaticMode(40,60,80));
      break;
 //   case custom:
//      fanControl (automaticMode());
//      break;
    default:
      EngineFan_Mode = 0;
      break;
  }
}

uint8_t automaticMode( float i1, float i2, float i3)
{
  if(Engine_Temperature>i3)
    return 3;
  else if (Engine_Temperature>i2)
    return 2;
  else if (Engine_Temperature>i1)
    return 1;
  else 
    return 0; 
}
