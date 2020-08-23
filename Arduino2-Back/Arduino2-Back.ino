//  Version 4.0  //


#include <KacperPinControl.h>
#include <AM2320.h>
#include <LiquidCrystal_I2C.h>

//pins
#define V120_VoltageRead_Pin    A0 
#define V12_VoltageRead_Pin     A1
#define SensorR_Pin             A6
#define SensorL_Pin             A7  
#define Fan_Control_Pin1        9       //Minimum Power
#define Fan_Control_Pin2        12      //Medium  Power
#define Fan_Control_Pin3        10      //Maximum Power
#define Light_Control_Pin       8
#define Button_Pin              13      //D13
#define HallSensor_Pin          2
#define REF                     4980    //mV  reference value, check at pin REF

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

//objects
AM2320 SensorA(&Wire);
Button button(Button_Pin);
LiquidCrystal_I2C lcd(0x27,20,4);
union converter
{
  float Float;
  uint8_t ByteArray[4];
};


//functionts
inline void lcd_Setup (void);
inline void fan_Control (uint8_t mode);     
uint8_t automaticMode (float,float,float);
inline void button_Control (unsigned long &Time);
inline void lcd_Control(void);
void hallSensor_ISR (void);
float valueInRange (float Value, const int Minimum, const int Maximum, const int Range=0, float OldValue=0);
void communicationWithA1();

//Global Variables (This Arduino)
unsigned long loop_saved_time =2000; 
float       SensorL_Temperature;    //MCP9700
float       SensorR_Temperature;    //MCP9700
float       SensorR_FilteredTemp;
float       SensorL_FilteredTemp;


//Global Variables (Multiple Arduinos)
bool        BackLight_Off;
uint8_t     EngineFan_Mode=normal;
uint8_t     EngineFan_State=4;
uint8_t     Engine_Speed;         // in Revolutionts per Second
float       Engine_Temperature;   // calculated from sensor r and l filtered temp.
float       Engine_AirTemperature;
float       Engine_AirHumidity;    //AM2320
float       Circut_Voltage;
float       Battery_Voltage;



void setup()
{
  Serial.begin(38400);
  Wire.begin();

  pinMode(Fan_Control_Pin1,OUTPUT);
  pinMode(Fan_Control_Pin2,OUTPUT);
  pinMode(Fan_Control_Pin3,OUTPUT);
  pinMode(Button_Pin,INPUT);
  pinMode(Light_Control_Pin,OUTPUT);
  digitalWrite(Light_Control_Pin,BackLight_Off = 1);
  pinMode(HallSensor_Pin,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HallSensor_Pin), hallSensor_ISR, RISING);

  lcdSetup();
}

const float R1 = 9850;
const float R2 = 4630;
const float R3 = 99570;
const float R4 = 3258;

void loop()
{ 
  unsigned long Time=millis();
  
  //LoopSpeed(5000,Time);     // naprawić
  buttonControl(Time);          // zrobić aby przy max jednej/dwóch zmiennych program aktywował funkcje w różnych odstępach czasu?    
  fanControl(EngineFan_Mode);   // replace mode witch EngineFan_Mode
  
  if(Time>=loop_saved_time)  //this kode runs once each second
  {  
    loop_saved_time+=1000;
    if(SensorA.Read())
      SensorA.cTemp=0;  //SensorA_Error

    SensorR_Temperature = (float)readVoltage(SensorR_Pin,REF)/10-50;
    SensorL_Temperature = (float)readVoltage(SensorL_Pin,REF)/10-50;
    
    const float x=10;
    SensorR_FilteredTemp=(SensorR_FilteredTemp*(x-1)+valueInRange(SensorR_Temperature,-40,120,40,SensorR_FilteredTemp))/x;
    SensorL_FilteredTemp=(SensorL_FilteredTemp*(x-1)+valueInRange(SensorL_Temperature,-40,120,40,SensorL_FilteredTemp))/x;
    Engine_Temperature = (SensorL_FilteredTemp+SensorR_FilteredTemp)/2;
    
    Circut_Voltage=readVoltage(V12_VoltageRead_Pin,REF) * (R1+R2)/(R2*1000);
    Battery_Voltage=readVoltage(V120_VoltageRead_Pin,REF) * (R3+R4)/(R4*1000);
    Engine_AirTemperature = SensorA.cTemp;
    Engine_AirHumidity = SensorA.Humidity;
    
    // Custom Filters
    // Battery_Voltage+=(Battery_Voltage/1000)*15;    // +1,5%
    
    lcdControl();
    communicationWithA1();
    Engine_Speed=0;
  }
}

void lcdSetup (void)
{
  lcd.init();
  lcd.backlight();
  
  lcd.setCursor(0,0); 
  lcd.print("12V :");
  lcd.setCursor(10,0); 
  lcd.print("120V:");

  lcd.setCursor(0,1); 
  lcd.print("TemS:");
  lcd.setCursor(10,1); 
  lcd.print("TemR:");

  lcd.setCursor(0,2); 
  lcd.print("TemA:");    //
  lcd.setCursor(10,2); 
  lcd.print("TemL:");
  
  lcd.setCursor(0,3); 
  lcd.print("RPM :");    //"HumA:"
  lcd.setCursor(10,3); 
  lcd.print("Fan :");
}

void lcdControl()
{ 
  lcd.setCursor(5,0); 
  lcd.print("    ");
  lcd.setCursor(15,0); 
  lcd.print("    ");
  lcd.setCursor(5,0); 
  lcd.print(Circut_Voltage,1);
  lcd.setCursor(15,0); 
  lcd.print(Battery_Voltage,1);
  
  lcd.setCursor(5,1); 
  lcd.print("    ");
  lcd.setCursor(15,1); 
  lcd.print("    ");
  lcd.setCursor(5,1); 
  lcd.print(Engine_Temperature,1);
  lcd.setCursor(15,1); 
  lcd.print(SensorR_Temperature,1);

  lcd.setCursor(5,2); 
  lcd.print("    ");
  lcd.setCursor(15,2); 
  lcd.print("    ");
  lcd.setCursor(5,2); 
  lcd.print(Engine_AirTemperature,1);  
  lcd.setCursor(15,2); 
  lcd.print(SensorL_Temperature,1);

  lcd.setCursor(6,3);   // 5 --> 6
  lcd.print("    ");
  lcd.setCursor(15,3); 
  lcd.print("    ");
  lcd.setCursor(6,3);   // 5 --> 6
  lcd.print(Engine_Speed*60);    //Engine_AirHumidity
  lcd.setCursor(15,3);  
  lcd.print(EngineFan_Mode);
}
  

void buttonControl(unsigned long &Time)
{
  switch(button.checkSignal(Time))
  {
    case 0:
      break;
    case 1:
      EngineFan_Mode++;
      if(EngineFan_Mode==custom)
        EngineFan_Mode=off;
      break;
    case 10:
      if(BackLight_Off)
        digitalWrite(Light_Control_Pin,BackLight_Off=0);
      else
        digitalWrite(Light_Control_Pin,BackLight_Off=1);
      break;
  }
}

void fanControl (uint8_t mode)
{
  if(EngineFan_Mode==EngineFan_State)
    return;
  switch(mode)    // replace mode witch EngineFan_Mode
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
//    case custom:
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

void hallSensor_ISR () //This function is called whenever a magnet/interrupt is detected by the arduino
{
 Engine_Speed++;
}

float valueInRange (float Value, const int Minimum, const int Maximum, const int Range=0, float OldValue=0)
{  
  if(Value>=Minimum&&Value<=Maximum)
    if(Range>0)
      if(Value>(OldValue-Range)&&Value<(OldValue+Range))
        return Value;
  return OldValue;
}

void communicationWithA1()
{
  converter Letter;
  //1
  Letter.Float=Battery_Voltage;
  Serial.write((byte)1);
  Serial.write(Letter.ByteArray,4);
  Serial.write((byte)10);
  //2
  Letter.Float=Circut_Voltage;
  Serial.write((byte)2);
  Serial.write(Letter.ByteArray,4);
  Serial.write((byte)10);
  //3
  Letter.Float=Engine_Temperature;
  Serial.write((byte)3);
  Serial.write(Letter.ByteArray,4);
  Serial.write((byte)10);
  //4
  Letter.Float=Engine_AirTemperature;
  Serial.write((byte)4);
  Serial.write(Letter.ByteArray,4);
  Serial.write((byte)10);
  //5
  Letter.Float=Engine_AirHumidity;
  Serial.write((byte)5);
  Serial.write(Letter.ByteArray,4);
  Serial.write((byte)10);
  //6
  Letter.Float=EngineFan_State;
  Serial.write((byte)6);
  Serial.write(Letter.ByteArray,4);
  Serial.write((byte)10);
  //7
  Letter.Float=EngineFan_Mode;
  Serial.write((byte)7);
  Serial.write(Letter.ByteArray,4);
  Serial.write((byte)10);
  //8
  Letter.Float=Engine_Speed;
  Serial.write((byte)8);
  Serial.write(Letter.ByteArray,4);
  Serial.write((byte)10);
}
