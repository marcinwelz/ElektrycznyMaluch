
//    Version 2.4

#include <AM2320.h>
#include <LiquidCrystal_I2C.h>

// constants    
//#define Debug                       //Comment this line in final version
#define V120_VoltageRead_Pin    A0    
#define V12_VoltageRead_Pin     A1    
#define Fan_Control_Pin1        9       //Minimum Power
#define Fan_Control_Pin2        12      //Medium  Power
#define Fan_Control_Pin3        10      //Maximum Power
#define Light_Control_Pin       8
#define Button_Pin              13      //D13
#define Reference_Value         5       //You can check it at pin "REF" 
#define Correction_Coefficient  1,01145 //corrects


//data
unsigned long button_click_time;
unsigned long loop_saved_time;
uint16_t  battery_voltage;     //*100
uint16_t  circuit_voltage;     //*100
//battery_current;  
//circuit_current;  
float  engine_temp;          
float  engine_humidity;      
float  driver_temp;          
uint8_t  engine_fan_mode;
uint8_t  engine_fan_current_level;
bool  brake;                    
bool  driver_fan;          
bool  engine;   
bool  back_light; 
struct customMode {uint8_t i1,i2,i3;} custom_fan_mode;


//List of Functions 
void lcdSetup (void);
void showDataOnLcd (void);
void meassureVoltage (void);
int Read_Pin_Temp(const int Pin);
void fanControl (uint8_t mode);
uint8_t automaticMode (float,float,float);
void buttonControl(void);
void printFanModeName (int8_t mode);



//Debug Functints
void DebugFunctionSet();

#ifdef Debug
void codeSpeed (const unsigned long A,const unsigned long B,unsigned long &Time);
extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;
extern "C" {
  int freeMemory()
  {
    int free_memory;
    if((int)__brkval == 0)
      free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
      free_memory = ((int)&free_memory) - ((int)__brkval);
    return free_memory;
  }
}
#endif

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


// setup
TwoWire twoWire;
AM2320 sensor(&twoWire);
LiquidCrystal_I2C lcd(0x27,20,4);


void setup()
{
  pinMode(Fan_Control_Pin1,OUTPUT);
  pinMode(Fan_Control_Pin2,OUTPUT);
  pinMode(Fan_Control_Pin3,OUTPUT);
  pinMode(Button_Pin,INPUT);
  pinMode(Light_Control_Pin,OUTPUT);
  digitalWrite(Light_Control_Pin,HIGH);
  
  back_light = 1;
  engine_fan_mode=normal;
  button_click_time=0;
  engine_fan_current_level=0;

  twoWire.begin();          //is it necessary?
  //Serial.begin(38400);
  lcdSetup();
  delay (1000);
}


void loop() 
{
  unsigned long Time=millis();
  sensor.Read();
  engine_temp=sensor.cTemp;
  meassureVoltage();
  buttonControl();
  fanControl(engine_fan_mode);
  
  if(Time>(loop_saved_time+2000))
  {  
    showDataOnLcd();
    loop_saved_time=Time;
  }
  
//  delay(RefreshTime);
}

void lcdSetup (void)
{
  lcd.init();
  lcd.backlight();
  
  lcd.setCursor(0,0); 
  lcd.print("CircuitV: ");
  lcd.setCursor(19,0);
  lcd.print("L");

  lcd.setCursor(0,1); 
  lcd.print("BatteryV: ");
  lcd.setCursor(19,1);
  lcd.print("P");

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

  lcd.setCursor(10,0);
  output=circuit_voltage;
  lcd.print(output/100,2);  

  lcd.setCursor(10,1);
  output=battery_voltage;
  lcd.print(output/10,1);  
  
  lcd.setCursor(14,2);
  lcd.print(engine_temp,1); 
  if(engine_temp<10000)
    lcd.print(" "); 

  lcd.setCursor(16,0);
  lcd.print(Read_Pin_Temp(A6));
  lcd.setCursor(16,1);
  lcd.print(Read_Pin_Temp(A7));
}

void  meassureVoltage (void)
{
  float p,P,R1,R2,X = 0;

          // Circut Voltage
  p =analogRead(V12_VoltageRead_Pin);     // read pin p=(0~~1023)
  P = (p*Reference_Value /1023);          // change value of p to voltage (0V - ~5V)
  R1 = 9850; 
  R2 = 4630; 
  X = (P* (R1+R2)/R2);                    // calculate X
  X*=100;
  circuit_voltage = X;                    // save value into circuit_voltage 
 
          // Battery Voltage
  p =analogRead(V120_VoltageRead_Pin);
  P = (p*Reference_Value /1023);
  R1 = 99570;
  R2 = 3258; 
  X = (P* (R1+R2)/R2);
  X*=10;
  X*=Correction_Coefficient;     
  battery_voltage = X;
}

int Read_Pin_Temp(const int Pin)
{
  // 1023p = ref mV
  // analogRead(A1)p = x mV
  // x = ref*analogRead(A1)/1023
  float V = (Reference_Value*analogRead(Pin))/1023;  // voltage in mV

  V *= 0.1;
  V -= 50;
  //500mV => 0*C
  //+10mV => +1*C 
  return (int)V-2;
  //100mV => -40*C //minimum
}

void fanControl (uint8_t mode)
{
  if(mode==engine_fan_current_level)
    return;
  switch(mode)
  {
    case off: 
      digitalWrite(Fan_Control_Pin1,HIGH);
      digitalWrite(Fan_Control_Pin2,HIGH);
      digitalWrite(Fan_Control_Pin3,HIGH);
      engine_fan_current_level=0;
      break;
    case minimum:
      digitalWrite(Fan_Control_Pin1,LOW);
      digitalWrite(Fan_Control_Pin2,HIGH);
      digitalWrite(Fan_Control_Pin3,HIGH);
      engine_fan_current_level=1;
      break;
    case medium:
      digitalWrite(Fan_Control_Pin1,HIGH);
      digitalWrite(Fan_Control_Pin2,LOW);
      digitalWrite(Fan_Control_Pin3,HIGH);
      engine_fan_current_level=2;
      break;      
    case maximum:
      digitalWrite(Fan_Control_Pin1,HIGH);
      digitalWrite(Fan_Control_Pin2,HIGH);
      digitalWrite(Fan_Control_Pin3,LOW);
      engine_fan_current_level=3;
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
    case custom:
      fanControl (automaticMode());
      break;
    default:
      engine_fan_mode = 0;
      break;
  }
}

uint8_t automaticMode(float i1, float i2, float i3)
{
  if(engine_temp>i3)
    return 3;
  else if (engine_temp>i2)
    return 2;
  else if (engine_temp>i1)
    return 1;
  else 
    return 0; 
}

uint8_t automaticMode()
{
  automaticMode (custom_fan_mode.i1, custom_fan_mode.i2, custom_fan_mode.i3);
}

void buttonControl()
{
  switch (digitalRead(Button_Pin))
  {
  case HIGH:                          // button down
    if(button_click_time)               // button is held                               
      return;                             // do nothing
    else                                // button has been clicked
      button_click_time=millis();         // save the time of click
    break;
  case LOW:                           // button up
    if (button_click_time==0)           //button is untouched
      return;                             // do nothing
    else                                //button is released
    {
      if((millis()-button_click_time)>1500)   // more than 1.5 sec
        {
          if(back_light)                        // if light is on
          {
            digitalWrite(Light_Control_Pin,LOW);  // turn light off
            back_light=false;
          }
          else                                  // if light is off
          {
            digitalWrite(Light_Control_Pin,HIGH); // turn light on
            back_light=true;
          }
        }
        else                                  // less than 1.5 sec
        {
          engine_fan_mode++;                      // chenge mode of engine fan           
          if(engine_fan_mode==7)                      // there is 8 modes (0-7)
            engine_fan_mode=0;                        // mode number 7 (custom) not yet available)
          printFanModeName(engine_fan_mode);          
        }
      button_click_time=0;                    // end button release section
      return;                        
    }                 
  }
}

void printFanModeName (int8_t mode)
{
  lcd.setCursor(8,3);
  switch(mode)
  {
    case off: 
      lcd.print(" OFF  ");
      break;
    case minimum:
      lcd.print(" 1    ");
      break;
    case medium:
      lcd.print(" 2    ");
      break;      
    case maximum:
      lcd.print(" 3    ");
      break;
    case winter:
      lcd.print("Winter");
      break;
    case normal:
      lcd.print("Normal");
      break;
    case summer:
      lcd.print("Summer");
      break;
    case custom:            // not yet in use
      lcd.print("Custom");
      break;
    default:
      lcd.print(" ERROR");
      break;
  }
}
