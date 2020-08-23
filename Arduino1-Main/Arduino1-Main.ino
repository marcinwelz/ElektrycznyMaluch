//  Version 4.0  //

#include <WString.h>
#include <OneWire.h>
#include <DS18B20.h>

#define EngineRadiator_SensorDS_Address 
#define EngineControler_SensorDS_Address 
 
//pins
#define Pin1    29
#define Pin2    28
#define Pin3    27
#define Pin4    26
#define Pin5    25
#define Pin6    24
#define Pin7    23  
#define Pin8    22  
#define EngineControler__TemperatureSensor_Pin 31
#define EengineRadiator_TemperatureSensor_Pin 30
#define ButtonPin 5
#define BackArduinoResetPin 4

//objects
class Sensor_DS18B20
{
  protected:
    const byte SensorDS_Address[8];
    DS18B20 sensorDS;
    OneWire oneWire;
  public:
    float Temperature;
    Sensor_DS18B20 (byte Onewire_Pin, const byte Address[8]) : oneWire(Onewire_Pin), SensorDS_Address({Address[0],Address[1],Address[2],Address[3],Address[4],Address[5],Address[6],Address[7]}), sensorDS(&oneWire) 
    {
      //Debug::Get(Onewire_Pin,"Sensor Instalation at Pin: ");
      sensorDS.begin();
    }

    void ReadTemperature()
    {
      if(sensorDS.available())
      {
        float temperature = sensorDS.readTemperature(SensorDS_Address);
        sensorDS.request(SensorDS_Address);
      }
    }
};

union converter
{
  float Float;
  uint8_t ByteArray[4];
};

//functionts
void recieveFromA2 ();
void overrideDataById (uint8_t Id, float Variable); 
inline void SendStringToRapsberry();


//Global Variables (This Arduino)
unsigned long loop_saved_time =2000;
unsigned long Time;

//Global Variables (Multiple Arduinos)
float       EngineRadiator_Temperature;
float       EngineController_Temperature;
float       Serial1_NumberOfComunicationErrors;

// Variables from A2
bool        BackLight_Off;
uint8_t     Engine_Speed;
uint8_t     EngineFan_Mode;
uint8_t     EngineFan_State;
float       Engine_AirTemperature;
float       Engine_AirHumidity;
float       Engine_Temperature;
float       Circut_Voltage;
float       Battery_Voltage;


void setup() 
{
  Serial.begin(38400);
  Serial1.begin(38400);
  pinMode(Pin1,OUTPUT);
  pinMode(Pin2,OUTPUT);
  pinMode(Pin3,OUTPUT);
  pinMode(Pin4,OUTPUT);
  pinMode(Pin5,OUTPUT);
  pinMode(Pin6,OUTPUT);
  pinMode(Pin7,OUTPUT);
  pinMode(Pin8,OUTPUT);
 
  digitalWrite(Pin1,HIGH);
  digitalWrite(Pin2,HIGH);
  digitalWrite(Pin3,HIGH);
  digitalWrite(Pin4,HIGH);
  digitalWrite(Pin5,HIGH);
  digitalWrite(Pin6,HIGH);
  digitalWrite(Pin7,HIGH);
  digitalWrite(Pin8,HIGH);
}

void loop()
{
  Time=millis();
  
  //EngineRadiator_Temperature;
  
  recieveFromA2();
  
  if(Time>=loop_saved_time+300)  //this kode runs once each second
  {  
    Serial.println();
    loop_saved_time=Time;
    SendStringToRapsberry();  //Serial is occupied    
  }
}

inline void SendStringToRapsberry()
{
  String text="";
  Serial.println((text="Battery_Voltage: "              + String((float)Battery_Voltage)));
  Serial.println((text="Circut_Voltage: "               + String((float)Circut_Voltage)));
  Serial.println((text="Engine_Temperature: "           + String((float)Engine_Temperature)));
  Serial.println((text="Engine_AirTemperature: "        + String((float)Engine_AirTemperature)));
  Serial.println((text="Engine_AirHumidity: "           + String((float)Engine_AirHumidity)));
  Serial.println((text="Engine_Speed: "                 + String((float)Engine_Speed)));
  Serial.println((text="EngineFan_Mode: "               + String((float)EngineFan_Mode)));
  Serial.println((text="EngineFan_State: "              + String((float)EngineFan_State)));
  Serial.println((text="EngineController_Temperature: " + String((float)EngineController_Temperature)));
  Serial.println((text="EngineRadiator_Temperature: "   + String((float)EngineRadiator_Temperature)));
  Serial.println((text="Serial1_NumberOfComunicationErrors: "    + String((float)Serial1_NumberOfComunicationErrors)));
  
}


// Old Version of comunication
/*String makeReadableString(String text, auto& Variable)
{
  String output="";
  output+= "voltage: ";                  output+= String((float)sendData[0]/10,1);
  output+= "\ncircuit_voltage: ";        output+= String((float)sendData[1]/10,1);
  output+= "\nbattery_consumption: ";    output+= String((float)sendData[2]/10,1);
  output+= "\npower_consumption: ";      output+= String((float)sendData[3]/10,1);
  output+= "\nengine_temp: ";            output+= String((float)sendData[4]/10,1);
  output+= "\nengine_humidity: ";        output+= String((float)sendData[5]/10,1);
  output+= "\nadditional_temp: ";        output+= String((float)sendData[6]/10,1);
  return output;
}*/


void recieveFromA2 ()
{
  if(Serial1.available()>=6)
  {
    converter Buffer;
    byte Id=0;
    Id = Serial1.read();
    Serial1.readBytes(Buffer.ByteArray,4);
   /* { Serial.print(Id); Serial.print("  ");
      Serial.print(Buffer.ByteArray[0]);  Serial.print("  ");
      Serial.print(Buffer.ByteArray[1]);  Serial.print("  ");
      Serial.print(Buffer.ByteArray[2]);  Serial.print("  ");
      Serial.print(Buffer.ByteArray[3]);  Serial.print("  ");
      Serial.println(Serial1.peek());
    } */
    overrideDataById(Id,Buffer.Float);
    if(Serial1.peek()==10) Serial1.read();
  }
}

void overrideDataById (uint8_t Id, float Variable)
{
  switch (Id)
  {
    case 0 :
      return;
    case 1 :
      Battery_Voltage = Variable;
      break;
    case 2 :
      Circut_Voltage = Variable;
      break;
    case 3 :
      Engine_Temperature = Variable;
      break;
    case 4 :
      Engine_AirTemperature = Variable;
      break;
    case 5 :
      Engine_AirHumidity = Variable;
      break;
    case 6 :
      EngineFan_State = Variable;
      break;
    case 7 :
      EngineFan_Mode = Variable;
      break;
    case 8 :
      Engine_Speed = Variable;
      break;
    default :
    //break;  // Debug !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      char SteamFixer;
      Serial1_NumberOfComunicationErrors++;
      while(Serial1.available())
      {
        SteamFixer = Serial1.read();
        if(SteamFixer==10) return;
      }
  }
}
