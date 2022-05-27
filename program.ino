//--inicializace pro komunikování se serverem
#include <ThingerESP8266.h>
#include <ESP8266WiFi.h>
#define USERNAME "testec"
#define DEVICE_ID "1"
#define DEVICE_CREDENTIAL "6mmcN8!?zA?MDFd#"
#define SSID "Internet_14A1"
#define SSID_PASSWORD "*******"
#define SENSOR  0
//---
//--- inicializace pinu pro krokovej motor
#include <AccelStepper.h>
#define IN1 5
#define IN2 4
#define IN3 14
#define IN4 12
const int stepsPerRevolution = 1024;//fo půlky otočení

AccelStepper stepper(AccelStepper::HALF4WIRE, IN1, IN3, IN2, IN4);
//--

unsigned long int avgValue;  //Store the average value of the sensor feedback
float b;
int buf[10],temp;
ThingerESP8266 thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);
long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;

//--inicializace kalibračních věcí,pulzu a milisekund kvuli zjišťování zpoždění
float calibrationFactor = 4.5;
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
unsigned int phValue =0;
//--
void IRAM_ATTR pulseCounter()//inkrementace počítače pulzu
{
  pulseCount++;
}
void setup()
{//---nastavení wifiny, pinu na senzory a vynulování proměnných
  Serial.begin(115200);
  thing.add_wifi(SSID, SSID_PASSWORD);
  pinMode(SENSOR, INPUT_PULLUP);
  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  previousMillis = 0;
  attachInterrupt(digitalPinToInterrupt(SENSOR), pulseCounter, FALLING);

//inicializace atributu pro fungování motoru
  stepper.setMaxSpeed(500);
  stepper.setAcceleration(100);
  // nastavení pozice
  stepper.moveTo(stepsPerRevolution);
}
void loop()
{
 for(int i=0;i<10;i++)       //nasbírání 10 bzorku do pole
  { 
    buf[i]=analogRead(A0);
    delay(30);
  }
  for(int i=0;i<9;i++)        //klasik bublesort na seřazení hodnot
  {
    for(int j=i+1;j<10;j++)
    {
      if(buf[i]>buf[j])
      {
        temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }
    }
  }
  


  
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {//sledování času a podle podmínky vykonání nebo nevykonání fukcí
    avgValue=0;
    for(int i=2;i<8;i++)                      //průměruju hodnoty, osekám to o krajní potencionální špatný hodnoty
    avgValue+=buf[i];
    float volt=(float)avgValue*5.0/1024/6;//vzoreček
    phValue=-5.70 * volt + 31.8;//ph hodnota vzoreček

    
    pulse1Sec = pulseCount;//tento blok je velký výpočet na protečený mililitry, jak celkový tak aktuální
    pulseCount = 0;
    flowRate = ((1000.0 / (millis() - previousMillis)) * pulse1Sec) / calibrationFactor;
    previousMillis = millis();
    flowMilliLitres = (flowRate / 60) * 1000;
    totalMilliLitres += flowMilliLitres;
    
    // kontrolní výpis do konzole
    Serial.print("Hodnota Ph: ");
    Serial.print(phValue);
    Serial.print("      ");
    Serial.print("Průtok: ");
    Serial.print(int(flowRate));  
    Serial.print("L/min");
    Serial.print("\t");      
    Serial.print("Celkový průtok od spuštění: ");
    Serial.print(totalMilliLitres);
    Serial.print("mL / ");
    Serial.print(totalMilliLitres / 1000);
    Serial.println("L");
    
    //odeslání na server
    thing["data"] >> [](pson& out){
    out["Flow Rate"] = flowRate;
    out["Total"]= totalMilliLitres;
    out["PH"]= phValue;
     };
    thing.handle();
    thing.stream(thing["data"]);
  }

  if(phValue<6.5){//pokud ph klesne pod 7 (optimální je 7,4) tak se otočí servo, který nechá vypadnout chlorovou tabletku
    stepper.moveTo(stepsPerRevolution);//otočí se o 180 stupnu s tabletkou
    if (stepper.distanceToGo() == 0){//dojde do konce
    stepper.moveTo(-stepper.currentPosition());//vrátí se zpátky do podávací polohy
    stepper.run();
  }
  }
  }
}
