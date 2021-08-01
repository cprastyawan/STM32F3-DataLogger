#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <RtcDS3231.h>
#include <SD.h>
#include <Adafruit_BMP280.h>
#include <ADS1X15.h>

#define ACS712_30 0.065
#define BUZZERPIN PA5
#define COMMA String(",")
#define DIRECTION_DIVIDER 45
#define MAP(x, in_min, in_max, out_max, out_min) (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

typedef union{
  float f32;
  uint8_t c8[4];
} Floating;

uint8_t send[8];

Floating T_send, P_send, I_send, V_send, W_send;

HardwareSerial Serial2(PA3, PA2);
LiquidCrystal_I2C lcd(0x27, 20, 4);
//SPIClass SPIESP32(PB15, PB14, PB13, PB12);

RtcDateTime dt;
RtcDS3231<TwoWire> Rtc(Wire);
char dateTimeString[32];
char dateString[16];
char timeString[16];

char rxChar;
String rxString;
bool captureData = false;
bool dc = false;                                                                                                                                                                                                                                             

uint32_t previousMillis = 0;
uint32_t sdCardPrevMillis = 0;

Adafruit_BMP280 bmp;

float temperature;
float temperatureTotal = 0.0;
float pressure;
float pressureTotal = 0.0;

float voltage;
float voltRawTotal = 0.0;
float voltTotal = 0.0;
float voltZeroPoint;

File logFile;

ADS1115 ads(0x48);
const float ADSMultiplier = 0.1875F;
uint32_t currentRawTotal = 0;
float currentZeroPoint = 0;
float current;
uint16_t currentRaw = 0;
float currentTotal;

uint32_t count = 0;
uint8_t dataCount = 0;

uint32_t inputRising = 0;
float anemoFrequency = 0.0;

volatile unsigned long sTime = 0;
unsigned long dataTimer = 0;
volatile float cPulseTime = 0, pulseTime = 0;
volatile unsigned int avgWindCount = 0;
volatile bool start = true;
float maxWindSpeed = 60.0;

float wSpeed;
float wDir;
float wDirTotal;
float wDirMax = 0.0;
float wDirMin = 360.0;

String ToSDCard;

String Wind_Direction;
String Last_Wind_Direction;
String windSend;
float Degree;
float Arah;
char *Hasil_Pembacaan;
int Threshold_Direction;
int Temp[20],counter;
float LastDegree;

void sendToESP();
float getAnemoFreq(float pulseTime);
float getWindMPH(float freq);
float getWindKPH(float MPHspeed);
float getWindMs(float cMPHspeed);
float getAvgWindSpeed(float pulse, int periode);
void windVelocity();
void anemoISR();
void addToString(String *strDest, String str);
void sendToSDCard();
void windDirection();

void setup() {
  // put your setup code here, to run once:
  Serial2.begin(115200);
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);

  SPI.setMISO(PB4);
  SPI.setMOSI(PB5);
  SPI.setSCLK(PB3);
  SPI.setSSEL(PA15);

  analogReadResolution(12);

  lcd.init();
  lcd.clear();
  lcd.backlight();

  lcd.setCursor(5,0);
  lcd.print("YAKINJAYA");
  lcd.setCursor(4,1);
  lcd.print("Data Logger");
  delay(2000);

  Rtc.Begin();
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);

  if(!Rtc.GetIsRunning()) Rtc.SetIsRunning(true);

  RtcDateTime now = Rtc.GetDateTime();
  if(now < compiled) Rtc.SetDateTime(compiled);

  if(!SD.begin(PA15)){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SDCard gagal!");
    while(1);
  }

  logFile = SD.open("log.csv", FILE_WRITE);
  if(logFile){
    logFile.println();
    logFile.close();
  }

  if(!bmp.begin(0x76)){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("BMP280 gagal!");
    while(1);
  }

  if(!ads.begin()){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ADS1115 gagal!");
    while(1);
  }

  ads.setGain(0);
  ads.setMode(0);

  for(int i = 0; i < 20; i++){
    currentRawTotal += ads.toVoltage(ads.readADC(0));
  }

  currentZeroPoint = currentRawTotal / 20.0;

  for(int i = 0; i < 50; i++){
    voltTotal += ads.toVoltage(ads.readADC(1));
  }

  voltZeroPoint = voltTotal / 50.0;
  voltTotal = 0;
  
  pinMode(PA5, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PA5), anemoISR, RISING);
  inputRising = millis();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Kalibrasi WD");
  lcd.setCursor(0, 1);
  lcd.print("Putar WD 360dg");


  for(int i = 0; i < 500; i++){
    wDir = ads.toVoltage(ads.readADC(2)) * (360.0 / 4.34);
    if(wDir > wDirMax) wDirMax = wDir;
    if(wDir < wDirMin) wDirMin = wDir;
    delay(10);
  }

  lcd.setCursor(0, 2);
  lcd.print("Selesai!");
  delay(2000);
  //lcd.clear();

  previousMillis = millis();
  sdCardPrevMillis = previousMillis;
}

void loop() {
  // put your main code here, to run repeatedly:
  dt = Rtc.GetDateTime();
  temperatureTotal += bmp.readTemperature();
  pressureTotal += (bmp.readPressure());
  currentTotal += ads.toVoltage(ads.readADC(0));
  voltTotal += (ads.toVoltage(ads.readADC(1)) - voltZeroPoint);
  wDirTotal = wDirTotal + (ads.toVoltage(ads.readADC(2)));
  count += 1;

  //voltReal = voltRawTotal / count;
  //voltRealTotal += 3.30 * ((float)voltRaw / 4096.0) * ((560.0 + 6900.0) / 560.0);

  windVelocity();
  
  if(dataCount >= 5) {
    dataCount = 0;
  }

  if(millis() - previousMillis >= 2000){
    previousMillis = millis();

    lcd.clear();

    current = ((currentTotal / count) - currentZeroPoint) * ACS712_30;
    currentTotal = 0;

    temperature = temperatureTotal / count;
    temperatureTotal = 0;

    pressure = (pressureTotal / count) / 100.0;
    pressureTotal = 0;

    voltage = (voltTotal / count) * (500.0 / 5.0);
    voltTotal = 0;

    //wDir = (wDirTotal / count) * (360.0 / 4.34);
    wDir = ads.toVoltage(ads.readADC(2)) * (360.0 / 4.34);
    wDir = MAP(wDir, wDirMin, wDirMax, 0.0, 359.99);
    wDirTotal = 0;

    windDirection();

    sprintf(dateString, "%04u-%02u-%02u", dt.Year(), dt.Month(), dt.Day());
    sprintf(timeString, "%02u:%02u:%02u", dt.Hour(), dt.Minute(), dt.Second());
    sprintf(dateTimeString, "%s %s", dateString, timeString);
    lcd.setCursor(0, 0);
    lcd.print(String(dateString) + " " + String(timeString));

    lcd.setCursor(0, 1);
    lcd.print(String("I: ") + String(current) + String(" V: ") + String(voltage));

    lcd.setCursor(0, 2);
    lcd.print(String("P: ") + String(pressure) + String(" T: ") + String(temperature));

    lcd.setCursor(0, 3);
    lcd.print(String("WS: ") + String(wSpeed) + String(" ") + String(Wind_Direction));

    sendToESP();
    ToSDCard += String(dateString + COMMA + timeString + COMMA + String(voltage) + COMMA + String(current) + COMMA + String(temperature) + COMMA + String(pressure) + COMMA + String(wSpeed) + COMMA + String(wDir) + String(dc ? "Disconnect" : "Connect") + String("\n"));

    count = 0;
    dataCount++;
  }


  //currentRawTotal += analogRead(PB1);
  //count++;

  if(millis() - sdCardPrevMillis >= 10000){
    sdCardPrevMillis = millis();

    //sendToESP();

    if(dc){
      logFile = SD.open("dc.csv", FILE_WRITE);  

      if(logFile){
        logFile.print(ToSDCard);
        logFile.close();
      }
    }

    logFile = SD.open("log.csv", FILE_WRITE);
    if(logFile){
      //Format data
      //logFile.println(dateString + COMMA + timeString + COMMA + String(voltage) + COMMA + String(temperature) + COMMA + String(pressure) + COMMA + String(wSpeed) + COMMA + String(wDir));
      logFile.print(ToSDCard);
      ToSDCard = String("");
      //logFile.println(dateTimeString + String("\tCurrent: ") + String(current) + String(" A") + String("\tTemperature: ") + String(temperature) + String(" C") + String("\tPressure: ") + String(pressure) + String(" hPa"));
      logFile.close();
    }
  }

  if(Serial2.available() > 0){
    if(rxChar == '#' && !captureData){
      captureData = true;
    } else if(rxChar == 'S' && captureData){
      captureData = false;
      if(rxString.compareTo("DC")) {
        dc = true;
      } else if(rxString.compareTo("CO")) dc = false;
      rxString = "";
    } else if(captureData){
      rxString += rxChar;
    }
  }
}

void sendToESP(){
  Floating temp;
  /*Serial2.write("#D");
  Serial2.write(dateString);
  Serial2.write('S');

  Serial2.write("#Y");
  Serial2.write(timeString);
  Serial2.write('S');*/

  Serial2.write("#$");
  Serial2.write(dateTimeString);
  Serial2.write('S');

  temp.f32 = temperature;
  Serial2.write("#T");
  Serial2.write(temp.c8, 4);
  Serial2.write('S');

  temp.f32 = pressure;
  Serial2.write("#P");
  Serial2.write(temp.c8, 4);
  Serial2.write('S');

  temp.f32 = current;
  Serial2.write("#I");
  Serial2.write(temp.c8, 4);
  Serial2.write('S');

  temp.f32 = voltage;
  Serial2.write("#V");
  Serial2.write(temp.c8, 4);
  Serial2.write('S');


  temp.f32 = wSpeed;
  Serial2.write("#E");
  Serial2.write(temp.c8, 4);
  Serial2.write('S');


  temp.f32 = wDir;
  Serial2.write("#W");
  //Serial2.write(temp.c8, 4);
  Serial2.print(windSend);
  Serial2.write('S');

  Serial2.write("#COMPLS");
}

void windVelocity(){
  unsigned long rTime = millis();

  if((rTime - sTime) > 2500){
    pulseTime = 0;
  }

  if((rTime - dataTimer) > 1000){
    float avgWindSpeed = getAvgWindSpeed(cPulseTime, avgWindCount);

    //if(avgWindSpeed >= maxWindSpeed) //digitalWrite(BUZZERPIN, HIGH);
    //else //digitalWrite(BUZZERPIN, LOW);
  }

  cPulseTime = 0;
  avgWindCount = 0;

  float anemoFreq = 0;

  if(pulseTime > 0.0) anemoFreq = getAnemoFreq(pulseTime);

  float windSpeedMPH = getWindMPH(anemoFreq);
  float windSpeedMS = getWindMs(windSpeedMPH);
  wSpeed = windSpeedMS;

  dataTimer = millis();

}

float getAnemoFreq(float pulseTime) {
  return (1 / pulseTime);
}

float getWindMPH(float freq) {
  return (freq * 2.5);
}

float getWindKPH(float MPHspeed) {
  return (MPHspeed * 1.61);
}

float getWindMs(float cMPHspeed) {
  return (cMPHspeed * 0.44704);
}
float getAvgWindSpeed(float pulse, int periode) {
  if (periode) {
    return getAnemoFreq((float)(pulse / periode));
  }
  else {
    return 0;
  }
}

void anemoISR(){
  unsigned long cTime = millis();
  pulseTime = (float)(cTime - sTime) / 1000.0;
  cPulseTime += pulseTime;
  avgWindCount++;
  sTime = cTime;
}

void sendToSDCard(){

}

void windDirection(){
  Degree = wDir;
  //for(int i=0; i<8; i++){
    Arah = (int)Degree % 45;
    Threshold_Direction = Degree/45;
    if(Threshold_Direction==0 && Arah<22.5 || Threshold_Direction==7 && Arah>22.5){
      counter++;
      if(counter >= 1){ 
        Wind_Direction = "Utara";
        windSend = "UUUU";
        counter=0;
        Last_Wind_Direction = Wind_Direction;
      }
      else
        Wind_Direction = Last_Wind_Direction;
    }
    else if(Threshold_Direction==1 && Arah<22.5 || Threshold_Direction==0 && Arah>22.5){
      counter++;
      if(counter >= 1){ 
        Wind_Direction = "Timur Laut";
        windSend = "cccc";
        counter=0;
        Last_Wind_Direction = Wind_Direction;
      }
      else
        Wind_Direction = Last_Wind_Direction;
    }
    else if(Threshold_Direction==2 && Arah<22.5 || Threshold_Direction==1 && Arah>22.5){
      counter++;
      if(counter >= 1){ 
        Wind_Direction = "Timur";
        windSend = "zzzz";
        counter=0;
        Last_Wind_Direction = Wind_Direction;
      }
      else
        Wind_Direction = Last_Wind_Direction;
    }
    else if(Threshold_Direction==3 && Arah<22.5 || Threshold_Direction==2 && Arah>22.5){
      counter++;
      if(counter >= 1){ 
        Wind_Direction = "Tenggara";
        windSend = "iiii";
        counter=0;
        Last_Wind_Direction = Wind_Direction;
      }
      else
        Wind_Direction = Last_Wind_Direction;
    }
    else if(Threshold_Direction==4 && Arah<22.5 || Threshold_Direction==3 && Arah>22.5){
      counter++;
      if(counter >= 1){ 
        Wind_Direction = "Selatan";
        windSend = "5555";
        counter=0;
        Last_Wind_Direction = Wind_Direction;
      }
      else
        Wind_Direction = Last_Wind_Direction;
    }
    else if(Threshold_Direction==5 && Arah<22.5 || Threshold_Direction==4 && Arah>22.5){
      counter++;
      if(counter >= 1){ 
        Wind_Direction = "Barat Laut";
        windSend = "LLLL";
        counter=0;
        Last_Wind_Direction = Wind_Direction;
      }
      else
        Wind_Direction = Last_Wind_Direction;
    }
    else if(Threshold_Direction==6 && Arah<22.5 || Threshold_Direction==5 && Arah>22.5){
      counter++;
      if(counter >= 1){ 
        Wind_Direction = "Barat";
        windSend = "****";
        counter=0;
        Last_Wind_Direction = Wind_Direction;
      }
      else
        Wind_Direction = Last_Wind_Direction;
    }
    else if(Threshold_Direction==7 && Arah<22.5){
      counter++;
      if(counter >= 1){ 
        Wind_Direction = "Barat Daya";
        windSend = "BBBB";
        counter=0;
        Last_Wind_Direction = Wind_Direction;
      }
      else
        Wind_Direction = Last_Wind_Direction;
    }
    else
      Wind_Direction = Last_Wind_Direction;
  //}
  //Serial.println(Wind_Direction);
  LastDegree = Degree;
}