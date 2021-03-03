#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ACS712.h>
#include <SPI.h>
#include <RtcDS3231.h>
#include <SD.h>
#include <Adafruit_BMP280.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
ACS712 acs712(ACS712_30A, PB1);

RtcDateTime dt;
RtcDS3231<TwoWire> Rtc(Wire);
char dateString[16];
char timeString[16];

uint32_t previousMillis1 = 0;
uint32_t previousMillis2 = 0;
uint8_t countLCD = 0;

uint32_t currentRawTotal = 0;
uint32_t count = 0;
float current = 0.0;
uint16_t currentRaw = 0;
Adafruit_BMP280 bmp;

float temperature = 0.0;
float pressure = 0.0;

File logFile;

void setup() {
  // put your setup code here, to run once:
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
  lcd.setCursor(4,0);
  lcd.print("YAKINJAYA");
  lcd.setCursor(2,1);
  lcd.print("Data Logger");

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

  if(!bmp.begin(0x76)){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("BMP280 gagal!");
    while(1);
  }

  acs712.setZeroPoint(acs712.calibrate());
  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(millis() - previousMillis1 >= 1000){
    previousMillis1 = millis();
    lcd.clear();

    sprintf(dateString, "%02u/%02u/%04u", dt.Day(), dt.Month(), dt.Year());
    lcd.setCursor(0, 0);
    lcd.print(dateString);

    sprintf(timeString, "%02u:%02u:%02u", dt.Hour(), dt.Minute(), dt.Second());
    lcd.setCursor(0, 1);
    lcd.print(timeString);

  }

  //currentRawTotal += analogRead(PB1);
  //count++;
  current = acs712.getCurrentDC();
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure() / 100;

  if(millis() - previousMillis2 >= 2000){
    previousMillis2 = millis();

    logFile = SD.open("log.txt", FILE_WRITE);

    if(logFile){
      logFile.println(dateString + String(", ") + timeString + String("\tCurrent: ") + String(current) + String(" A") + String("\tTemperature: ") + String(temperature) + String(" C") + String("\tPressure: ") + String(pressure) + String(" hPa"));
      logFile.close();

      lcd.clear();
      lcd.setCursor(0, 0);

      countLCD++;

      if(countLCD > 3) countLCD = 1;

      switch(countLCD){
        case 1:
        lcd.print(String("Current: ") + String(current));
        break;

        case 2:
        lcd.print(String("Temp: ") + String(temperature));
        break;

        case 3:
        lcd.print(String("Press: ") + String(pressure));
        break;

        default:
        countLCD = 1;
        break;
      }
      
      lcd.setCursor(0, 1);
      lcd.print(String("Berhasil simpan!"));

      //lcd.clear();
      //lcd.setCursor(0, 0);
      //lcd.print("Berhasil simpan!");
    }   
  }
  dt = Rtc.GetDateTime();
}