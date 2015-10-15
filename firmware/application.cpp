//#pragma SPARK_NO_PREPROCESSOR
#include "application.h"

// This #include statement was automatically added by the Particle IDE.
#include "adafruit-led-backpack/adafruit-led-backpack.h"
#include "SparkIntervalTimer/SparkIntervalTimer.h"


#include <math.h>

#define TEMP_SENSOR 0x27
#define HEAT_PIN    A1
#define POT_PIN     A2
#define DESIRED_TEMP_FLASH_ADDRESS 0x80000
#define publish_delay 10000
#define TEMPCAL -4
unsigned int lastPublish = 0;
int EEPROM_ADDR = 10;
int LED = D7;                         // LED is connected to D7

Adafruit_8x8matrix matrix1;
Adafruit_8x8matrix matrix2;

static const uint8_t smile[] = {
  0b00111100,
  0b01000010,
  0b10100101,
  0b10000001,
  0b10100101,
  0b10011001,
  0b01000010,
  0b00111100
};

int I2C_Status = 0;
int potValue = 0;
int currentTemperature = 0;
int currentHumidity = 0;
int desiredTemperature = 0;
bool isHeatOn = false;
int displayTime = 10000;
int lastChangedPot = -80;
double fTemp = 0.0;

void OnTimer(void) {  //Handler for the timer, will be called automatically
  matrix1.clear();
  matrix2.clear();
  if (!isHeatOn){
    matrix1.setCursor(1, 0);
    matrix1.write('.');
  }
  else {
    matrix2.setCursor(1, 0);
    matrix2.write('+');
  }
  matrix1.setBrightness(1);  // 1-15
  matrix2.setBrightness(1);  // 1-15
  matrix1.writeDisplay();
  matrix2.writeDisplay();
}

IntervalTimer myTimer;  // To dim display
IntervalTimer myTimer7;  // To blink LeD


void displayTemperature(void)
{
  char ones = abs(desiredTemperature) % 10;
  char tens = (abs(desiredTemperature) / 10) % 10;

  matrix1.clear();
  matrix1.setCursor(0, 0);
  matrix1.write(tens + '0');
  //matrix1.drawBitmap(0, 0, smile, 8, 8, LED_ON);
  matrix1.setBrightness(1);  // 1-15
  matrix1.blinkRate(0);      // 0-3
  matrix1.writeDisplay();


  matrix2.clear();
  matrix2.setCursor(0, 0);
  matrix2.write(ones + '0');
  matrix2.setBrightness(1);  // 1-15
  matrix2.blinkRate(0);      // 0-3
  matrix2.writeDisplay();
  
  // Reset clock
  myTimer.resetPeriod_SIT(displayTime, hmSec);
}

void saveTemperature()
{
  Serial.println("Saving temperature to flash");
  uint8_t values[2] = { (uint8_t)desiredTemperature, 0 };
  EEPROM.put(EEPROM_ADDR, values);
}

void loadTemperature()
{
  Serial.println("Loading and displaying temperature from flash");
  uint8_t values[2];
  EEPROM.get(EEPROM_ADDR, values);
  desiredTemperature = values[0];
  displayTemperature();
}

int setTemperature(int t)
{
  desiredTemperature = t;
  displayTemperature();
  saveTemperature();
  return desiredTemperature;
}

int setTemperatureFromString(String t)
{
  // TODO more robust error handling
  //      what if t is not a number
  //      what if t is outside 50-90 range

  Serial.print("Setting desired temp from web to ");
  Serial.println(t);

  return setTemperature(t.toInt());
}

void setupMatrix(Adafruit_8x8matrix m)
{
  m.clear();
  m.writeDisplay();
  m.setTextSize(1);
  m.setTextWrap(false);
  m.setTextColor(LED_ON);
  m.setRotation(0);
  m.setCursor(0, 0);
}

void setup()
{
  Wire.setSpeed(CLOCK_SPEED_100KHZ);
  Wire.begin();

  pinMode(LED, OUTPUT);               // sets pin as output

  matrix1.begin(0x70);
  matrix2.begin(0x71);
  setupMatrix(matrix1);
  setupMatrix(matrix2);


  Particle.variable("stat", &I2C_Status, INT);
  Particle.variable("pot", &potValue, INT);
  Particle.variable("temp", &currentTemperature, INT);
  Particle.variable("hum", &currentHumidity, INT);
  Particle.variable("set", &desiredTemperature, INT);
  Particle.variable("call", &isHeatOn, BOOLEAN);
  Particle.function("set_temp", setTemperatureFromString);
  Particle.variable("fTemp", &fTemp, DOUBLE);

  Serial.begin(9600);

  loadTemperature();

  myTimer.begin(OnTimer, displayTime, hmSec);
  pinMode(HEAT_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);
}

void loop()
{
  unsigned long now = millis();
  static int wait = 0;
  if (!wait)
  {
    wait = 1000;

    //Wire.beginTransmission(TEMP_SENSOR);
    //Wire.endTransmission();
    delay(40);
    Wire.requestFrom(TEMP_SENSOR, 4); // request 4 bytes information
    uint8_t b = Wire.read();
    uint8_t bend = Wire.endTransmission();
    Serial.print("I2C Status bits are ");
    I2C_Status = bend;
    //I2C_Status = b >> 6;
    Serial.println(I2C_Status);

    int humidity = (b << 8) & 0x3f00;
    humidity |= Wire.read();
    float percentHumidity = humidity / 163.83;
    currentHumidity = roundf(percentHumidity);
    Serial.print("Relative humidity is ");
    Serial.println(percentHumidity);

    int temp = (Wire.read() << 6) & 0x3fc0;
    temp |= Wire.read() >> 2;
    temp *= 165;
    fTemp = (temp/16383.0 - 40.0)*1.8 + 32.0 + TEMPCAL; // convert to fahrenheit
    currentTemperature = roundf(fTemp);
    Serial.print("Temperature is ");
    Serial.println(fTemp);
  }

  //int pot;
  potValue = 4095 - analogRead(POT_PIN);
  if (1000 == wait)
  {
    Serial.print("Potentiometer reading: ");
    Serial.println(potValue);
  }


  // If user has adjusted the potentiometer
  if (fabsf(potValue - lastChangedPot) > 16)  // adjust from 64 because my range is 1214 not 4095
  {
    // Don't set temp on boot
    if (lastChangedPot >= 0)
    {
	  // my pot puts out 2872 - 4088 observed using following
      int t = roundf((float(potValue)-2872)/(4088-2872)*25+50);
      setTemperature(t);
      Serial.print("Setting desired temp based on potentiometer to ");
      Serial.println(t);
    }
    lastChangedPot = potValue;
  }

  isHeatOn = desiredTemperature > currentTemperature;
  digitalWrite(HEAT_PIN, isHeatOn);
  digitalWrite(LED, isHeatOn);   
  --wait;
  
  // Publish 
  if ((now - lastPublish) < publish_delay) {
        // it hasn't been 10 seconds yet...
        return;
    }
    //Particle.publish("librato_ThermoSet", String(desiredTemperature), 60, PRIVATE);
    Particle.publish("librato_ThermoTemp", String(fTemp), 60, PRIVATE);
    //Particle.publish("librato_ThermoHeatOn", String(isHeatOn), 60, PRIVATE);
    lastPublish = now;
}
