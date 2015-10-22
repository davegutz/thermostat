//#pragma SPARK_NO_PREPROCESSOR
#include "application.h"
#include "adafruit-led-backpack/adafruit-led-backpack.h"
#include "SparkIntervalTimer/SparkIntervalTimer.h"
#include "SparkTime/SparkTime.h"
#include "blynk/blynk.h"
#include "blynk/BlynkHandlers.h"
#include <math.h>
//#define BLYNK_DEBUG // Optional, this enables lots of prints
//#define BLYNK_PRINT Serial
#define DESIRED_TEMP_FLASH_ADDRESS 0x80000  // Flash temp address
#undef  FAKETIME                            // For simulating rapid time passing
#define HEAT_PIN    A1                      // Heat relay output pin on Photon
#define POT_PIN     A2                      // Potentiometer input pin on Photon
#define publish_delay 20000                 // Time between cloud updates
#define TEMP_SENSOR 0x27                    // Temp sensor bus address
#define TEMPCAL -8                          // Calibrate temp sense, F
#define MINSET  50                          // Minimum setpoint allowed, F
#define MAXSET  72                          // Maximum setpoint allowed, F
#define WAIT 5000                           // Event handling time
char                blynkAuth[]     = "4f1de4949e4c4020882efd3e61bdd6cd";
int                 blynkPub        = 0;    // blynk logging index
int                 currentHumidity = 0;    // Relative humidity, %
int                 currentTemperature = 0; // Temperature, F
unsigned long       currentTime;            // Time result
int                 desiredTemperature = 62;// Selected sched, F
int                 displayTime     = 10000;// Elapsed time display LED
int                 EEPROM_ADDR     = 10;   // Flash address
int                 EEPROM_ADDR2    = 20;   // Flash address
double              fTemp           = 0.0;  // Sensed temp, F
int                 I2C_Status      = 0;    // Bus status
bool                isHeatOn        = false;// Heat demand to relay control
bool                isWebPerm       = false;// Web toggled permanent and acknowledged
int                 lastChangedPot  = -80;  // To check for activity
int                 lastChangedSched= -80;  // To check for activity
int                 lastChangedWebDesired=-80; // To check for activity
unsigned int        lastPublish     = 0;    // Last publish time
bool                lastWebPerm     = false;// Web toggled permanent and acknowledged
int                 LED             = D7;   // Status LED
Adafruit_8x8matrix  matrix1;                // Tens LED matrix
Adafruit_8x8matrix  matrix2;                // Ones LED matrix
IntervalTimer       myTimer;                // To dim display
IntervalTimer       myTimer7;               // To blink LeD
int                 potValue        = 0;    // Dial raw value, F
SparkTime           rtc;                    // Time value
int                 schedValue      = 0;    // Sched raw value, F
unsigned long       lastTime        = 0UL;  // To check for activity
UDP                 UDPClient;              // Time structure
static const int    verbose         = 2;    // Debug, as much as you can tolerate
int                 webDesired      = 62;   // Web sched, F
bool                webPerm         = false;// Web permanence request

// Schedules
bool fHourChErr = false;
static float fHourCh[7][4] = {  // day of week 0=Sun x Event in day, hour of day
    6, 8, 18, 22,
    4, 7, 18, 22,
    4, 7, 18, 22,
    4, 7, 18, 22,
    4, 7, 18, 22,
    4, 7, 18, 22,
    6, 8, 18, 22
};
static const float fTempCh[7][4] = {  // day of week 0=Sun x Event in day, temp to set
    68, 62, 68, 62,
    68, 62, 68, 62,
    68, 62, 68, 62,
    68, 62, 68, 62,
    68, 62, 68, 62,
    68, 62, 68, 62,
    68, 62, 68, 62
};

// Handler for the display dimmer timer, called automatically
void OnTimerDim(void)
{
    matrix1.clear();
    matrix2.clear();
    if (!isHeatOn)
    {
        matrix1.setCursor(1, 0);
        matrix1.write('.');
    }
    else
    {
        matrix2.setCursor(1, 0);
        matrix2.write('+');
    }
    matrix1.setBrightness(1);  // 1-15
    matrix2.setBrightness(1);  // 1-15
    matrix1.writeDisplay();
    matrix2.writeDisplay();
}

// Display temperature setpoint on LEDs
void displayTemperature(void)
{
    char ones = abs(desiredTemperature) % 10;
    char tens = (abs(desiredTemperature) / 10) % 10;
    uint8_t smile[] = {
        0b00111100,
        0b01000010,
        0b10100101,
        0b10000001,
        0b10100101,
        0b10011001,
        0b01000010,
        0b00111100
    };
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

// Save temperature setpoint to flash for next startup
void saveTemperature()
{
    uint8_t values[2] = { (uint8_t)desiredTemperature, 0 };
    EEPROM.put(EEPROM_ADDR, values);
    uint8_t values2[2] = { (uint8_t)isWebPerm, 0 };
    EEPROM.put(EEPROM_ADDR2, values2);
}

// Load saved temperature setting so pickup where left off in case of power failure
void loadTemperature()
{
    Serial.println("Loading and displaying temperature from flash");
    uint8_t values[2];
    EEPROM.get(EEPROM_ADDR, values);
    desiredTemperature = values[0];
    displayTemperature();
    EEPROM.get(EEPROM_ADDR2, values);
    isWebPerm = values[0];
}

// Process a new temperature setting
int setTemperature(int t)
{
    desiredTemperature = t;
    displayTemperature();
    saveTemperature();
    return desiredTemperature;
}

// Setup LEDs
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

// Attach a Slider widget to the Virtual pin 4 - and control the web desired temperatuer
BLYNK_WRITE(V4) {
    if (param.asInt() > 0)
    {
        webDesired = param.asDouble();
    }
}

// Attach a switch widget to the Virtual pin 6 - and demand continuous web control
BLYNK_WRITE(V6) {
    webPerm = param.asInt();
}

// Setup
void setup()
{
    Wire.setSpeed(CLOCK_SPEED_100KHZ);
    Wire.begin();
    pinMode(LED, OUTPUT);               // sets pin as output
    matrix1.begin(0x70);
    matrix2.begin(0x71);
    setupMatrix(matrix1);
    setupMatrix(matrix2);
    Particle.variable("call",       &isHeatOn,              BOOLEAN);
    Particle.variable("fTemp",      &fTemp,                 DOUBLE);
    Particle.variable("hum",        &currentHumidity,       INT);
    Particle.variable("pot",        &potValue,              INT);
    Particle.variable("temp",       &currentTemperature,    INT);
    Particle.variable("set",        &desiredTemperature,    INT);
    Particle.variable("stat",       &I2C_Status,            INT);
    loadTemperature();
    pinMode(HEAT_PIN,   OUTPUT);
    pinMode(POT_PIN,    INPUT);
    myTimer.begin(OnTimerDim, displayTime, hmSec);

    // Time schedule stuff
    rtc.begin(&UDPClient, "pool.ntp.org");
    rtc.setTimeZone(-5); // gmt offset
    for (int day=0; day<7; day++) for (int num=0; num<4; num++)
    {
        if (fHourCh[day][num] > 24) fHourChErr = true;
        fHourCh[day][num] = fHourCh[day][num]+day*24;
    }
    for (int day=0; day<7; day++) for (int num=0; num<3; num++)
    {
        if (fHourCh[day][num] >= fHourCh[day][num+1]) fHourChErr = true;
    }
    Serial.begin(9600);
    delay(5000); // Allow board to settle
    Blynk.begin(blynkAuth);
}

// Loop
void loop()
{
    unsigned long   now = millis();         // Keep track of time
    static uint8_t  dayOfWeek;              // Day of week integer
    static uint8_t  hour;                   // Hour integer
    static uint8_t  minute;                 // Minutes integer
    static float    fHour;                  // Combined time value
    static int      wait = 0;               // Counter for processing
    Blynk.run();
    if (!wait)
    {
        wait = WAIT;
        delay(40);

        // Read sensors
        Wire.requestFrom(TEMP_SENSOR, 4); // request 4 bytes information
        uint8_t b       = Wire.read();
        uint8_t bend    = Wire.endTransmission();
        I2C_Status      = bend;
        //I2C_Status = b >> 6;
        int humidity    = (b << 8) & 0x3f00;
        humidity        |= Wire.read();
        float percentHumidity = humidity / 163.83;
        currentHumidity = roundf(percentHumidity);
        int temp        = (Wire.read() << 6) & 0x3fc0;
        temp            |= Wire.read() >> 2;
        fTemp           = (float(temp)*165.0/16383.0 - 40.0)*1.8 + 32.0 + TEMPCAL; // convert to fahrenheit and calibrate
        currentTemperature = roundf(fTemp);
        if (verbose>1)
        {
            Serial.printf("I2C = %ld, Hum Rel= %f, Temp = %f\n", I2C_Status, percentHumidity, fTemp);
        }
    }

    //interrogate pot;
    potValue = 4095 - analogRead(POT_PIN);
    if ((WAIT==wait) & (verbose>2)) 
    {
        Serial.printf("Pot reading= %ld\n", potValue);
    }

    // interrogate schedule
    if (WAIT==wait)
    {
        currentTime = rtc.now(); 
        if (currentTime!=lastTime)
        {
            #ifndef FAKETIME
                dayOfWeek = rtc.dayOfWeek(currentTime); 
                hour      = rtc.hour(currentTime);
                minute    = rtc.minute(currentTime);
            #else
                // Rapid time passage simulation to test schedule functions
                dayOfWeek = rtc.minute(currentTime)*7/6;      // minutes = days
                hour      = rtc.second(currentTime)*24/60;    // seconds = hours
                minute    = 0;                                // forget minutes
            #endif
            fHour     = float(dayOfWeek)*24.0 + float(hour) + float(minute)/60.0;  // 0-6 days and 0 is Sunday
            
            // Find spot in schedules
            unsigned int day = int(fHour/24);                   // Day known apriori
            unsigned int num = 4;
            while ( (num>0) & (fHour<fHourCh[day][num])) num--; // Event =  num
            schedValue = fTempCh[day][num-1];
        } // currentTime!=lastTime
    } // WAIT==wait
    
    // If user has adjusted the potentiometer (overrides schedule until next schedule change)
    if (fabsf(potValue - lastChangedPot) > 16)  // adjust from 64 because my range is 1214 not 4095
    {
        // Don't set temp on boot
        if (lastChangedPot >= 0)
        {
	        // my pot puts out 2872 - 4088 observed using following
            int t = roundf((float(potValue)-2872)/(4088-2872)*26+47);
            t = min(max(MINSET, t), MAXSET);
            setTemperature(t);
            isWebPerm = false;
            if (verbose>0)
            {
                Serial.printf("Setpont based on pot:  %ld\n", t);
            }
        }
        lastChangedPot = potValue;
    } // fabsf(potValue - lastChangedPot) > 16
    // Otherwise if web has adusted setpoint (overridden temporarily by pot, until next adjust)
    else if (fabsf(webDesired-lastChangedWebDesired)>0 | isWebPerm)
    {
        // Don't set temp on boot
        if (lastChangedWebDesired>0 & fabsf(webDesired-lastChangedWebDesired)>0)
        {
            int t = min(max(MINSET, webDesired), MAXSET);   
            setTemperature(t);
            if (verbose>0) Serial.printf("Setpoint based on web:  %ld\n", t);
        }
        lastChangedWebDesired = webDesired;
    } // fabsf(schedValue-lastChangedSched)>0
    // Otherwise if sshedule has adusted setpoint (overridden temporarily by pot, until next adjust)
    else if (fabsf(schedValue-lastChangedSched)>0)
    {
        // Don't set temp on boot
        if (lastChangedSched > 0)
        {
            int t = min(max(MINSET, schedValue), MAXSET);   
            if (fHourChErr)
            {
                Serial.println("***Table error, ignoring****");
            }
            else 
            {
                setTemperature(t);
                if (verbose>0) Serial.printf("Setpoint based on schedule:  %ld\n", t);
            }
        }
        lastChangedSched = schedValue;
    } // fabsf(schedValue-lastChangedSched)>0
    isHeatOn = desiredTemperature > currentTemperature;
    digitalWrite(HEAT_PIN, isHeatOn);
    digitalWrite(LED, isHeatOn);   

    // Blynk Web Control Function
    if (lastWebPerm!=webPerm) isWebPerm = webPerm;   // Acknowledge
    lastWebPerm = webPerm;
    if ( Particle.connected() & ((now-lastPublish)>=publish_delay) )
    {
        Blynk.virtualWrite(V0, isHeatOn);
        Blynk.virtualWrite(V5, isWebPerm);
        if (verbose>1) Serial.printf("'on' sent to blynk:  %ld\n", isHeatOn);
        if (verbose>1) Serial.printf("'set' sent to blynk:  %ld\n", desiredTemperature);
        Blynk.virtualWrite(V1, desiredTemperature);
        if (verbose>1) Serial.printf("'temp' sent to blynk:  %f\n", fTemp);
        Blynk.virtualWrite(V2, fTemp);
        if (verbose>1) Serial.printf("hum' sent to blynk:  %ld\n", currentHumidity);
        Blynk.virtualWrite(V3, currentHumidity);
        if (verbose>1) Serial.printf("'set' received from blynk:  %ld\n", webDesired);
        if (verbose>1) Serial.printf("Perm' acknowledge to blynk:  %ld\n", isWebPerm);
        if (verbose>1) Serial.printf("'Perm' received from blynk:  %ld\n", webPerm);
        lastPublish = now;
    } // now-lastPublish > publish_delay

    --wait;
}  // loop


