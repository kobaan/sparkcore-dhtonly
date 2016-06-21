#include "SparkCarbon.h"
#include "SparkTime.h"

#include <math.h>

#define REED0 D2 // Reed0 - Window Left
#define REED1 D4 // Reed1 - Window Right
#define PIR D7 // PIR - Motion Detection
#define LDR A0 // LDR - Light Intensity
#define DHTPIN D6 // DHT-Pin - Temperature / Humidity
#define DHTTYPE DHT11

#define boolean bool
#define MAXTIMINGS 85

#define DHT11 11
#define DHT22 22
#define DHTNAN 999999

class DHT {
private:
    uint8_t data[6];
    uint8_t _pin, _type, _count;
    bool read(void);
    unsigned long _lastreadtime;
    bool firstreading;
public:
    DHT(uint8_t pin, uint8_t type, uint8_t count=6);
    void begin(void);
    float readTemperature(void);
    float convertCtoF(float);
    float readHumidity(void);
};

DHT::DHT(uint8_t pin, uint8_t type, uint8_t count) {
    _pin = pin;
    _type = type;
    _count = count;
    firstreading = true;
}

void DHT::begin(void) {
    // set up the pins!
    pinMode(_pin, INPUT);
    digitalWrite(_pin, HIGH);
    _lastreadtime = 0;
}

float DHT::readTemperature(void) {
    float _f;
    if (read()) {
        switch (_type) {
            case DHT11:
                _f = data[2];
                return _f;
                
            case DHT22:
                _f = data[2] & 0x7F;
                _f *= 256;
                _f += data[3];
                _f /= 10;
                
                if (data[2] & 0x80)
                    _f *= -1;
                return _f;
        }
    }
    
    return DHTNAN;
}

float DHT::readHumidity(void) {
    float _f;
    if (read()) {
        switch (_type) {
            case DHT11:
                _f = data[0];
                return _f;
            case DHT22:
                _f = data[0];
                _f *= 256;
                _f += data[1];
                _f /= 10;
                return _f;
        }
    }
    return DHTNAN;
}

bool DHT::read(void) {
    uint8_t laststate = HIGH;
    uint8_t counter = 0;
    uint8_t j = 0, i;
    unsigned long currenttime;
    
    // pull the pin high and wait 250 milliseconds
    digitalWrite(_pin, HIGH);
    delay(250);
    
    currenttime = millis();
    if (currenttime < _lastreadtime) {
        // ie there was a rollover
        _lastreadtime = 0;
    }
    
    if (!firstreading && ((currenttime - _lastreadtime) < 2000)) {
        //delay(2000 - (currenttime - _lastreadtime));
        return true; // return last correct measurement
    }
    
    firstreading = false;
    _lastreadtime = millis();
    
    data[0] = data[1] = data[2] = data[3] = data[4] = 0;
    
    // now pull it low for ~20 milliseconds
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    delay(20);
    noInterrupts();
    digitalWrite(_pin, HIGH);
    delayMicroseconds(40);
    pinMode(_pin, INPUT);
    
    // read in timings
    for ( i=0; i< MAXTIMINGS; i++) {
        counter = 0;
        
        while (digitalRead(_pin) == laststate) {
            counter++;
            delayMicroseconds(1);
            if (counter == 255)
                break;
        }
        
        laststate = digitalRead(_pin);
        
        if (counter == 255)
            break;
        
        // ignore first 3 transitions
        if ((i >= 4) && (i%2 == 0)) {
            // shove each bit into the storage bytes
            data[j/8] <<= 1;
            if (counter > _count)
                data[j/8] |= 1;
            j++;
        }
    }
    interrupts();
    
    // check we read 40 bits and that the checksum matches
    if ((j >= 40) &&  (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)))
        return true;
    return false;
}

float oversampleRead(uint16_t input_pin,uint8_t times) {
	float average=0.0;
	for (uint8_t i=0;i<times;i++) {
        average+=analogRead(input_pin);
	}
	average=average/times;
	return average;
}

DHT dht(DHTPIN, DHTTYPE);

bool dhtfail=0; // valid DHT readings ?
float dhthumidity; // DHT humidity
float dhttemperature; // DHT temperature
char humidity[5]; // humidity
char temperature[5]; // temperature
char reed0[2]; // left window
char reed1[2]; // right window
char motion[2]; // PIR motion
char light[5]; // light intensity

const unsigned int loopdelay=3000; // only query sensors every 3 seconds
unsigned long looptime;
TCPClient tcpClient;
SparkCarbon carbon;
UDP UDPClient;
SparkTime rtc;
bool sending = false;	// Failsafe to make sure we don't try to post while already posting (this is probably entirely unneeded, but I'm paranoid)
unsigned int lowcount=0;

void setup() {
 pinMode(REED0, INPUT_PULLUP);
 pinMode(REED1, INPUT_PULLUP);
 pinMode(PIR, INPUT);
 pinMode(LDR, INPUT);
 rtc.begin(&UDPClient, "storage.local");
 rtc.setTimeZone(+1);
 carbon.begin(&tcpClient, { 10, 1, 1, 4 });
 dht.begin();
 looptime=millis();
}

void loop() {
   if ((millis() > looptime + loopdelay) && sending==false) {
     lowcount++;
     sending = true;
     uint32_t now = rtc.nowEpoch();
     
     // Check left window reed
     if (digitalRead(REED0)) {
        sprintf(reed0,"0");
     }  else {
        sprintf(reed0,"1");
     }
     carbon.sendData("sparkcore.2.reed0", reed0, now);
     
     // Check right window reed
     if (digitalRead(REED1)) {
        sprintf(reed1,"0");
     }  else {
        sprintf(reed1,"1");
     }
     carbon.sendData("sparkcore.2.reed1", reed1, now);
     
     // Check PIR motion
     if (digitalRead(PIR)) {
        sprintf(motion,"1");
     }  else {
        sprintf(motion,"0");
     }
     carbon.sendData("sparkcore.2.motion", motion, now);
     
     lowcount++;
     if (lowcount >= 10) { // report those less often
       // Check light intensity
       sprintf(light,"%0.1f",(oversampleRead(LDR,8)/40.96));
       carbon.sendData("sparkcore.2.light", light, now);
     
       // Check Humidity & Temperature
       dhtfail = 0;
       dhthumidity = dht.readHumidity();
       dhttemperature = dht.readTemperature();
       if (dhttemperature==DHTNAN || dhthumidity==DHTNAN) {
           dhtfail = 1;
       } else {
           dhtfail = 0;
           sprintf(humidity,"%.1f",dhthumidity);   
           carbon.sendData("sparkcore.2.humidity", humidity, now);
           sprintf(temperature,"%.1f",dhttemperature);   
           carbon.sendData("sparkcore.2.temperature", temperature, now);
       }
       lowcount=0;
     }
     
     sending = false;
     looptime=millis(); 
   }
}
