#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "DHT.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include <SD.h>

SoftwareSerial mySerial(8, 7);

Adafruit_GPS GPS(&mySerial);
#define GPSECHO  false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

// DHT22 temperature/humidity sensor
#define DHTPIN_22 3        // what pin we're connected to
#define DHTTYPE_22 DHT22      // DHT 22  (AM2302)
DHT dht_22(DHTPIN_22, DHTTYPE_22);

//BMP085 pressure/temperature sensor
Adafruit_BMP085 bmp = Adafruit_BMP085(10085);

// Set the pins used
#define chipSelect 10

void setup()  
{
    
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
//  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);
  delay(1000);

  dht_22.begin();      
  
 /* Initialise the PRESSURE sensor */
  bmp.begin();
  
  SD.begin(chipSelect);

// LED status light  
  pinMode(4, OUTPUT);  
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();

float pres, temp_pres, alt; 

void loop()                     // run over and over again
{
  
// READ RH AND TEMP FROM DHT22  
float h_22 = dht_22.readHumidity();
float t_22 = dht_22.readTemperature();  
  
// READ IN PRESSURE
sensors_event_t event;
bmp.getEvent(&event);
if (event.pressure)
{
pres = event.pressure;
bmp.getTemperature(&temp_pres);
//float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;  
float seaLevelPressure = 1016.9;  
alt = bmp.pressureToAltitude(seaLevelPressure,event.pressure,temp_pres); 
}
  
// DO GPS STUFF  
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer  

// WRITE DATA TO FILE  
Serial.println(F("WRITING TO SD CARD"));
File logfile = SD.open("datalog.txt", FILE_WRITE);    

    digitalWrite(4, HIGH);   // Turn on the LED
    delay(200);              // Wait for one second
    digitalWrite(4, LOW);    // Turn off the LED

logfile.print(F("20"));logfile.print(GPS.year,DEC);logfile.print(F("-"));

if (GPS.month < 10) {
  logfile.print(F("0"));logfile.print(GPS.month,DEC);logfile.print(F("-"));
}else{
  logfile.print(GPS.month,DEC);logfile.print(F("-"));
}
if (GPS.day < 10) {
  logfile.print(F("0"));logfile.print(GPS.day,DEC);logfile.print(F("-"));
}else{
  logfile.print(GPS.day,DEC);logfile.print(F("-"));
}
if (GPS.hour < 10) {
  logfile.print(F("0"));logfile.print(GPS.hour,DEC);
}else{
  logfile.print(GPS.hour,DEC);
}
if (GPS.minute < 10) {
  logfile.print(F("0"));logfile.print(GPS.minute,DEC);logfile.print(F(":"));
}else{
  logfile.print(GPS.minute,DEC);logfile.print(F(":"));
}
if (GPS.seconds < 10) {
  logfile.print(F("0"));logfile.print(GPS.seconds,DEC);logfile.print(F(", "));
}else{
  logfile.print(GPS.seconds,DEC);logfile.print(F(", "));
}
    logfile.print((int)GPS.fix);logfile.print(F(", "));
    logfile.print((int)GPS.fixquality);logfile.print(F(", "));
   
    if (GPS.fix) {      
      logfile.print((int)GPS.satellites);logfile.print(F(", "));                              
      logfile.print(ToDecimalDegrees(GPS.latitude),6);logfile.print(F(", "));      
      logfile.print(ToDecimalDegrees(GPS.longitude),6);logfile.print(F(", "));            
      logfile.print(GPS.altitude,7);logfile.print(F(", "));                  
      logfile.print(GPS.speed,5);logfile.print(F(", "));   
    }      
    else
    {
      logfile.print(F("99"));logfile.print(F(", "));                              
      logfile.print(F("999999"));logfile.print(F(", "));      
      logfile.print(F("999999"));logfile.print(F(", "));            
      logfile.print(F("9999999"));logfile.print(F(", "));                  
      logfile.print(F("99999"));logfile.print(F(", "));
    }
    
    logfile.print(pres);logfile.print(F(", "));
    logfile.print(temp_pres);logfile.print(F(", "));
    logfile.print(alt);logfile.print(F(", "));
    logfile.print(t_22);logfile.print(F(", "));
    logfile.print(h_22);logfile.println(F(", "));
    logfile.close();
}
}

// CONVERT LAT/LON TO DECIMAL DEGREES
float ToDecimalDegrees(float formattedLatLon)
{
  float decDegrees = (float)((int)formattedLatLon / 100);
  float decMinutes = formattedLatLon - (decDegrees * 100);
  float fractDegrees = decMinutes / 60.0;
 
  return decDegrees + fractDegrees;
}