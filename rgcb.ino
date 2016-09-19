

/*Eli's geocache box by Eli Shafer 9/9/2016
 * 
 * This is my version of the Reverse Geocache Box.
 * Udates include use of a Nokia LCD screen and customisations.
 * 
 * 
 */



/* Engagement Box   by Kenton Harris 11/12/2012
Reverse Geocache Box
This program unlocks a box that has reached a certain location.

The device uses the following products from Adafruit Industries:
Arduino Uno: https://www.adafruit.com/products/50
Ultimate GPS (version1): http://www.adafruit.com/products/746
16x2 Character LCD: https://www.adafruit.com/products/181
TPro Micro Servo SG90: https://www.adafruit.com/products/169
Half Sized Perma proto: https://www.adafruit.com/products/571

Tutorials for these products found on learn.adafruit.com helped with much of this.
Copyright (c) 2012, Adafruit Industries
All rights reserved.

Thanks to bnordlund9 for much of the code. This is  simplified verison of his Geobox found here:
http://www.youtube.com/watch?v=g0060tcuofg
Credit to Mikal Hart of http://arduiniana.org/ for the original idea of Reverse Geocache.

*/
#include <math.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#define GPS_RX_PIN 7
#define GPS_TX_PIN 8
SoftwareSerial mySerial(GPS_RX_PIN,GPS_TX_PIN);
// The TinyGPS++ object
TinyGPSPlus gps;
#define GPSECHO false                             //make true to debug GPS
boolean usingInterrupt = false;
void useInterrupt(boolean);
#define radians(angleDegrees) (angleDegrees * M_PI / 180.0)

//TODO Redo GPS to make it work with tinyGPS++


//PCD libtrary init
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

// Hardware SPI (faster, but must use certain hardware pins):
// SCK is LCD serial clock (SCLK) - this is pin 13 on Arduino Uno
// MOSI is LCD DIN - this is pin 11 on an Arduino Uno
// pin 5 - Data/Command select (D/C)
// pin 4 - LCD chip select (CS)
// pin 3 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(5, 4, 3);

//Servo
#include <PWMServo.h>
PWMServo servoLatch;

//Declarations
#define deg2rad  0.01745329251994
#define rEarth  6371000.0                                          //can replace with 3958.75 mi, 6370.0 km, or 3440.06 NM
float range = 3000;                                                      // distance from HERE to THERE
//String here;                                                             // read from GPS


//TODO rename defines:
bool gpsWasFixed = HIGH;                                                  // did the GPS have a fix?
#define ledFix  4                                                          // pin for fix LED
#define servoPin  9                                                        // pin for servo
#define servoLock  110                                                     // angle (deg) of "locked" servo
#define servoUnlock  0                                                     // angle (deg) of "unlocked" servo

//String there = "N32 50.026, E34 58.688";                                //Desired Location goes here. Make sure you use the same syntax and number of characters

struct coordinates{
  float lat = 32.8334561;
  float lon = 34.9787866;
}there;
/*
struct coordinates there;
there.lat = 32.8285484;
there.lon = 34.9893783;
*/

void setup()
{
  servoLatch.attach(SERVO_PIN_B);
  servoLatch.write(servoLock);
  delay(50);
  display.begin();
  Serial.begin(115200);
  Serial.println("Debug GPS Test:");

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);                          // RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);                             // 1 Hz update rate
  useInterrupt(true);                                                    // reads the steaming data in a background
  delay(1000);
  

}

void loop(){
  // Parse GPS and recalculate RANGE
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))                                      // also sets the newNMEAreceived() flag to false
      return;                                                            // We can fail to parse a sentence in which case we should just wait for another
  }
    if (GPS.fix) {
    gpsWasFixed = HIGH;
    digitalWrite(ledFix, HIGH);

  
    //here = gps2string ((String) GPS.lat, GPS.latitude, (String) GPS.lon, GPS.longitude);
    range = haversine(GPS.latitudeDegrees*deg2rad, GPS.longitudeDegrees*deg2rad, there.lat*deg2rad, there.lon*deg2rad);
    Serial.print("Here: ");                                        //for GPS debug
    Serial.print(GPS.latitudeDegrees); Serial.println(GPS.longitudeDegrees);
    Serial.print("There: ");
    Serial.print(there.lat,3);Serial.println(there.lon,3);
    Serial.print("Range: ");
    Serial.print(range);
    Serial.println("m");
    
    /*Original
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Distance to LOYL");
    //lcd.setCursor(0,1);
    //lcd.print("                ");
    lcd.setCursor(0,1);
    lcd.print(range);
    */
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(BLACK);
    display.setCursor(0,0);
    display.println("Distance to LOYL");
    display.setTextSize(1);
    display.println(range);
    display.display();

    delay(500);
  }
  else {                                                              //No GPS fix- take box outside
    /*
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Hello Person!");
    lcd.setCursor(0,1);
    lcd.print("Take me outside!");
    */
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(BLACK);
    display.setCursor(0,0);
    display.println("Yo!");
    display.println("Take me outside");
    display.display();
    delay(200);
  }
  
  if (range < 300.0){
    servoLatch.write(servoUnlock);
    delay(1000);
    /*
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Person: you can now ");
    lcd.setCursor(0,1);
    lcd.print("Open me!");
    */
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(BLACK);
    display.setCursor(0,0);
    display.println("Yo!");
    display.println("You can now open me!");
    display.display();

    delay(5000);
  }
}


SIGNAL(TIMER0_COMPA_vect) {
  // Interrupt is called once a millisecond, looks for any new GPS data, and stores it
  char c = GPS.read();
  if (GPSECHO)
    if (c) UDR0 = c;  
}

void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

String int2fw (int x, int n) {
  // returns a string of length n (fixed-width)
  String s = (String) x;
  while (s.length() < n) {
    s = "0" + s;
  }
  return s;
}
/*
String gps2string (String lat, float latitude, String lon, float longitude) {
  // returns "Ndd mm.mmm, Wddd mm.mmm";
  int dd = (int) latitude/100;
  int mm = (int) latitude % 100;
  int mmm = (int) round(1000 * (latitude - floor(latitude)));
  String gps2lat = lat + int2fw(dd, 2) + " " + int2fw(mm, 2) + "." + int2fw(mmm, 3);
  dd = (int) longitude/100;
  mm = (int) longitude % 100;
  mmm = (int) round(1000 * (longitude - floor(longitude)));
  String gps2lon = lon + int2fw(dd, 3) + " " + int2fw(mm, 2) + "." + int2fw(mmm, 3);
  String myString = gps2lat + ", " + gps2lon;
  return myString;
};

float string2radius (String myString) {
  // returns a floating-point number: e.g. String myString = "Radius: 005.1 NM";
  float r = ((myString.charAt(8) - '0') * 100.0) + ((myString.charAt(9) - '0') * 10.0) + ((myString.charAt(10) - '0') * 1.0) + ((myString.charAt(12) - '0') * 0.10);
  return r;
};*/
/*
float string2lat (String myString) {
  // returns radians: e.g. String myString = "N38 58.892, W076 29.177";
  float lat = ((myString.charAt(1) - '0') * 10.0) + (myString.charAt(2) - '0') * 1.0 + ((myString.charAt(4) - '0') / 6.0) + ((myString.charAt(5) - '0') / 60.0) + ((myString.charAt(7) - '0') / 600.0) + ((myString.charAt(8) - '0') / 6000.0) + ((myString.charAt(9) - '0') / 60000.0);
  Serial.print("float lat: ");
  Serial.println(lat);
  lat *= deg2rad;
  if (myString.charAt(0) == 'S')
    lat *= -1;                                                           // Correct for hemisphere
  return lat;
};

float string2lon (String myString) {
  // returns radians: e.g. String myString = "N38 58.892, W076 29.177";
  float lon = ((myString.charAt(13) - '0') * 100.0) + ((myString.charAt(14) - '0') * 10.0) + (myString.charAt(15) - '0') * 1.0 + ((myString.charAt(17) - '0') / 6.0) + ((myString.charAt(18) - '0') / 60.0) + ((myString.charAt(20) - '0') / 600.0) + ((myString.charAt(21) - '0') / 6000.0) + ((myString.charAt(22) - '0') / 60000.0);
  Serial.print("float lon: ");
  Serial.println(lon);
  lon *= deg2rad;
  if (myString.charAt(12) == 'W')
    lon *= -1;                                                           // Correct for hemisphere
  return lon;
};
*/

float haversine (double lat1, double lon1, double lat2, double lon2) {
  // returns the great-circle distance between two points (radians) on a sphere
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  lon1 = radians(lon1);
  lon2 = radians(lon2);
  double h = sq((sin((lat1 - lat2) / 2.0))) + (cos(lat1) * cos(lat2) * sq((sin((lon1 - lon2) / 2.0))));
  double d = 2.0 * rEarth * asin (sqrt(h)); 
  //Serial.println(d);
  return d;
  
};
