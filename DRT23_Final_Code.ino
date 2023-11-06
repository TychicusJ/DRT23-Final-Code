// DRT23- Autonomous Waypoint Navigation Car
// Arthur: Demetria Labat, Rony Akhter, & Tychicus Johnson
// Created 6 Novermber 2023

#include <Wire.h> // Used by I2C and QMC5883L compass
#include <QMC5883LCompass.h> // Library for the compass
#include <SoftwareSerial.h> // Software Serial for Serial Communications
#include <TinyGPSPlus.h> // Tiny GPS Plus Library - Download from http://arduiniana.org/libraries/tinygpsplus/
#include <AFMotor.h>

/*
   This sketch demonstrates the normal use of a TinyGPS++(TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and using the 
   9600-baud serial GPS device hooked up on pins 51(rx) and 49(tx) and a QMC5883 Magnetic Compass
   connected to the SCL/SDA pins.
*/

static const int RXPin = 51, TXPin = 49;                            // RX and TX pins are reverse on the board RX goto 49 and TX goto 51 
static const uint32_t GPSBaud = 9600;                               // Baud rate for GPS communication

// Assign a Unique ID to the QMC5883 Compass Sensor
QMC5883LCompass compass;                                            // QMC5883L Compass object
TinyGPSPlus gps;                                                    // The TinyGPSPlus object
SoftwareSerial ss(RXPin, TXPin);                                    // The serial connection to the GPS device

// setting up the value longitude, latitude, bearing, and heading
float latc,logc;                                                    // Current latitude and longitude
float userLat = 0.0;                                                // Latitude for user-defined waypoint
float userLng = 0.0;                                                // Longitude for user-defined waypoint
bool waypointSet = false;                                           // Flag to check if a new waypoint is set
float bearing;                                                      // Bearing angle between current and destination points
float heading;                                                      // Compass heading

// Setup Drive Motors using the Adafruit Motor Controller version 1.0 Library

AF_DCMotor motor1(1, MOTOR12_64KHZ);                               // create motor #1, 64KHz pwm
AF_DCMotor motor2(2, MOTOR12_64KHZ);                               // create motor #2, 64KHz pwm
AF_DCMotor motor3(3, MOTOR12_64KHZ);                               // create motor #3, 64KHz pwm
AF_DCMotor motor4(4, MOTOR12_64KHZ);                               // create motor #4, 64KHz pwm

int turn_Speed = 200;                                              // motor speed when using the compass to turn left and right
int mtr_Spd = 255;                                                 // motor speed when moving forward and reverse



void setup() 
{
  Wire.begin();                                                    // Initialize I2C communication
  Serial.begin(9600);                                             // Initialize serial communication with the computer
  ss.begin(GPSBaud);                                              // Initialize serial communication with the GPS device
  compass.init();                                                 // Initialize the compass sensor
  Serial1.begin(9600);                                            // Serial 1 is for Bluetooth communication - DO NOT MODIFY - JY-MCU HC-06 v1.40

  Serial1.println(F("Enter the next waypoint coordinates (latitude and longitude) in decimal degrees."));
  Serial1.println(F("Use the format: LATITUDE, LONGITUDE"));
}

void loop() 
{
  if (Serial1.available() > 0)                                    // Check for user input
  {
    String userInput = Serial1.readStringUntil('\n');
    parseUserInput(userInput);
  }

  if (waypointSet)                                                // Check if a waypoint has been set before processing GPS data

  headingcal();                                                   // Call function to read compass heading
    delay(200);
  gpsdata();                                                      // Call function to read GPS data
    delay(200);
  gpsheading();                                                   // Call function to calculate GPS heading
    delay(200);
  navigateToWaypoint();                                           // Call function to control motors based on heading
    delay(200);

}

void parseUserInput(String userInput)
{
  int commaIndex = userInput.indexOf(',');
  if (commaIndex != -1)
  {
    String latStr = userInput.substring(0, commaIndex);
    String lngStr = userInput.substring(commaIndex + 1);

    userLat = latStr.toFloat();                                   // Convert the string to a float
    userLng = lngStr.toFloat();                                   // Convert the string to a float
    waypointSet = true;                                           // Set the waypoint flag

    Serial1.println("New waypoint coordinates set.");
    Serial1.print("Latitude: ");
    Serial1.print(userLat, 6);
    Serial1.print(", Longitude: ");
    Serial1.println(userLng, 6);
  }
  else
  {
    Serial1.println("Invalid input format. Use the format: LATITUDE, LONGITUDE");
  }
}

// GPS data retrieval function
void gpsdata()
{
  smartDelay(1000);                                              // Wait for GPS data to stabilize
  unsigned long start;
  double lat_val, lng_val, alt_m_val;
  bool loc_valid, alt_valid;
  lat_val = gps.location.lat();
  loc_valid = gps.location.isValid();
  lng_val = gps.location.lng();
  alt_m_val = gps.altitude.meters();
  alt_valid = gps.altitude.isValid();

  if (!loc_valid)
    {
      Serial1.print("Latitude : ");
      Serial1.println("*****");                                    // Print placeholder for invalid latitude
      Serial1.print("Longitude : ");
      Serial1.println("*****");                                    // Print placeholder for invalid longitude
      delay(100);
    }

  else
    {
      Serial1.println("GPS READING: ");
      // DegMinSec(lat_val);
      Serial1.print("Latitude in Decimal Degrees : ");
      Serial1.println(gps.location.lat(), 6);                      // Print latitude with 6 decimal places

      // DegMinSec(lng_val); 
      Serial1.print("Longitude in Decimal Degrees : ");
      Serial1.println(gps.location.lng(), 6);                      // Print longitude with 6 decimal places
      delay(100);
    }

  latc = lat_val;
  logc = lng_val;
}

// Smart delay function to wait for GPS data
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
    {
      while (ss.available()) 
      gps.encode(ss.read());
    } while (millis() - start < ms);
}

// GPS heading calculation function
void gpsheading()
{
  float x, y, deltalog, deltalat;
  deltalog = userLng - logc;
  deltalat = userLat - latc;

  x = cos (userLat) * sin(deltalog);
  y=(cos(latc) * sin(userLat)) -(sin(latc) *cos(userLat) * cos(deltalog));
  
  bearing = (atan2(x, y)) * (180 / 3.14);
  Serial1.print("bearing: ");
  Serial1.println(bearing);

  // Radius of the Earth in meters
  double earthRadius = 6371000.0;                                    // Approximate value for Earth's radius

  // Haversine formula
  float a,d,c;
  a = (((sin(deltalat / 2))) * (sin(deltalat / 2))) + ((cos(latc)) * (cos(userLat)) * (((sin(deltalog / 2))) *(sin(deltalog / 2)))  );
  c = 2*(atan2(sqrt(a), sqrt(1 - a)));
  d = earthRadius * c; 
  //Serial1.println(d);
}

// Compass heading calculation function
double headingcal()
{
  int x, y, z;
  char myArray[3];
	compass.read();                                                       // Read data from the compass sensor
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();
  delay(100);

 double heading = atan2(x, y) / 0.0174532925;
  if(heading < 0) 
    {
      heading += 360;
    }
  
  heading = 360 - heading;                                            // N=0/360, E=90, S=180, W=270
  Serial1.print("heading: ");
  Serial1.println(heading);  

  return heading;                                                    // Return the calculated heading
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2)
{
  // Convert latitude and longitude from degrees to radians
  lat1 = radians(lat1);
  lon1 = radians(lon1);
  lat2 = radians(lat2);
  lon2 = radians(lon2);

  // Radius of the Earth in meters
  double earthRadius = 6371000.0; // Approximate value for Earth's radius

  // Haversine formula
  double dlon = lon2 - lon1;
  double dlat = lat2 - lat1;
  double a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double distance = earthRadius * c;

  return distance;
}

// Motor control functions
void navigateToWaypoint() {
  // Calculate the distance to the target waypoint
  double distanceToWaypoint = calculateDistance(gps.location.lat(), gps.location.lng(), userLat, userLng);

  // Set a threshold distance (adjust as needed)
  double waypointThreshold = 10.0; // meters

  // Define your heading error tolerance (in degrees) for motor control
  double headingErrorTolerance = 5.0; // Adjust this value as needed

  double desiredHeading = 0;

  // Continue adjusting motors until the waypoint is reached
  while (distanceToWaypoint > waypointThreshold) {
    // Calculate the desired heading to the waypoint
     desiredHeading = headingcal();

    // Calculate the heading error
    double currentAzimuth = compass.getAzimuth();
    double headingError = desiredHeading - currentAzimuth;

    // Normalize the heading error to be between -180 and 180 degrees
    while (headingError > 180.0) headingError -= 360.0;
    while (headingError <= -180.0) headingError += 360.0;

    // Adjust the motors based on the heading error
    if (fabs(headingError) > headingErrorTolerance) {
      if (headingError < 0) {
        Left(); // Turn left when the heading error is significant
      } else {
        Right(); // Turn right when the heading error is significant
      }
    } else {
      Forward(); // Go forward when the heading error is small
    }

    // Update the distance to the waypoint
    distanceToWaypoint = calculateDistance(gps.location.lat(), gps.location.lng(), userLat, userLng);

    // You can add a delay here to control the loop frequency
    delay(100); // Adjust the delay as needed
  }

  // When the waypoint is reached, stop the motors
  Stop1();
  Serial1.println(F("Waypoint reached!"));
}

// Motor control functions for different directions
// **********************************************************************************************************************************************************************
void Forward()
{
  motor1.setSpeed(mtr_Spd);                                                   
  motor2.setSpeed(mtr_Spd);     
  motor3.setSpeed(mtr_Spd);     
  motor4.setSpeed(mtr_Spd);     
  
  motor1.run(FORWARD);                                                        // go forward all wheels for specified time
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(100);
}

// **********************************************************************************************************************************************************************
void Stop1()
{
  motor1.run(RELEASE);                                                         
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  delay(100);
}
// **********************************************************************************************************************************************************************
void Right()
{
  motor1.setSpeed(turn_Speed);                                                   
  motor2.setSpeed(turn_Speed);     
  motor3.setSpeed(turn_Speed);     
  motor4.setSpeed(turn_Speed);   

  motor1.run(RELEASE);                                              
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(RELEASE);
  delay(100);                                                           //delay for 100ms more responsive turn per push on bluetooth                   
}

// **********************************************************************************************************************************************************************
void Left()
{
  motor1.setSpeed(turn_Speed);                                                   
  motor2.setSpeed(turn_Speed);     
  motor3.setSpeed(turn_Speed);     
  motor4.setSpeed(turn_Speed);    
  
  motor1.run(FORWARD);                                                        // Turn left
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(FORWARD);
  delay(100);    
}
// **********************************************************************************************************************************************************************
