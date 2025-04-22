/*
  Project: Custom Rover Control via Wi-Fi
  Author: Armando Nicolella, Pierangelo Malfi
 

  Description:
  This project enables real-time control of a rover using Wi-Fi connectivity.
  It integrates IMU data processing with Kalman filtering, motor control via
  Adafruit Motor Shields, and web server functionalities for remote operation.

  Credits:
  - Based on the OSOYOO Wi-Fi Robot Car App and tutorials:
    https://osoyoo.com/2020/05/12/v2-1-robot-car-kit-for-arduino-tutorial-introduction/
  - OSOYOO Wi-Fi Robot Car App on the App Store:
    https://apps.apple.com/us/app/osoyoo-wifi-robot-car-app/id1455384383

  Note:
  This code has been customized to accommodate specific hardware configurations
  and additional functionalities beyond the original OSOYOO implementation.
*/


#include <SPI.h>                      // SPI communication library
#include <WiFiNINA.h>                 // Library for WiFi connectivity with NINA module
#include <Arduino_LSM6DS3.h>          // Library to interface with the LSM6DS3 IMU sensor
#include <Adafruit_MotorShield.h>     // Library for controlling motors via Adafruit Motor Shield
//#include <utility/wifi_drv.h>       // Optional low-level WiFi driver (commented out)
#include <Kalman.h>                   // Kalman filter library for sensor fusion
#include <SD.h>                       // Library to interface with SD cards


//IMPORTANT: CHOOSE YOUR PARAMETERS

// Set Variable for Rover Control

int vel = 200; ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#define RESTRICT_PITCH
float time0 = 0;
float timeX = 0;
int variabile = 0;
int myDelay = 2000;
int lagTime = 500;


/* IMU Data */
float accX, accY, accZ;  //[g]
float gyroX, gyroY, gyroZ;  //[dps]
float gyroDriftX, gyroDriftY, gyroDriftZ;
int16_t tempRaw;
double roll, pitch, yaw ;
double dt;
double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter
double gyroXrate, gyroYrate, gyroZrate;
double temperature;
uint32_t timer;

/* Create the Kalman instances */
Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanZ;

/* Set variable for SD Card */
const int chipSelect = 4;

// IMPORTANT: CHOOSE YOUR PARAMETERS

// Rover control parameters
int vel = 200;                         // Default velocity value (modifiable via app)

#define RESTRICT_PITCH                 // Enable pitch-restricted Kalman estimation

float time0 = 0;
float timeX = 0;
int variabile = 0;
int myDelay = 2000;                    // Delay for command sequencing
int lagTime = 500;                     // Delay used to manage lag between commands

/* IMU Data */
float accX, accY, accZ;                // Accelerometer readings [g]
float gyroX, gyroY, gyroZ;             // Gyroscope readings [°/s]
float gyroDriftX, gyroDriftY, gyroDriftZ; // Gyro drift compensation
int16_t tempRaw;                       // Raw temperature data from IMU
double roll, pitch, yaw;              // Orientation angles
double dt;                             // Time delta between IMU updates [s]

double gyroXangle, gyroYangle, gyroZangle; // Integrated gyro-only angles
double compAngleX, compAngleY, compAngleZ; // Complementary filter angles
double kalAngleX, kalAngleY, kalAngleZ;    // Kalman filter angles
double gyroXrate, gyroYrate, gyroZrate;    // Drift-compensated gyro rates
double temperature;                   // Processed temperature in °C
uint32_t timer;                       // Timer for delta time calculation

/* Kalman filter instances */
Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanZ;

/* SD card interface */
const int chipSelect = 4;             // Chip select pin for SD module

//////////////////// BUFFER SETUP

int const dimBuff = 2;                // Buffer size (recommended: 45)
float vettore[10 * dimBuff] = {};     // Data storage buffer

int skid = 2;                         // Drive configuration: 2 = skid steering

// Motor driver setup
Adafruit_MotorShield AFMS1 = Adafruit_MotorShield();  // Main motor shield (0x60 by default)

// Motor shield selection based on I2C address
#define ADA0x61
//#define ADA0x62

#if defined(ADA0x61)
// Configuration for address 0x61 (rocker on front bogie)
char ssid[] = "Rover_0061";           // WiFi network SSID
Adafruit_MotorShield AFMS2 = Adafruit_MotorShield(0x61); // Secondary shield

// Right motors
Adafruit_DCMotor *A0x60M1 = AFMS1.getMotor(1); // Front Right
Adafruit_DCMotor *A0x60M4 = AFMS1.getMotor(4); // Middle Right
Adafruit_DCMotor *A0x60M3 = AFMS1.getMotor(3); // Rear Right

// Left motors
Adafruit_DCMotor *A0x6nM2 = AFMS2.getMotor(2); // Front Left
Adafruit_DCMotor *A0x6nM4 = AFMS2.getMotor(4); // Middle Left
Adafruit_DCMotor *A0x6nM3 = AFMS2.getMotor(3); // Rear Left

const int scheda = 61;

#elif defined(ADA0x62)
// Configuration for address 0x62
char ssid[] = "Rover_0062";
Adafruit_MotorShield AFMS2 = Adafruit_MotorShield(0x62);

// Right motors
Adafruit_DCMotor *A0x6nM4 = AFMS2.getMotor(4); // Front Right
Adafruit_DCMotor *A0x6nM3 = AFMS2.getMotor(3); // Middle Right
Adafruit_DCMotor *A0x6nM2 = AFMS2.getMotor(2); // Rear Right

// Left motors
Adafruit_DCMotor *A0x60M4 = AFMS1.getMotor(4); // Front Left
Adafruit_DCMotor *A0x60M3 = AFMS1.getMotor(3); // Middle Left
Adafruit_DCMotor *A0x60M1 = AFMS1.getMotor(1); // Rear Left

const int scheda = 62;

#else
// Default configuration (address 0x64)
char ssid[] = "Rover_0064";
Adafruit_MotorShield AFMS2 = Adafruit_MotorShield(0x64);

// Right motors
Adafruit_DCMotor *A0x6nM4 = AFMS2.getMotor(4); // Front Right
Adafruit_DCMotor *A0x6nM3 = AFMS2.getMotor(3); // Middle Right
Adafruit_DCMotor *A0x6nM2 = AFMS2.getMotor(2); // Rear Right

// Left motors
Adafruit_DCMotor *A0x60M4 = AFMS1.getMotor(4); // Front Left
Adafruit_DCMotor *A0x60M3 = AFMS1.getMotor(3); // Middle Left
Adafruit_DCMotor *A0x60M1 = AFMS1.getMotor(1); // Rear Left

const int scheda = 64;
#endif

// WiFi connection state
int status = WL_IDLE_STATUS;

// UDP setup for receiving control commands from mobile app
char packetBuffer[5];                 // Small buffer for incoming commands
WiFiUDP Udp;                          // UDP communication object
unsigned int localPort = 8888;        // Listening port for incoming packets


void setup() {
  time0 = micros();                    // Initialize reference timestamp
  String message = "";                // Placeholder for future use

  // Initialize serial communication
  Serial.begin(57600);                // Serial monitor (debug)
  Serial1.begin(115200);             // UART for ESP module (initial AT command config)
  Serial1.write("AT+UART_DEF=9600,8,1,0,0\r\n");  // Set ESP default baud rate to 9600
  delay(200);
  Serial1.write("AT+RST\r\n");        // Reset ESP module
  delay(200);
  Serial1.begin(9600);                // Reinitialize Serial1 at 9600 (ESP mode)

  // Initialize both Adafruit Motor Shields
  AFMS1.begin();                      // Primary motor shield
  AFMS2.begin();                      // Secondary motor shield (I2C address selected earlier)

  // Initialize IMU sensor
  IMU.begin();
  delay(500);                         // Allow sensor to stabilize
  calibrateIMU(500, 500);             // Calibrate gyroscope

  // Read initial IMU values
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(accX, accY, accZ);
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
  }

  updatePitchRoll();                  // Compute initial roll and pitch
  updateYaw();                        // Compute initial yaw

  // Initialize Kalman filters with first sensor values
  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);
  kalmanZ.setAngle(yaw);

  // Initialize complementary and gyro-only estimations
  gyroXangle = roll;
  gyroYangle = pitch;
  gyroZangle = yaw;
  compAngleX = roll;
  compAngleY = pitch;
  compAngleZ = yaw;

  timer = micros();                   // Start loop timing

  // Check for WiFi shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    while (true);                     // Stop execution if shield is missing
  }

  // Start Access Point mode
  Serial.print("Attempting to start AP ");
  Serial.println(ssid);
  status = WiFi.beginAP(ssid);        // Start AP with configured SSID

  Serial.println("You're connected to the network");
  printWifiStatus();                  // Print AP info to Serial

  Udp.begin(localPort);               // Start listening for UDP packets
  Serial.print("Listening on port ");
  Serial.println(localPort);

  // Initialize SD card
  SD.begin(chipSelect);
}


/******************************************************************************** SETUP END *************************************************************************************************************/

/******************************************************************************** LOOP START *************************************************************************************************************/

void loop() {

  String message = "";

  readNwriteSD(); // Log IMU and filtered data to SD card

  // Check for incoming UDP packet from mobile app
  int packetSize = Udp.parsePacket();
  if (packetSize) {                               // If a packet is received
    Serial.print("Received packet of size ");
    Serial.println(packetSize);

    int len = Udp.read(packetBuffer, 255);        // Read packet into buffer
    if (len > 0) {
      packetBuffer[len] = 0;                      // Null-terminate buffer
    }

    char c = packetBuffer[0];                     // First character identifies the command

    // Command switch-case for control
    switch (c) {

      // Full forward motion
      case 'A':
        LevogiroPos(0, 0, 0, scheda);
        delay(lagTime);
        LevogiroPos(vel, 1, 1, scheda);
        break;

      // Full left turn
      case 'L':
        if (scheda == 61) {
          LevogiroPos(0, 0, 0, scheda);
          delay(lagTime);
          LevogiroPos(vel, 1000, 1, scheda);
        } else {
          LevogiroPos(0, 0, 0, scheda);
          delay(lagTime);
          LevogiroPos(vel, 1, 1000, scheda);
        }
        break;

      // Full right turn
      case 'R':
        if (scheda == 61) {
          LevogiroPos(0, 0, 0, scheda);
          delay(lagTime);
          LevogiroPos(vel, 1, 1000, scheda);
        } else {
          LevogiroPos(0, 0, 0, scheda);
          delay(lagTime);
          LevogiroPos(vel, 1000, 1, scheda);
        }
        break;

      // Full backward motion
      case 'B':
        LevogiroNeg(0, 0, 0, scheda);
        delay(lagTime);
        LevogiroNeg(vel, 1, 1, scheda);
        break;

      // Emergency stop (all motors)
      case 'E':
        LeftWheelsStop();
        RightWheelsStop();
        break;

      // Slight forward left
      case 'F':
        if (scheda == 61) {
          LevogiroPos(0, 0, 0, scheda);
          delay(lagTime);
          LevogiroPos(vel, 2, 1, scheda);
        } else {
          LevogiroPos(0, 0, 0, scheda);
          delay(lagTime);
          LevogiroPos(vel, 1, 2, scheda);
        }
        break;

      // Slight forward right
      case 'H':
        if (scheda == 61) {
          LevogiroPos(0, 0, 0, scheda);
          delay(lagTime);
          LevogiroPos(vel, 1, 2, scheda);
        } else {
          LevogiroPos(0, 0, 0, scheda);
          delay(lagTime);
          LevogiroPos(vel, 2, 1, scheda);
        }
        break;

      // Slight backward left
      case 'I':
        if (scheda == 61) {
          LevogiroNeg(0, 0, 0, scheda);
          delay(lagTime);
          LevogiroNeg(vel, 2, 1, scheda);
        } else {
          LevogiroNeg(0, 0, 0, scheda);
          delay(lagTime);
          LevogiroNeg(vel, 1, 2, scheda);
        }
        break;

      // Slight backward right
      case 'K':
        if (scheda == 61) {
          LevogiroNeg(0, 0, 0, scheda);
          delay(lagTime);
          LevogiroNeg(vel, 1, 2, scheda);
        } else {
          LevogiroNeg(0, 0, 0, scheda);
          delay(lagTime);
          LevogiroNeg(vel, 2, 1, scheda);
        }
        break;

      // Skid turn right
      case 'O':
        if (scheda == 61) {
          SkidSteering(0, 0, 0, scheda, skid);
          delay(lagTime);
          SkidSteering(vel, 1, 1, scheda, 1);
        } else {
          SkidSteering(0, 0, 0, scheda, skid);
          delay(lagTime);
          SkidSteering(vel, 1, 1, scheda, 0);
        }
        break;

      // Skid turn left
      case 'T':
        if (scheda == 61) {
          SkidSteering(0, 0, 0, scheda, skid);
          delay(lagTime);
          SkidSteering(vel, 1, 1, scheda, 0);
        } else {
          SkidSteering(0, 0, 0, scheda, skid);
          delay(lagTime);
          SkidSteering(vel, 1, 1, scheda, 1);
        }
        break;

      default:
        break; // Ignore unrecognized commands
    }
  }
}

// Print the current Wi-Fi status and IP address
void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  Serial.println();
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
  Serial.println();
}
