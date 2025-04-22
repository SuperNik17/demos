/*
 * web_server_control.ino
 * 
 * Authors: Armando Nicolella, Pierangelo Malfi
 * Affiliation: University of Naples Federico II – LAM4R (Laboratory of Applied Mechanics for Robotics)
 * Description: WiFi-based control and telemetry system for a rocker-bogie mobile robot.
 * This code handles IMU data processing (Kalman filter), SD logging, motor control via Adafruit Motor Shield,
 * and provides a web interface for basic rover interaction.
 * 
 * All rights reserved © LAM4R – UNINA
 */
 

#include <SPI.h>                      // SPI communication library
#include <WiFiNINA.h>                 // Library for WiFi connectivity with NINA module
#include <Arduino_LSM6DS3.h>          // Library to interface with the LSM6DS3 IMU sensor
#include <Adafruit_MotorShield.h>     // Library for controlling motors via Adafruit Motor Shield
//#include <utility/wifi_drv.h>       // Optional low-level WiFi driver (commented out)
#include <Kalman.h>                   // Kalman filter library for sensor fusion
#include <SD.h>                       // Library to interface with SD cards



//IMPORTANT: CHOOSE YOUR PARAMETERS
#define RESTRICT_PITCH
//#define SD_USE   //Uncomment if you want use SD instead of Serial/Webserver storage


// IMPORTANT: CHOOSE YOUR PARAMETERS
#define RESTRICT_PITCH              // Limit rotation to pitch to avoid gimbal lock
//#define SD_USE                    // Uncomment to use SD storage instead of Serial/Webserver

int keyIndex = 0;                   // Network key index (used only for WEP encryption)

Kalman kalmanX;                     // Kalman filter instance for X-axis
Kalman kalmanY;                     // Kalman filter instance for Y-axis
Kalman kalmanZ;                     // Kalman filter instance for Z-axis

/* IMU Data */
float accX, accY, accZ;             // Accelerometer values in g
float gyroX, gyroY, gyroZ;          // Gyroscope values in degrees per second
float gyroDriftX, gyroDriftY, gyroDriftZ;  // Gyroscope drift values
int16_t tempRaw;                    // Raw temperature reading from IMU

double roll, pitch, yaw;           // Orientation angles
double dt;                          // Delta time between updates

double gyroXangle, gyroYangle, gyroZangle;     // Angles calculated using only gyroscope data
double compAngleX, compAngleY, compAngleZ;     // Angles calculated using complementary filter
double kalAngleX, kalAngleY, kalAngleZ;        // Angles calculated using Kalman filter

double gyroXrate, gyroYrate, gyroZrate;        // Gyroscope rates
double temperature;                 // Processed temperature value

uint32_t timer;                     // Timer used to calculate delta time


// BUFFER SIZE DEFINITION
int const dimBuff = 45;                      // Recommended value: 70
float vettore[10 * dimBuff] = {};            // Buffer array for data storage

// WiFi server setup
int status = WL_IDLE_STATUS;                 // WiFi connection status
WiFiServer server(80);                       // Create server on port 80

// Rover control variables
float time0 = 0;
float timeX = 0;
int variabile = 0;
int vel = 0;
int myDelay = 2000;                          // Delay between actions in milliseconds
int lagTime = 500;                           // Communication lag compensation

// SD card configuration
const int chipSelect = 4;                    // Chip select pin for SD card module

// Adafruit Motor Shield setup
Adafruit_MotorShield AFMS1 = Adafruit_MotorShield();  // Primary motor shield

char pass[] = "12345678";                    // WiFi password (same for all rovers)

// Coordinate system follows the IMU orientation and forward direction of the front bogie
// Counter-clockwise (levogiro) reference frame

// ⚠ IMPORTANT NOTE:
// The naming of the motor shields (e.g., AFMS1, AFMS2) and the associated motor pins
// depend heavily on how the system has been physically assembled.
// Be sure to verify the actual wiring and motor connections before choosing or modifying
// the defined configuration below (ADA0x61, ADA0x62, etc.)


#define ADA0x61                               // Define which Motor Shield address is active
//#define ADA0x62                              // Uncomment to use Motor Shield at address 0x62

#if defined(ADA0x61)
// Configuration: Rocker arm mounted on front bogie
char ssid[] = "Rover_0061";                  // Network SSID
Adafruit_MotorShield AFMS2 = Adafruit_MotorShield(0x61);  // Secondary motor shield at address 0x61

// Right side motors
Adafruit_DCMotor *A0x60M1 = AFMS1.getMotor(1);  // Front Right
Adafruit_DCMotor *A0x60M4 = AFMS1.getMotor(4);  // Middle Right
Adafruit_DCMotor *A0x60M3 = AFMS1.getMotor(3);  // Rear Right

// Left side motors
Adafruit_DCMotor *A0x6nM2 = AFMS2.getMotor(2);  // Front Left
Adafruit_DCMotor *A0x6nM4 = AFMS2.getMotor(4);  // Middle Left
Adafruit_DCMotor *A0x6nM3 = AFMS2.getMotor(3);  // Rear Left

const int scheda = 61;

#elif defined(ADA0x62)
char ssid[] = "Rover_0062";                  // Network SSID
Adafruit_MotorShield AFMS2 = Adafruit_MotorShield(0x62);  // Secondary shield at 0x62

// Right side motors
Adafruit_DCMotor *A0x6nM4 = AFMS2.getMotor(4);  // Front Right
Adafruit_DCMotor *A0x6nM3 = AFMS2.getMotor(3);  // Middle Right
Adafruit_DCMotor *A0x6nM2 = AFMS2.getMotor(2);  // Rear Right

// Left side motors
Adafruit_DCMotor *A0x60M4 = AFMS1.getMotor(4);  // Front Left
Adafruit_DCMotor *A0x60M3 = AFMS1.getMotor(3);  // Middle Left
Adafruit_DCMotor *A0x60M1 = AFMS1.getMotor(1);  // Rear Left

const int scheda = 62;

#else
char ssid[] = "Rover_0064";                  // Default configuration: address 0x64
Adafruit_MotorShield AFMS2 = Adafruit_MotorShield(0x64);  // Secondary shield at 0x64

// Right side motors
Adafruit_DCMotor *A0x6nM4 = AFMS2.getMotor(4);  // Front Right
Adafruit_DCMotor *A0x6nM3 = AFMS2.getMotor(3);  // Middle Right
Adafruit_DCMotor *A0x6nM2 = AFMS2.getMotor(2);  // Rear Right

// Left side motors
Adafruit_DCMotor *A0x60M4 = AFMS1.getMotor(4);  // Front Left
Adafruit_DCMotor *A0x60M3 = AFMS1.getMotor(3);  // Middle Left
Adafruit_DCMotor *A0x60M1 = AFMS1.getMotor(1);  // Rear Left

const int scheda = 64;
#endif








/******************************************************************************** SETUP START *************************************************************************************************************/

void setup() {

  time0 = micros();                     // Record initial timestamp
  String message = "";                  // Placeholder for future messages

  // Initialize serial communication
  Serial.begin(9600);

  // Initialize both Adafruit Motor Shields
  AFMS1.begin();
  AFMS2.begin();

  // Initialize the IMU sensor
  IMU.begin();
  delay(500);                           // Wait for IMU to stabilize
  calibrateIMU(500, 500);               // Perform IMU calibration with 500 samples

  // Read initial accelerometer and gyroscope data
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(accX, accY, accZ);
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
  }

  // Compute initial orientation
  updatePitchRoll();
  updateYaw();

  // Initialize Kalman filter with initial angles
  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);
  kalmanZ.setAngle(yaw);

  // Initialize gyro-only angle estimates
  gyroXangle = roll;
  gyroYangle = pitch;
  gyroZangle = yaw;

  // Initialize complementary filter estimates
  compAngleX = roll;
  compAngleY = pitch;
  compAngleZ = yaw;

  timer = micros();                    // Start timer for loop timing

  // Check if WiFi module is present
  if (WiFi.status() == WL_NO_MODULE) {
    while (true);                      // Halt if no WiFi module
  }

  // Start WiFi Access Point
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    while (true);                      // Halt if AP creation fails
  }

  delay(1000);                         // Wait for AP to initialize

  // Start web server
  server.begin();

  // Initialize SD card
  SD.begin(chipSelect);
}

/******************************************************************************** SETUP END *************************************************************************************************************/


/******************************************************************************** LOOP START *************************************************************************************************************/
void loop() {

  String message = "";
  SD.begin(chipSelect);  // Re-initialize SD card each loop (can be optimized)

  // Monitor WiFi status changes
  if (status != WiFi.status()) {
    status = WiFi.status();
    if (status == WL_AP_CONNECTED) {
      ; // Client connected
    } else {
      ; // Client disconnected
    }
  }

  WiFiClient client = server.available(); // Listen for incoming clients

  if (client) {
    boolean currentLineIsBlank = true;
    String currentLine = "";

    while (client.connected()) {
      delayMicroseconds(10); // Prevents overloading SPI on Nano RP2040 Connect

      if (client.available()) {
        char c = client.read();
        currentLine += c;

        if (c == '\n' && currentLineIsBlank) {

          // Send HTTP response
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println();

          if (currentLine.indexOf("ajax_switch") != -1) {
            // Placeholder for AJAX handler (currently unused)
            ;
          } else {
            // Send HTML interface
            client.println("<html>");
            client.println("<head>");
            client.println("<script>");
            client.println("function GetSwitchState() {");
            client.println("nocache = \"&nocache=\" + Math.random() * 1000000;");
            client.println("var request = new XMLHttpRequest();");
            client.println("request.onreadystatechange = function() {");
            client.println("if (this.readyState == 4) {");
            client.println("if (this.status == 200) {");
            client.println("if (this.responseText != null) {");
            client.println("document.getElementById(\"switch_txt\").innerHTML = this.responseText;");
            client.println("}}}}");
            client.println("request.open(\"GET\", \"ajax_switch\" + nocache, true);");
            client.println("request.send(null);");
            client.println("setTimeout('GetSwitchState()', 500);");
            client.println("}");
            client.println("</script>");
            client.println("<head>");
            client.println("<body onload=\"GetSwitchState()\">");

            // Rover control UI
            message += "<table> ";
            message += "<tr>";
            message += "<td><p><a href=\"/rover?a=1\"><button style=\"width:200;height:200;font-size:120px;\" class=\"button\"> \\ </button></a></p> ";
            message += "<td><p><a href=\"/rover?a=2\"><button style=\"width:200;height:200;font-size:120px;\" class=\"button\"> ^ </button></a></p> ";
            message += "<td><p><a href=\"/rover?a=3\"><button style=\"width:200;height:200;font-size:120px;\" class=\"button\"> / </button></a></p> ";
            message += "<tr>";
            message += "<td><p><a href=\"/rover?a=4\"><button style=\"width:200;height:200;font-size:120px;\" class=\"button\"> < </button></a></p> ";
            message += "<td><p><a href=\"/rover?a=5\"><button style=\"width:200;height:200;font-size:60px;\" class=\"button\">Stop</button></a></p> ";
            message += "<td><p><a href=\"/rover?a=6\"><button style=\"width:200;height:200;font-size:120px;\" class=\"button\"> > </button></a></p> ";
            message += "<tr>";
            message += "<td><p><a href=\"/rover?a=7\"><button style=\"width:200;height:200;font-size:120px;\" class=\"button\"> / </button></a></p> ";
            message += "<td><p><a href=\"/rover?a=8\"><button style=\"width:200;height:200;font-size:120px;\" class=\"button\"> v </button></a></p> ";
            message += "<td><p><a href=\"/rover?a=9\"><button style=\"width:200;height:200;font-size:120px;\" class=\"button\"> \\ </button></a></p> ";
            message += "</table> ";

            client.println(message);

            // Velocity input form
            String strvel = "<form method=get style=\"font-size:35px;\">Velocity:<input type=number style=\"width:180;height:70;font-size:35px;\" min=0 max=255 size=3 name=b value=" + String(vel);
            strvel += ">&nbsp;<input name=H type=submit style=\"width:120;height:70;font-size:35px;\" value=INVIA></form>";
            client.print(strvel);

            // Dynamic value update
            variabile = variabile + 1;
            client.println("<p id=\"switch_txt\">Receiving data: ...</p>");
            client.println("</body>");
            client.println("</html>");

            client.println(); // End of HTTP response
          }

          currentLine = "";
          break;
        }

        if (c == '\n') {
          currentLineIsBlank = true;
        }
        else if (c != '\r') {
          currentLineIsBlank = false;
        }

        // Extract velocity from GET parameter
        if (currentLine.endsWith("&H=INVIA")) {
          int si = currentLine.indexOf("b=");
          int sf = currentLine.indexOf("&H=INVIA");
          String st = currentLine.substring(si + 2, sf);
          vel = atoi(st.c_str());
        }

        // Command handler for rover movement
        if (currentLine.endsWith("GET /rover?a=1")) {
          LevogiroPos(0, 0, 0, scheda);
          delay(lagTime);
          LevogiroPos(vel, scheda == 61 ? 2 : 1, scheda == 61 ? 1 : 2, scheda);
        }
        if (currentLine.endsWith("GET /rover?a=2")) {
          LevogiroPos(0, 0, 0, scheda);
          delay(lagTime);
          LevogiroPos(vel, 1, 1, scheda);
        }
        if (currentLine.endsWith("GET /rover?a=3")) {
          LevogiroPos(0, 0, 0, scheda);
          delay(lagTime);
          LevogiroPos(vel, scheda == 61 ? 1 : 2, scheda == 61 ? 2 : 1, scheda);
        }
        if (currentLine.endsWith("GET /rover?a=4")) {
          LevogiroPos(0, 0, 0, scheda);
          delay(lagTime);
          LevogiroPos(vel, scheda == 61 ? 1000 : 1, scheda == 61 ? 1 : 1000, scheda);
        }
        if (currentLine.endsWith("GET /rover?a=5")) {
          LeftWheelsStop();
          RightWheelsStop();
        }
        if (currentLine.endsWith("GET /rover?a=6")) {
          LevogiroPos(0, 0, 0, scheda);
          delay(lagTime);
          LevogiroPos(vel, scheda == 61 ? 1 : 1000, scheda == 61 ? 1000 : 1, scheda);
        }
        if (currentLine.endsWith("GET /rover?a=7")) {
          LevogiroNeg(0, 0, 0, scheda);
          delay(lagTime);
          LevogiroNeg(vel, scheda == 61 ? 2 : 1, scheda == 61 ? 1 : 2, scheda);
        }
        if (currentLine.endsWith("GET /rover?a=8")) {
          LevogiroNeg(0, 0, 0, scheda);
          delay(lagTime);
          LevogiroNeg(vel, 1, 1, scheda);
        }
        if (currentLine.endsWith("GET /rover?a=9")) {
          LevogiroNeg(0, 0, 0, scheda);
          delay(lagTime);
          LevogiroNeg(vel, scheda == 61 ? 1 : 2, scheda == 61 ? 2 : 1, scheda);
        }
      }
    }

    // Close the connection
    delay(1);
    client.stop();
  }
}

/******************************************************************************** LOOP END *************************************************************************************************************/


/******************************************************************************** FUNCTION START *************************************************************************************************************/

// RGB LED ILLUMINATION (Optional for the Rover)
//void RGBled(int r, int g, int b) {
//  WiFiDrv::analogWrite(25, g);
//  WiFiDrv::analogWrite(26, r);
//  WiFiDrv::analogWrite(27, b);
//}

/******************************************************************************** FUNCTION END *************************************************************************************************************/
