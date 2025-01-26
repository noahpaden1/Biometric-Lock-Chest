#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_Fingerprint.h>
#include <SoftwareSerial.h>
#include <Servo.h>

#define SERVO_CHANNEL 0   // Define the PWM channel for the servo
#define SERVO_MIN  120     // Minimum pulse width 
#define SERVO_MAX  400     // Maximum pulse width

Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver();

// Define the pins for SoftwareSerial
SoftwareSerial mySerial(2, 3);

// Create the fingerprint sensor object
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

// Define RGB LED pins
const int redPin = 9;
const int greenPin = 10;
const int bluePin = 11;

bool isServoInMinPosition = true; // State variable to track the servo's position

void setup() {
  Serial.begin(9600);
  Serial.println("Starting setup...");
  
  driver.begin();
  driver.setPWMFreq(60);  // Set frequency to 60 Hz for servos
  delay(10);     
  
  Serial.println("Servo attached");

  // Wait for Serial Monitor to open
  while (!Serial) {
    delay(10);
  }
  Serial.println("Serial communication established");
  
  // Set up the fingerprint sensor
  finger.begin(57600);
  Serial.println("Fingerprint sensor initialized");
  
  if (finger.verifyPassword()) {
    Serial.println("Found fingerprint sensor!");
  } else {
    Serial.println("Did not find fingerprint sensor :(");
    while (1) { delay(1); }
  }

  // Set up RGB LED pins
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  Serial.println("RGB LED pins configured");

  // Turn on red LED initially
  setColor(255, 0, 0);
  Serial.println("RGB LED set to red");
}

void loop() {
  // Check if a fingerprint is detected
  uint8_t result = getFingerprintID();
  if (result == FINGERPRINT_OK) {
    if (finger.confidence > 60) {
      // Toggle the servo position between min and max
      if (isServoInMinPosition) {
        driver.setPWM(SERVO_CHANNEL, 0, SERVO_MAX);
        isServoInMinPosition = false;
        Serial.println("Servo moved to max position");
      } else {
        driver.setPWM(SERVO_CHANNEL, 0, SERVO_MIN);
        isServoInMinPosition = true;
        Serial.println("Servo moved to min position");
      }

      // Turn on green LED to indicate success
      setColor(0, 255, 0);
      delay(1500); // Delay for green light effect
    } else {
      // Turn on red LED if fingerprint is detected but confidence is not sufficient
      setColor(255, 0, 0);
      Serial.println("Fingerprint detected but confidence < 60");
    }
  } else {
    // Turn on red LED if no fingerprint is detected
    setColor(255, 0, 0);
    Serial.println("No fingerprint detected");
  }
  delay(100); // Short delay to prevent excessive updates
}

uint8_t getFingerprintID() {
  uint8_t p = finger.getImage();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.println("No finger detected");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }
  
  // OK success!
  p = finger.image2Tz();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }
  
  // OK converted!
  p = finger.fingerSearch();
  if (p == FINGERPRINT_OK) {
    Serial.println("Found a print match!");
    Serial.print("Found ID #"); Serial.print(finger.fingerID); 
    Serial.print(" with confidence of "); Serial.println(finger.confidence);
    return FINGERPRINT_OK;
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
  } else if (p == FINGERPRINT_NOTFOUND) {
    Serial.println("Did anot find a match");
  } else {
    Serial.println("Unknown error");
  }
  return p;
}

// Function to set the color of the RGB LED
void setColor(int red, int green, int blue) {
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
  Serial.print("RGB set to: R=");
  Serial.print(red);
  Serial.print(", G=");
  Serial.print(green);
  Serial.print(", B=");
  Serial.println(blue);
}
