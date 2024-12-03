
#define BLYNK_TEMPLATE_ID "TMPL3K2rpdTiN"
#define BLYNK_TEMPLATE_NAME "accident prevention system using iot"
#define BLYNK_AUTH_TOKEN "ZDHM45zQmDKRETDb-jCu8Zjq_kAuYKZO"

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial


#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <BlynkSimpleEsp32.h>

// Replace with your WiFi credentials
const char *ssid = "KSP";
const char *password = "9550421866";

// Server endpoint
const char *serverURL = "https://www.circuitdigest.cloud/send_sms?ID=101";

// Pin definitions
#define MQ3_PIN 34
#define TRIG_PIN 5
#define ECHO_PIN 18
#define RELAY_PIN 32
#define BUZZER_PIN 27

// Objects and variables
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

TinyGPSPlus gps;
HardwareSerial gpsSerial(1); // Use Serial1 for GPS
long duration;
float distance;
int mq3Value;


BlynkTimer timer;

void sendAccidentAlert(float latitude, float longitude)
{
  // Prepare Google Maps link
  String googleMapsLink = "https://maps.google.com/?q=" + String(latitude, 6) + "," + String(longitude, 6);

  // Send SMS using CircuitDigest API
  const char *smsApiKey = "1ggahLd6t9rw"; // Replace with your API key
  const char *smsUrl = "https://www.circuitdigest.cloud/send_sms?ID=101";
  String mobileNumber = "919550421866"; // Replace with recipient's mobile number
  String var1 = "Accident Detected";    // First dynamic value
  String var2 = googleMapsLink;         // Include Google Maps link in the second dynamic value

  HTTPClient smsHttp;

  // Prepare the JSON payload for the SMS
  String smsPostData = String("{\"mobiles\":\"") + mobileNumber +
                       "\",\"var1\":\"" + var1 + "\",\"var2\":\"" + var2 + "\"}";

  // Start HTTP request
  smsHttp.begin(smsUrl);
  smsHttp.addHeader("Authorization", smsApiKey);
  smsHttp.addHeader("Content-Type", "application/json");
  smsHttp.setTimeout(5000); // Set timeout to 5 seconds

  // Send the POST request
  int smsResponseCode = smsHttp.POST(smsPostData);
  String smsResponse = smsHttp.getString(); // Get the response body

  // Handle the response
  if (smsResponseCode > 0)
  {
    Serial.println("SMS Sent Successfully: " + String(smsResponseCode));
    Serial.println("Response: " + smsResponse);
    Serial.println("Test Message Sent: Accident Detected! Check Location: " + googleMapsLink);
  }
  else
  {
    Serial.println("Error Sending SMS: " + String(smsResponseCode));
    Serial.println("Response: " + smsResponse);
  }

  // Close the HTTP request
  smsHttp.end();
}

void setup()
{
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // GPS TX to ESP32 RX (16), RX to ESP32 TX (17)

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  // Initialize ADXL345
  if (!adxl.begin())
  {
    lcd.setCursor(0, 1);
    lcd.print("ADXL Error");
    Serial.println("Error: Failed to initialize ADXL345");
    // while (1);
  }

  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize motor relay
  pinMode(RELAY_PIN, OUTPUT);

  // Initialize buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);

  lcd.clear();
  lcd.print("Setup Done!");
  Serial.println("Setup Complete!");
  delay(2000);
}



void uploadData()
{

  Blynk.virtualWrite(V0, mq3Value);
  Blynk.virtualWrite(V1, distance);
  // Changed to V5
}

void loop()
{

  Blynk.run();
      timer.run();

  // Read GPS data
  while (gpsSerial.available() > 0)
  {
    gps.encode(gpsSerial.read());
  }

  // Display GPS data on LCD
  lcd.clear();
  if (gps.location.isUpdated())
  {
    lcd.setCursor(0, 0);
    lcd.print("Lat: ");
    lcd.print(gps.location.lat(), 6);
    lcd.setCursor(0, 1);
    lcd.print("Lon: ");
    lcd.print(gps.location.lng(), 6);
    Serial.print("GPS Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" | Lon: ");
    Serial.println(gps.location.lng(), 6);
  }
  else
  {
    lcd.setCursor(0, 0);
    lcd.print("GPS Waiting...");
    Serial.println("GPS Waiting...");
  }

  delay(2000);

  // Read MQ-3 alcohol sensor
  mq3Value = analogRead(MQ3_PIN);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MQ-3: ");
  lcd.print(mq3Value);
  Serial.println("MQ-3 Value: " + String(mq3Value));

  // Disable motor and sound buzzer if alcohol detected
  if (mq3Value > 2000)
  {                                // Example threshold
    digitalWrite(RELAY_PIN, LOW);  // Motor OFF
    digitalWrite(BUZZER_PIN, LOW); // Buzzer ON
    lcd.setCursor(0, 1);
    lcd.print("Motor Disabled");
    Serial.println("Motor Disabled: Alcohol Detected");
  }
  else
  {
    digitalWrite(RELAY_PIN, HIGH);  // Motor ON
    digitalWrite(BUZZER_PIN, HIGH); // Buzzer OFF
    lcd.setCursor(0, 1);
    lcd.print("Motor Enabled");
    Serial.println("Motor Enabled: Alcohol Not Detected");
  }

  delay(2000);

  // Ultrasonic distance measurement
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Distance: ");
  lcd.print(distance);
  lcd.print(" cm");
  Serial.println("Distance: " + String(distance) + " cm");

  // Stop motor and sound buzzer if obstacle is close
  if (distance < 10)
  {                                // Example threshold
    digitalWrite(RELAY_PIN, LOW);  // Motor OFF
    digitalWrite(BUZZER_PIN, LOW); // Buzzer ON
    lcd.setCursor(0, 1);
    lcd.print("Obstacle!");
    Serial.println("Obstacle Detected!");
    Blynk.logEvent("alert");
    if (gps.location.isValid())
    {
      sendAccidentAlert(gps.location.lat(), gps.location.lng());
    }
  }
  else
  {
    digitalWrite(RELAY_PIN, HIGH);  // Motor ON
    digitalWrite(BUZZER_PIN, HIGH); // Buzzer OFF
    lcd.setCursor(0, 1);
    lcd.print("Clear Path");
    Serial.println("Path Clear");
  }

  delay(2000);

  // Read ADXL345 accelerometer
  sensors_event_t event;
  adxl.getEvent(&event);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("X:");
  lcd.print(event.acceleration.x);
  lcd.print(" Y:");
  lcd.print(event.acceleration.y);
  lcd.setCursor(0, 1);
  lcd.print("Z:");
  lcd.print(event.acceleration.z);

  Serial.print("ADXL X: ");
  Serial.print(event.acceleration.x);
  Serial.print(" | Y: ");
  Serial.print(event.acceleration.y);
  Serial.print(" | Z: ");
  Serial.println(event.acceleration.z);

  // Detect sudden movement (e.g., accident) and activate buzzer
  if (event.acceleration.x > 10 || event.acceleration.y > 10 || event.acceleration.z > 10)
  {                                // Example threshold
    // digitalWrite(BUZZER_PIN, LOW); // Buzzer ON
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Accident!");
    Serial.println("Accident Detected!");
    Blynk.logEvent("alert");

    if (gps.location.isValid())
    {
      sendAccidentAlert(gps.location.lat(), gps.location.lng());
    }
  }
  else
  {
    // digitalWrite(BUZZER_PIN, HIGH); // Buzzer OFF
    Serial.println("No Accident Detected");
  }

  delay(2000);
}
