#include <Arduino.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <Adafruit_Sensor.h>
#include "DHTesp.h"
#include <MQUnifiedsensor.h>
#include <Firebase_ESP_Client.h>

// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "SALLE_MACHINE_SP"
#define WIFI_PASSWORD "083EDk@BbG"

// Insert Firebase project API Key
#define API_KEY "AIzaSyD_KJEpsO_Y8nBe6nB6m1c5FAGDc7hfR9g"
// Insert Authorized Email and Corresponding Password
#define USER_EMAIL "yayehaby2000@gmail.com"
#define USER_PASSWORD "abiba2000"

// Insert RTDB URLefine the RTDB URL
#define DATABASE_URL "https://p1iottechima-default-rtdb.europe-west1.firebasedatabase.app/"
// Define Firebase objects
FirebaseData stream;
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
// Variable to save USER UID
String uid;
String macAdress;

// Variables to save database paths
String databasePath;
String tempPath;
String humPath;
String smokePath;
String lightPath;
#define LIGHT_SENSOR_PIN 36 // ESP32 pin GIOP36 (ADC0)
int analogValue = analogRead(LIGHT_SENSOR_PIN);

/************************Hardware Related Macros************************************/
#define         Board                   ("ESP-32") // Wemos ESP-32 or other board, whatever have ESP32 core.
#define         Pin                     (32)  //IO25 for your ESP32 WeMos Board, pinout here: https://i.pinimg.com/originals/66/9a/61/669a618d9435c702f4b67e12c40a11b8.jpg
/***********************Software Related Macros************************************/
#define         Type                    ("MQ-2") //MQ2 or other MQ Sensor, if change this verify your a and b values.
#define         Voltage_Resolution      (3.3) // 3V3 <- IMPORTANT. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define         ADC_Bit_Resolution      (12) // ESP-32 bit resolution. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define         RatioMQ2CleanAir        (60) // Ratio of your sensor, for this example an MQ-3
/*****************************Globals***********************************************/

// DHT sensor
DHTesp dht;
float temperature;
float humidity;
MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);
float smoke;

float light;

// Timer variables (send new readings every three minutes)
unsigned long sendDataPrevMillis = 0;
unsigned long timerDelay = 180000;
// Initialize WiFi
void initWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  Serial.println();
  macAdress = WiFi.macAddress();
  Serial.println(WiFi.macAddress());
  Serial.println();
}
// Initialize DHT
void initDHT(){
  dht.setup(15, DHTesp::DHT11); // Connect DHT sensor to GPIO 17
  delay(dht.getMinimumSamplingPeriod());

  float humidity = dht.getHumidity();
  float temperature = dht.getTemperature();

  Serial.print(dht.getStatusString());
  Serial.print("\t");
  Serial.print(humidity, 1);
  Serial.print("\t\t");
  Serial.print(temperature, 1);
  Serial.print("\t\t");
  Serial.print(dht.toFahrenheit(temperature), 1);
  Serial.print("\t\t");
  Serial.print(dht.computeHeatIndex(temperature, humidity, false), 1);
  Serial.print("\t\t");
  Serial.println(dht.computeHeatIndex(dht.toFahrenheit(temperature), humidity, true), 1);
  delay(2000);
}
// Initialize MQ2
void initMQ(){
  MQ2.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ2.setA(521853); MQ2.setB(-3.821); // Configure the equation to to calculate Benzene concentration
  /*
    Exponential regression:
  Gas    | a      | b
  LPG    | 44771  | -3.245
  CH4    | 2*10^31| 19.01
  CO     | 521853 | -3.821
  Alcohol| 0.3934 | -1.504
  Benzene| 4.8387 | -2.68
  Hexane | 7585.3 | -2.849
  */
  MQ2.init();
  // Acknowledgements: https://jayconsystems.com/blog/understanding-a-gas-sensor
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ2.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
    Serial.print(".");
  }
  MQ2.setR0(calcR0/10);
  Serial.println("  done!.");
  
  //if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  //if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}
  /*****************************  MQ CAlibration ********************************************/ 
  MQ2.serialDebug(true);

  MQ2.update(); // Update data, the arduino will read the voltage from the analog pin
  MQ2.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  MQ2.serialDebug(); // Will print the table on the serial port
  delay(500); //Sampling frequency

}
// Write float values to the database
void sendFloat(String path, float value){
  if (Firebase.RTDB.setFloat(&fbdo, path.c_str(), value)){
    Serial.print("Writing value: ");
    Serial.print (value);
    Serial.print(" on the following path: ");
    Serial.println(path);
    Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());
    Serial.println("TYPE: " + fbdo.dataType());
    Serial.println("valid");
  }
  else {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
    
  }
}
void streamCallback(FirebaseStream data){
  Serial.printf("stream path, %s\nevent path, %s\ndata type, %s\nevent type, %s\n\n",
                data.streamPath().c_str(),
                data.dataPath().c_str(),
                data.dataType().c_str(),
                data.eventType().c_str());
  printResult(data); //see addons/RTDBHelper.h
  Serial.println();

  //This is the size of stream payload received (current and max value)
  //Max payload size is the payload size under the stream path since the stream connected
  //and read once and will not update until stream reconnection takes place.
  //This max value will be zero as no payload received in case of ESP8266 which
  //BearSSL reserved Rx buffer size is less than the actual stream payload.
  Serial.printf("Received stream payload size: %d (Max. %d)\n\n", data.payloadLength(), data.maxPayloadLength());
}
void streamTimeoutCallback(bool timeout){
  if (timeout)
    Serial.println("stream timeout, resuming...\n");
  if (!stream.httpConnected())
    Serial.printf("error code: %d, reason: %s\n\n", stream.httpCode(), stream.errorReason().c_str());
}
void setup(){
  Serial.begin(115200);
  // Initialize WiFi
  initWiFi();
  // Initialize DHT
  initDHT();
  // Initialize MQ2
  initMQ();
  // Initialize LIGHT

  // Assign the api key (required)
  config.api_key = API_KEY;
  // Assign the user sign in credentials
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  // Assign the RTDB URL (required)
  config.database_url = DATABASE_URL;

  Firebase.reconnectWiFi(true);
  fbdo.setResponseSize(4096);

  // Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  // Assign the maximum retry of token generation
  config.max_token_generation_retry = 5;
  // Initialize the library with the Firebase authen and config
  Firebase.begin(&config, &auth);
  // Getting the user UID might take a few seconds
  Serial.println("Getting User UID");
  while ((auth.token.uid) == "") {
    Serial.print('.');
    delay(1000);
  }
  // Print user UID
  uid = auth.token.uid.c_str();
  Serial.print("User UID: ");
  Serial.print(uid);

  // Update database path
  databasePath = "/UsersData/" + uid;
  Serial.println();
  delay(100);

  // Define database path for sensor readings
  //tempPath = databasePath + "/80:7D:3A:B9:1E:04/DHT/temperature"; // --> UsersData/<user_uid>/<macAdress>/DHT/temperature
  //humPath = databasePath + "/80:7D:3A:B9:1E:04/DHT/humidity"; 
  //smokePath = databasePath + "/E8:68:E7:24:A9:88/MQ2/smoke"; 
  lightPath = databasePath + "/A4:CF:12:0E:86:E0/PRESIS/light"; 

  // Assign a calback function to run when it detects changes on the database
  Firebase.RTDB.setStreamCallback(&stream, streamCallback, streamTimeoutCallback);

  delay(2000);
}

void loop(){
  // the raw analog reading
  // Send new readings to database
  if (Firebase.ready() && (millis() - sendDataPrevMillis > timerDelay || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();

    // Get latest sensor readings
    humidity = dht.getHumidity();
    temperature = dht.getTemperature();
    //smoke = MQ2.readSensor();
    light = analogRead(LIGHT_SENSOR_PIN);

    // Send readings to database:
    //sendFloat(tempPath, temperature);
    //sendFloat(humPath, humidity);
    //sendFloat(smokePath, smoke);
    sendFloat(lightPath, light);
  }
}