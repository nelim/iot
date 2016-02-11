#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <DHT.h>
#include "FS.h"

//#define DEBUG

// MQTT Topics
String t_relay = "/office/relay";
String t_rstate = "/office/relay_state";
String t_temp = "/office/temp";
String t_hum = "/office/hum";

IPAddress server_ip(x, x, x, x);

const int BUTTON = 0;      // Button GPIO
const int RELAY = 5;       // relay GPIO
const int DHTPIN = 12;       // what pin we're connected to
const int PIR = 13;        // PIR datchik PIN
int pirState = LOW;        // we start, assuming no motion detected
int pirVal = 0;           // current pir value


const int DELAY = 50;      // minimal loop delay

#define HOSTNAME "ESP8266-OTA-" // Hostname. The setup function adds the Chip ID at the end.


const int DHTTYPE = DHT22;   // DHT 22  (AM2302)
#define DHTREAD (1000UL * 60 * 1) // one minute
unsigned long rolltime = millis() + DHTREAD;

DHT dht(DHTPIN, DHTTYPE, 28);

const char* otapassword = "xxxx";

WiFiServer server(23);
WiFiClient espClient;
PubSubClient client(espClient, server_ip);

#define MAX_SRV_CLIENTS 1
WiFiClient serverClients[MAX_SRV_CLIENTS];


// Telnet Debug Function
void printDebug(const char* c) {
  Serial.println(c);

  //Prints to telnet if connected
  if (serverClients[0] && serverClients[0].connected()) {
    serverClients[0].println(c);
  }
}

enum BTN_EVENTS
{
  EV_NONE = 0, EV_SHORTPRESS, EV_LONGPRESS
};

volatile unsigned long time_pressed = 0;
volatile unsigned long time_released = 0;
volatile int btnState = EV_NONE;

void checkButtonPress()
{
  btnState = EV_NONE;
  int pinState = digitalRead(BUTTON);
  int btnPressed = !pinState; // pressed is LOW. released is HIGH

  if (btnPressed)
    time_pressed = millis();
  else
  {
    btnState = EV_SHORTPRESS;
    time_released = millis();
    if ((time_released - time_pressed) > 2000)
      btnState = EV_LONGPRESS;
  }
}
void handleButton()
{
  if (btnState > EV_NONE)
  {
    switch (btnState)
    {
      case EV_SHORTPRESS:
        //digitalWrite(RELAY, !digitalRead(RELAY));
        switch (digitalRead(RELAY))
        {
          case HIGH:
            digitalWrite(RELAY, LOW);
            client.publish(t_rstate, "0");
            break;
          case LOW:
            digitalWrite(RELAY, HIGH);
            client.publish(t_rstate, "1");
            break;
        }
        break;
      case EV_LONGPRESS:
        digitalWrite(RELAY, LOW);
        client.publish(t_rstate, "0");
        break;
    }
    time_pressed = 0;
    time_released = 0;
    btnState = EV_NONE;
  }
}

bool loadWifiConfig(String *ssid, String *pass)
{
  File configFile = SPIFFS.open("/wifi.txt", "r");
  if (!configFile)
  {
    Serial.println("Failed to load /wifi.txt");
    return false;
  }
  String content = configFile.readString();
  configFile.close();
  content.trim();

  int pos = content.indexOf("\r\n");
  int le = 2;
  if (pos == -1)
  {
    le = 1;
    pos = content.indexOf("\n");
    if (pos == -1)
      pos = content.indexOf("\r");
  }

  if (pos == -1)
  {
    Serial.println("Infvalid content.");
    Serial.println(content);
    return false;
  }

  *ssid = content.substring(0, pos);
  *pass = content.substring(pos + le);

  ssid->trim();
  pass->trim();
}

void setupWifi() {
  String wifi_ssid = "";
  String wifi_password = "";

  if (!SPIFFS.begin())
  {
    Serial.println("WIFI: Failed to mount SPIFFS.");
    return;
  }

  if (!loadWifiConfig(&wifi_ssid, &wifi_password))
  {
    Serial.println("WIFI: No connection information available.");
    return;
  }

  Serial.println("");
  Serial.print("WIFI: Connecting to: " + (String)wifi_ssid + " ...");

  WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WIFI: Connected. IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(const MQTT::Publish& pub) {
#ifdef DEBUG
  Serial.print(pub.topic());
  Serial.print(" => ");
  Serial.println(pub.payload_string());
#endif

  if ( pub.topic() == t_relay ) {
    if (pub.payload_string() == "1" ) {
      digitalWrite(RELAY, HIGH);   // turn the RELAY on
      client.publish(t_rstate, "1");
      printDebug("mqtt relay on");
    } else if ( pub.payload_string() == "0" ) {
      digitalWrite(RELAY, LOW);    // turn the RELAY off
      client.publish(t_rstate, "0");
      printDebug("mqtt relay off");
    } else {
      Serial.print("I do not know what to do with ");
      Serial.print(pub.payload_string());
      Serial.print(" on topic ");
      Serial.println(pub.topic());
    }
  }
}

void connect_to_MQTT() {
  client.set_callback(callback);

  if (client.connect("esp8266_EVB")) {
    Serial.println("(re)-connected to MQTT");
    client.subscribe(t_relay);
  } else {
    Serial.println("Could not connect to MQTT");
  }
}

void setup() {
  pinMode(RELAY, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(PIR, INPUT);

  attachInterrupt(digitalPinToInterrupt(BUTTON), checkButtonPress, CHANGE);
  dht.begin();

  digitalWrite(RELAY, LOW);   // set to state 0
  Serial.begin(115200);
  setupWifi();
  connect_to_MQTT();

  // Set Hostname.
  String hostname(HOSTNAME);
  hostname += String(ESP.getChipId(), HEX);

  // Start OTA server.
  ArduinoOTA.setHostname((const char *)hostname.c_str());
  ArduinoOTA.setPassword(otapassword);
  ArduinoOTA.begin();

  server.begin();
  server.setNoDelay(true);

}

void loop() {
  handleButton();

  client.loop();

  if (! client.connected()) {
    Serial.println("Not connected to MQTT....");
    connect_to_MQTT();
    delay(5000);
  }

  delay(DELAY);

  uint8_t i;

  if (server.hasClient()) {
    if (!serverClients[0] || !serverClients[0].connected()) {
      if (serverClients[0]) serverClients[0].stop();
      serverClients[0] = server.available();
    } else {
      server.available().stop();
    }
#ifdef DEBUG
    Serial1.print("New client: "); Serial1.print(i);
#endif
    serverClients[0].flush();
  }


  if ((long)(millis() - rolltime) >= 0) {

    rolltime += DHTREAD;

    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();
    // Read temperature as Celsius
    float t = dht.readTemperature();

    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }

    client.publish(t_temp, String(t));
    client.publish(t_hum, String(h));

#ifdef DEBUG
    Serial.println(millis());
    Serial.print("Humidity: ");
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("Temperature: ");
    Serial.print(t);
    Serial.println(" *C ");
#endif
  }
   
  pirVal = digitalRead(PIR); // read input value
  if (pirVal == HIGH) { // check if the input is HIGH
    if (pirState == LOW) {
      // we have just turned on
      Serial.println("Motion detected!");
      // We only want to print on the output change, not state
      pirState = HIGH;
    }
  } else {
    if (pirState == HIGH) {
      // we have just turned of
      Serial.println("Motion ended!");
      // We only want to print on the output change, not state
      pirState = LOW;
    }
  }


  // Handle OTA server.
  ArduinoOTA.handle();
  //yield();

}
