/*
   Description: Heartbeat Pulse Sensor KY039 monitoring with ESP32 and IoT
    Author: Ganesh Gaikwad, IoT Developer
*/

#include <WiFi.h>
#include <PubSubClient.h>

//TO DO: ESP32 MQTT user config
const char* SSID = ".................."; // Wifi SSID
const char* PASSWORD = ".................."; // Wifi PASSWORD
const char* USERNAME = "................."; //  login USERNAME
const char* PUB_TOPIC = "publish/..../....."; // publish/USERNAME/apiKeyIn

const unsigned int writeInterval = 2000;   // write interval (in ms)
//AskSensors MQTT config
const char* mqtt_server = "..............."; //mqtt broker server
unsigned int mqtt_port = 1883;

// KY039 defines
#define TAB_LENGTH        4
#define RISE_THRESHOLD  5
#define CALIB_OFFSET  0
#define SENSOR_PIN A0

WiFiClient askClient;
PubSubClient client(askClient);

void setup() {
  Serial.begin(115200);
  Serial.println("*****************************************************");
  Serial.println("Program Start : ESP32 publishes KY039 Heartbeat data to cloud over MQTT");
  Serial.print(" connecting to WIFI : ");
  Serial.println(SSID);

  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("->WiFi connected");
  Serial.println("->IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

}

void loop() {

  // local variables declaration
  float analog_Tab[TAB_LENGTH], analog_sum;
  float KY039_data, last_heartbeat, before_heartbeat, analog_average;
  float beat_first, beat_second, beat_third ;
  bool rising;
  int rise_count, n_reads;
  long int last_beat, timer_now, timer_start, ptr;

  // WiFi connection
  if (!client.connected())
    reconnect();
  client.loop();
  // Init variables
  for (int i = 0; i < TAB_LENGTH; i++)  analog_Tab[i] = 0;
  analog_sum = 0, ptr = 0;

  while (1) {
    // calculate an average of the sensor during a 20 ms period to eliminate the 50 Hz noise caused by electric light
    n_reads = 0;
    analog_average = 0.;
    timer_start = millis();
    do {
      analog_average += analogRead(SENSOR_PIN);
      n_reads++;
      timer_now = millis();
    } while (timer_now < timer_start + 20);
    analog_average /= n_reads;  //  average

    // Add the newest measurement to FIFO array (subtract the oldest measurement)
    analog_sum -= analog_Tab[ptr];
    analog_sum += analog_average;
    analog_Tab[ptr] = analog_average;
    last_heartbeat = analog_sum / TAB_LENGTH;
    // check for a rising curve (= a heart beat)
    if (last_heartbeat > before_heartbeat) {
      rise_count++;
      if (!rising && rise_count > RISE_THRESHOLD) {
        // we have detected a rising curve, which implies a heartbeat.
        rising = true;         // The rising flag prevents us from detecting the same rise more than once.
        beat_first = millis() - last_beat;
        last_beat = millis();
        // Calculate the weighed average of heartbeat rate according to the three last beats
        // Filter equation reference: https://create.arduino.cc/projecthub/Johan_Ha/from-ky-039-to-heart-rate-0abfca
        KY039_data = 60000. / (0.4 * beat_first + 0.3 * beat_second + 0.3 * beat_third) + CALIB_OFFSET;
        Serial.print(KY039_data);
        Serial.println(" BPM\n"); // Unit
        // Record the time since last beat, keep track of the two previous times (beat_first, beat_second, beat_third) to get a weighed average.
        beat_third = beat_second;
        beat_second = beat_first;
        Serial.println("********** Publish MQTT data to Cloud");
        char mqtt_payload[30] = "";
        snprintf (mqtt_payload, 30, "m1=%f", KY039_data);
        Serial.print("Publish message: ");
        Serial.println(mqtt_payload);
        client.publish(PUB_TOPIC, mqtt_payload);
        Serial.println("> MQTT data published");
        Serial.println("**** End ");
        Serial.println("*****************************************************");
        delay(writeInterval);// delay
      }
    }
    else
    {
      // Ok, the curve is falling
      rising = false;
      rise_count = 0;
    }
    before_heartbeat = last_heartbeat;
    ptr++;
    ptr %= TAB_LENGTH;
  }
}

// MQTT callback
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("********** Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client", USERNAME, "")) {
      Serial.println("-> MQTT client connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("-> try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
