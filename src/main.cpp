#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include <InfluxDbClient.h>

#include "config.h"

OneWire ds(15); //data wire connected to GPIO15
DeviceAddress aquarium = AQUARIUM_DS18B20_ADDR;
DallasTemperature sensors(&ds);

double input = 80, output = 50, setpoint = 79;
double kp = 67905.48, ki = 1437.2, kd = 0; // from autotune

// slow PWM
double WindowSize = 60000;
unsigned long windowStartTime;
bool relayOn = false;
bool firstTime = true;

double aTuneStep = WindowSize / 2, aTuneNoise = 0.2, aTuneStartValue = WindowSize / 2;
unsigned int aTuneLookBack = 20;
byte ATuneModeRemember = 2;

PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
PID_ATune aTune(&input, &output);
boolean tuning = false;
unsigned long serialTime;

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;
const char *mqtt_server = MQTT_SERVER;
const char *mqtt_user = MQTT_USER;
const char *mqtt_password = MQTT_PASSWORD;

unsigned long mqttReportingTime;

WiFiClient espClient;
PubSubClient client(espClient);

void SerialReceive();
void SerialSend();
void changeAutoTune();
void AutoTuneHelper(boolean start);
void setup_wifi();
void reconnect();

InfluxDBClient influxClient;
unsigned long influxReportingTime;

unsigned long sampleTimeLoopMs = 2000;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  setup_wifi();
  configTzTime(LOCAL_TZ, "pool.ntp.org", "time.nis.gov");

  client.setServer(mqtt_server, 1883);
  reconnect();

  influxClient.setConnectionParamsV1(INFLUX_URL, INFLUX_DB, INFLUX_USER, INFLUX_PASSWORD);

  if (influxClient.validateConnection())
  {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(influxClient.getServerUrl());
  }
  else
  {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(influxClient.getLastErrorMessage());
  }


  sensors.begin();
  sensors.setResolution(12);

  //Setup the pid
  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetMode(AUTOMATIC);
  mqttReportingTime = influxReportingTime = serialTime = windowStartTime = millis();
  myPID.SetSampleTime(sampleTimeLoopMs);

  if (tuning)
  {
    tuning = false;
    changeAutoTune();
    tuning = true;
  }
}

void loop()
{
  // put your main code here, to run repeatedly:
  // byte i;
  // byte addr[8];
  unsigned long start = millis();
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();


  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.print("Aquarium 1(*F): ");
  Serial.println(sensors.getTempF(aquarium));
  input = sensors.getTempF(aquarium);

  if (tuning)
  {
    byte val = (aTune.Runtime());
    if (val != 0)
    {
      tuning = false;
    }
    if (!tuning)
    { //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp, ki, kd);
      AutoTuneHelper(false);
    }
  }
  else
  {
    myPID.Compute();
  }

  unsigned long now = millis();
  if (now - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
    Serial.println("NEW PWM WINDOW");
  }

  if (output > now - windowStartTime)
  {
    if (!relayOn || firstTime)
    {
      Serial.println("on");
      relayOn = true;
      client.publish("cmnd/aquariumheater/POWER", "on");
      firstTime = false;
    }
  }
  else
  {
    if (relayOn || firstTime)
    {
      Serial.println("off");
      relayOn = false;
      client.publish("cmnd/aquariumheater/POWER", "off");
      firstTime = false;
    }
  }

  if (millis() > serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime += 500;
  }
  if (millis() > influxReportingTime)
  {
    Serial.println("logging to influx");
    Point measurement("aquarium");
    measurement.addField("temp_f", input);
    influxClient.writePoint(measurement);
    influxReportingTime += 15000;
  }

  if (millis() > mqttReportingTime)
  {
    char msg[1024];
    snprintf(msg, sizeof(msg), "setpoint: %0.2f input: %0.2f output: %0.2f kp: %0.2f ki: %0.2f kd: %0.2f",
             setpoint, input, output, myPID.GetKp(), myPID.GetKi(), myPID.GetKd());
    client.publish("aquariumcontrol/status", msg);
    mqttReportingTime += 10000;
  }

  unsigned long elapsed = millis() - start;
  if (elapsed < sampleTimeLoopMs)
  {
    Serial.print("delay ");
    Serial.println(sampleTimeLoopMs - elapsed);
    delay(sampleTimeLoopMs - elapsed);
  }
}

void SerialSend()
{
  Serial.print("setpoint: ");
  Serial.print(setpoint);
  Serial.print(" ");
  Serial.print("input: ");
  Serial.print(input);
  Serial.print(" ");
  Serial.print("output: ");
  Serial.print(output);
  Serial.print(" ");
  if (tuning)
  {
    Serial.println("tuning mode");
  }
  else
  {
    Serial.print("kp: ");
    Serial.print(myPID.GetKp());
    Serial.print(" ");
    Serial.print("ki: ");
    Serial.print(myPID.GetKi());
    Serial.print(" ");
    Serial.print("kd: ");
    Serial.print(myPID.GetKd());
    Serial.println();
  }
}

void SerialReceive()
{
  if (Serial.available())
  {
    char b = Serial.read();
    Serial.flush();
    if ((b == 'a' && !tuning) || (b != 'a' && tuning))
    {
      changeAutoTune();
    }
  }
}

void AutoTuneHelper(boolean start)
{
  if (start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}

void changeAutoTune()
{
  if (!tuning)
  {
    //Set the output to the desired starting frequency.
    output = aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void setup_wifi()
{
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("aquarium_controller", mqtt_user, mqtt_password))
    {
      Serial.println("connected");
      // Subscribe
      // client.subscribe("esp32/output");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
