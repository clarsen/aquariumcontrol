# Aquarium Controller

## Intro

This is a next version of an aquarium controller and monitor.  The previous version is [here](https://github.com/clarsen/aquarium-heater)

It controls a heater via an MQTT (Tasmota) WiFi smart plug, measures temperature with a DS18B20 waterproof probe, uses a PID algorithm to control the set temperature, logs current temperature to InfluxDB and activity status to MQTT.

## Setup

include/config.h.sample needs to be copied to include/config.h and filled with values for your environment (WiFi, MQTT, InfluxDB, timezone).

## BOM

Total $36 of parts see [BOM](aquarium-controller-simple-pricing.csv)

The MQTT broker runs on a Raspberry Pi used for other purposes.  That isn't included in the cost.  Any heater which will always be on when powered up from the wall socket will suffice.  I used a 150W Fluval M heater for $35.
