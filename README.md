# Aquarium Controller

This is a next version of an aquarium controller and monitor.  The previous version is [here](https://github.com/clarsen/aquarium-heater)

It controls a heater via an MQTT (Tasmota) WiFi smart plug, measures temperature with a DS18B20 waterproof probe, uses a PID algorithm to control the set temperature, logs current temperature to InfluxDB and activity status to MQTT.

# setup
include/config.h.sample needs to be copied to include/config.h and filled with values that suite your environment (wifi + mqtt + influxdb).

# BOM
Total $36 of parts see [BOM](aquarium-controller-simple-pricing.csv)