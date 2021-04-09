#!/bin/bash

rm -f /var/run/pigpio.pid
sudo pigpiod

exec $@
