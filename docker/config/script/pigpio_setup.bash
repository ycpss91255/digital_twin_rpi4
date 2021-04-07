#!/bin/bash

unzip ./pigpio/main_library.zip -d /home/${USER}/
make -C /home/${USER}/pigpio-master
make install -C /home/${USER}/pigpio-master
tar xvf ./pigpio/piscope.tar -C /home/${USER}/

