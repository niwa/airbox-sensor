# -*- coding: utf-8 -*-
"""
Routine to parse the data line received from the sensors
Taken from niwa/dusty-acorn
20190702
    Focused ONLY on LittleBo parsing and MQTT publishing
20160705
    Changed the format of the data from the sensor.
    New dust sensor with more data and re-ordered the data channels
"""

import os
import time
from random import randint

import serial  # Serial communications

import paho.mqtt.client as mqtt # MQTT publishing

# Start the MQTT client
client = mqtt.Client()
client.connect("localhost",1883)
# Read the settings from the settings file
# Define test or live mode
with open("./config.txt") as sf:
    mode_line = sf.readline().rstrip('\n')
    serialn = sf.readline().rstrip('\n')
    # e.g. "/dev/ttyAMA0,9600,N,8,n"
    settings_line = sf.readline().rstrip('\n').split(',')
    # port = settings_line[0]
    baud = eval(settings_line[1])
    par = settings_line[2]
    byte = eval(settings_line[3])
    # ceol = settings_line[4]
    # Close the settings file
# Set the initial time for data storage
datapath = "/tmp"
timestamp = None
entry = None
rec_time = time.gmtime()
if mode_line == 'live':
    # If live ... open the serial port
    # Open the serial port and clean the I/O buffer
    ser = serial.Serial()
    ser.port = settings_line[0]
    ser.baudrate = baud
    ser.parity = par
    ser.bytesize = byte
    ser.open()
    ser.flushInput()
    ser.flushOutput()
else:
    # If test ... open and read sample file
    with open("pacman_sample.txt", "r") as file:
        lines = file.read().split('\n')
""" Reads data from pacman """
while True:
    if mode_line == 'live':
        # Get a line of data from PACMAN
        line = ser.readline()
    else:
        end = len(lines) - 1
        start = 0
        idx = randint(start, end)
        line = lines[idx]
    # Get the measurements
    # Data line is:
    # PM1
    # PM2.5
    # PM10
    # TSIPM1
    # TSIPM2.5
    # TSIPM10
    # Data7
    # Data8
    # Data9
    # Distance
    # Temperature
    # RH
    # CO2
    err_value = -99
    if len(line) > 0:
        if line[0].isdigit():
            p_vec = list(map(float, line.split()))
            if len(p_vec) >= 13:
                pm1 = p_vec[0]  # 0
                dust = p_vec[1]  # 1
                pm10 = p_vec[2]  # 2
                if pm10 > 1000:
                    pm10 = 1000
                distance = p_vec[9]  # 3
                t1 = p_vec[10]  # 4
                rh = p_vec[11]  # 5
                co2 = -1 * p_vec[12]  # 6
            else:
                print("Short data line")
                print(p_vec)
                pm1 = err_value  # 0
                dust = err_value  # 1
                pm10 = err_value  # 2
                distance = err_value  # 3
                t1 = err_value  # 4
                rh = err_value  # 5
                co2 = err_value  # 6
        else:
            print("Non numeric first character")
            print(line)
            pm1 = err_value  # 0
            dust = err_value  # 1
            pm10 = err_value  # 2
            distance = err_value  # 3
            t1 = err_value  # 4
            rh = err_value  # 5
            co2 = err_value  # 6
    else:
        print("Line too short")
        print(line)
        pm1 = err_value  # 0
        dust = err_value  # 1
        pm10 = err_value  # 2
        distance = err_value  # 3
        t1 = err_value  # 4
        rh = err_value  # 5
        co2 = err_value  # 6
    # Add "serialn" to the topics to publish
    client.publish("temperature",pm1)
    client.publish("rh",dust)
    client.publish("co2",pm10)
    # C D E F G A B
    # print(co2)
    #         0    1    2   3    4    5     6 
    print("{:<6}{:<6}{:<6}{:<6}{:<6}{:<7}{:<8}".format(*[pm1, dust, pm10, distance, t1, rh, co2]))