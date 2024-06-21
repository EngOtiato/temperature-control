#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

@author: Kevin Otiato

"""

import csv
import serial
import struct
import ctypes
import time
from picosdk.usbtc08 import usbtc08 as tc08
from picosdk.functions import assert_pico2000_ok

# Setting Constants
temp_input = float(input("Please enter the set point temperature value: "))
desired_temperature = temp_input

# TC-08 Data Loggers
chandle1 = ctypes.c_int16()
chandle2 = ctypes.c_int16()
status = {}

# Open units
status["open_unit1"] = tc08.usb_tc08_open_unit()
status["open_unit2"] = tc08.usb_tc08_open_unit()
assert_pico2000_ok(status["open_unit1"])
assert_pico2000_ok(status["open_unit2"])
chandle1 = status["open_unit1"]
chandle2 = status["open_unit2"]

# Set mains rejection to 50 Hz
status["set_mains1"] = tc08.usb_tc08_set_mains(chandle1, 0)
status["set_mains2"] = tc08.usb_tc08_set_mains(chandle2, 0)
assert_pico2000_ok(status["set_mains1"])
assert_pico2000_ok(status["set_mains2"])

# Set up channels for logger 1
channels1 = [1, 2, 3, 4, 5, 6, 7, 8]
typeK1 = ctypes.c_int8(75)
for channel in channels1:
    status["set_channel{}_1".format(channel)] = tc08.usb_tc08_set_channel(chandle1, channel, typeK1)
    assert_pico2000_ok(status["set_channel{}_1".format(channel)])

# Set up channels for logger 2
channels2 = [1, 2, 3, 4, 5, 6, 7, 8]
typeK2 = ctypes.c_int8(75)
for channel in channels2:
    status["set_channel{}_2".format(channel)] = tc08.usb_tc08_set_channel(chandle2, channel, typeK2)
    assert_pico2000_ok(status["set_channel{}_2".format(channel)])

# Get minimum sampling interval in ms for logger 1
status["get_minimum_interval_ms1"] = tc08.usb_tc08_get_minimum_interval_ms(chandle1)
assert_pico2000_ok(status["get_minimum_interval_ms1"])

# Get minimum sampling interval in ms for logger 2
status["get_minimum_interval_ms2"] = tc08.usb_tc08_get_minimum_interval_ms(chandle2)
assert_pico2000_ok(status["get_minimum_interval_ms2"])

# Functions to read temperature readings for each logger
def get_single_temperature_reading_1():
    temp = (ctypes.c_float * 9)()
    overflow = ctypes.c_int16(0)
    units = tc08.USBTC08_UNITS["USBTC08_UNITS_CENTIGRADE"]

    status = tc08.usb_tc08_get_single(chandle1, ctypes.byref(temp), ctypes.byref(overflow), units)
    assert_pico2000_ok(status)

    return list(temp)

def get_single_temperature_reading_2():
    temp = (ctypes.c_float * 9)()
    overflow = ctypes.c_int16(0)
    units = tc08.USBTC08_UNITS["USBTC08_UNITS_CENTIGRADE"]

    status = tc08.usb_tc08_get_single(chandle2, ctypes.byref(temp), ctypes.byref(overflow), units)
    assert_pico2000_ok(status)

    return list(temp)

# EA-PS 2342-10B
def send_telegram(sd, dn, obj, d1=None, d2=None):
    telegram = bytearray()
    telegram.append(sd)  # start delimiter
    telegram.append(dn)  # device number 0 or 1
    telegram.append(obj)  # object number. 54 == power supply control
    if d1 is not None:
        telegram.append(d1)
    if d2 is not None:
        telegram.append(d2)
    chk = sum(telegram)  # Calculate checksum
    telegram.append((chk >> 8) & 0xff)
    telegram.append(chk & 0xff)
    ser.write(telegram)


def set_output_on(output):
    send_telegram(0xf1, output, 54, 1, 1)
    reply = ser.read(6)


def set_output_off(output):
    send_telegram(0xf1, output, 54, 1, 0)
    reply = ser.read(6)


def set_remote_control(output):
    send_telegram(0xf1, output, 54, 0x10, 0x10)
    reply = ser.read(6)


def set_manual_control(output):
    send_telegram(0xf1, output, 54, 0x10, 0x00)
    reply = ser.read(6)


def set_output_voltage(output, voltage):
    p = int(voltage / 42.0 * 25600)
    send_telegram(0xf1, output, 50, (p >> 8) & 0xff, p & 0xff)
    reply = ser.read(6)


def set_output_current(output, current):
    p = int(current / 10.0 * 25600)
    send_telegram(0xf1, output, 51, (p >> 8) & 0xff, p & 0xff)
    reply = ser.read(6)


def get_voltage_current(output):
    send_telegram(0x75, output, 71)
    reply = ser.read(11)
    v = 0.0
    i = 0.0
    if len(reply) == 11:
        chk = sum(reply[:9])
        a = struct.unpack('>H', reply[9:])
        if chk == a[0]:
            a = struct.unpack('>HH', reply[5:9])
            v = round(float(a[0]) / 25600.0 * 42.0, 2)
            i = round(float(a[1]) / 25600.0 * 10.0, 2)
        else:
            print('Checksum error')
    return v, i

try:
    ser = serial.Serial('/dev/tty.usbmodem28140400071', 115200, parity=serial.PARITY_ODD, timeout=0.5)
except Exception as e:
    print('Error opening serial port:', str(e))
    exit()

# Set up the PI controller
Kp = 122.678  # Set the proportional gain
Ki = 0.01122  # Set the integral gain
integral = 0  # Initialize the integral term
last_error = 0  # Initialize the last error term
last_time = time.time()

# CSV Constants
time_interval = 10  # Set the time interval for temperature recording in seconds
Time = 0  # Initialize time to zero

fieldnames = ["Time", "Set_Point_Temp.", "Average_Internal_Temp.", "T1_cal", "T2_cal", "T3_cal", "T4_cal", "T5_cal", "T6_cal", "Ambient_temp","Hot_Plate_Temp.", "Power_Input"]

with open("mass_id_2.csv", "w", newline="") as my_csv_file:
    csv_writer = csv.DictWriter(my_csv_file, fieldnames=fieldnames)
    csv_writer.writeheader()

# Loop to control the Power Supply and Heater using the PI controller
while Time <= 5400:
    temp = get_single_temperature_reading_1()
    temps = get_single_temperature_reading_2()
    # Read the current temperature from a sensor at channel 4
    average_internal_temp = ((((temps[1] - 0.3515) / 1.0008) + (temps[2] - 0.4716) + (
            (temps[3] - 0.7512) / 0.9990)) / 3)
    T1_cal = (temp[1] - 0.1514) / 0.9999
    T2_cal = (temp[2] - 0.02479) / 0.9995
    T3_cal = (temp[3] - 0.03283) / 0.9971
    T4_cal = (temp[4] + 1.3483) / 1.001
    T5_cal = (temp[5] - 0.1514) / 0.9999
    T6_cal = (temp[6] + 0.04134) / 1.001
    Ambient_temp=temp[7]
    hot_plate = ((temp[8] - 0.4779) / 0.9993)

    # Calculate the error (difference between the current temperature and the setpoint)
    error = desired_temperature -  average_internal_temp

    # Calculate the time elapsed since the last integration
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time

    # Calculate the proportional term
    proportional = Kp * error

    # Calculate the integral term
    integral = integral + Ki * error * dt

    # Limit the integral term to prevent windup
    if integral > 100:
        integral = 100
    elif integral < 0:
        integral = 0

    # Calculate the output of the PI controller (power level in %)
    output = proportional + integral

    # Set the power level to the heater using the power supply
    # Channel 1 voltage control
    v_1 = min(max((output / 100.0) * 22.5, 0), 22.5)
    set_remote_control(0)
    set_output_voltage(0, v_1)
    set_output_on(0)

    # Channel 2 voltage control
    v_2 = min(max((output / 100.0) * 15.0, 0), 15.0)

    set_remote_control(1)
    set_output_voltage(1, v_2)
    set_output_on(1)
    
    # Deriving voltage and current for both channels and calculating power
    voltage_1, current_1 = get_voltage_current(0)
    voltage_2, current_2 = get_voltage_current(1)
    
    Total_Power=((voltage_1 * current_1)+(voltage_2 * current_2))
    
    # Posting into CSV
    with open("mass_id_2.csv", "a") as my_csv_file:
        csv_writer = csv.DictWriter(my_csv_file, fieldnames=fieldnames)

        info = {
            "Time": Time,
            "Set_Point_Temp.": temp_input,
            "Average_Internal_Temp.": average_internal_temp,
            "T1_cal": T1_cal,
            "T2_cal": T2_cal,
            "T3_cal": T3_cal,
            "T4_cal": T4_cal,
            "T5_cal": T5_cal,
            "T6_cal": T6_cal,
            "Ambient_temp": Ambient_temp,
            "Hot_Plate_Temp.": hot_plate,
            "Power_Input": Total_Power
        }
        csv_writer.writerow(info)
        
        print(Time, temp_input, average_internal_temp, T1_cal, T2_cal, T3_cal, T4_cal, T5_cal, T6_cal, Ambient_temp, hot_plate, Total_Power)
        
        time.sleep(time_interval)  # Wait for the desired time interval before taking the next measurement

        Time += time_interval  # Increment the elapsed time by the time interval
        
# Closing the unit when done
tc08.usb_tc08_close_unit(chandle1)
tc08.usb_tc08_close_unit(chandle2)
