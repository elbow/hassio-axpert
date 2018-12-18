#! /usr/bin/python

# Axpert Inverter control script

# Read values from inverter, sends values to emonCMS,
# read electric low or high tarif from emonCMS and setting charger and mode to hold batteries fully charged
# controls grid charging current to meet circuit braker maximum alloweble grid current(power)
# calculation of CRC is done by XMODEM mode, but in firmware is wierd mistake in POP02 command, so exception of calculation is done in serial_command(command) function

import time, sys, string
#import sqlite3
import json
import datetime
import calendar
import os
import fcntl
import re
from struct import pack
import paho.mqtt.client as mqtt

def connect():
    global client
    client = mqtt.Client(client_id="replus_inverter")
    client.connect(os.environ['MQTT_SERVER'])
    try:
        global file
        global fd
        # file = open('/dev/hidraw0', 'r+')
        file = open('/dev/hidraw2', 'r+')
        fd = file.fileno()
        fl = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
    except Exception as e:
        print('error open file descriptor: ' + str(e))
        exit()

def disconnect():
    file.close()

# Polynomial for CRC-CCITT
POLY = 0x1021

def crc16x(buf):
    crc = 0
    for ch in buf:
        crc ^= ord(ch) << 8
        crc = (crc << 1 ^ POLY) if (crc & 0x8000) else crc << 1
        crc = (crc << 1 ^ POLY) if (crc & 0x8000) else crc << 1
        crc = (crc << 1 ^ POLY) if (crc & 0x8000) else crc << 1
        crc = (crc << 1 ^ POLY) if (crc & 0x8000) else crc << 1
        crc = (crc << 1 ^ POLY) if (crc & 0x8000) else crc << 1
        crc = (crc << 1 ^ POLY) if (crc & 0x8000) else crc << 1
        crc = (crc << 1 ^ POLY) if (crc & 0x8000) else crc << 1
        crc = (crc << 1 ^ POLY) if (crc & 0x8000) else crc << 1
    return crc & 0xffff

def serial_command(command):
    # print(command)
    try:
	checksum = crc16x(command)
	command_crc = command + pack("<H", checksum) + '\x0d'
	command_crc = command + '\x0d'
	# print command_crc.encode('hex')

        os.write(fd, command_crc)

        response = ''
        timeout_counter = 0
        while '\r' not in response:
            if timeout_counter > 500:
                raise Exception('Read operation timed out')
            timeout_counter += 1
            try:
                response += os.read(fd, 100)
            except Exception as e:
                # print("error reading response...: " + str(e))
                time.sleep(0.01)
            if len(response) > 0 and response[0] != '(' or 'NAKss' in response:
                raise Exception('NAKss')
        response = response.rstrip()
        lastI = response.find('\r')
	# As you see we don't check the CRC just remove it.
        response = response[1:lastI-2]
        # print(response)
        return response
    except Exception as e:
        print('error reading inverter...: ' + str(e))
        disconnect()
        time.sleep(0.5)
        connect()
        # return serial_command(command)
        return None

#def get_parallel_data():
#    #collect data from Replus inverter
#    try:
#        data = '{'
#        response = serial_command('QPGS0')
#        nums = response.split(' ')
#        if len(nums) < 27:
#            return ''
#
#        if nums[2] == 'L':
#            data += '"Gridmode":1'
#        else:
#            data += '"Gridmode":0'
#        data += ',"BatteryChargingCurrent": ' + str(int(nums[12]))
#        data += ',"BatteryDischargeCurrent": ' + str(int(nums[26]))
#        data += ',"TotalChargingCurrent": ' + str(int(nums[15]))
#        data += ',"GridVoltage": ' + str(float(nums[4]))
#        data += ',"GridFrequency": ' + str(float(nums[5]))
#        data += ',"OutputVoltage": ' + str(float(nums[6]))
#        data += ',"OutputFrequency": ' + str(float(nums[7]))
#        data += ',"OutputAparentPower": ' + str(int(nums[8]))
#        data += ',"OutputActivePower": ' + str(int(nums[9]))
#        data += ',"LoadPercentage": ' + str(int(nums[10]))
#        data += ',"BatteryVoltage": ' + str(float(nums[11]))
#        data += ',"BatteryCapacity": ' + str(float(nums[13]))
#        data += ',"PvInputVoltage": ' + str(float(nums[14]))
#        data += ',"TotalAcOutputApparentPower": ' + str(int(nums[16]))
#        data += ',"TotalAcOutputActivePower": ' + str(int(nums[17]))
#        data += ',"TotalAcOutputPercentage": ' + str(int(nums[18]))
#        # data += ',"InverterStatus": ' + nums[19]
#        data += ',"OutputMode": ' + str(int(nums[20]))
#        data += ',"ChargerSourcePriority": ' + str(int(nums[21]))
#        data += ',"MaxChargeCurrent": ' + str(int(nums[22]))
#        data += ',"MaxChargerRange": ' + str(int(nums[23]))
#        data += ',"MaxAcChargerCurrent": ' + str(int(nums[24]))
#        data += ',"PvInputCurrentForBattery": ' + str(int(nums[25]))
#        if nums[2] == 'B':
#            data += ',"Solarmode":1'
#        else:
#            data += ',"Solarmode":0'
#
#        data += '}'
#    except Exception as e:
#        print('error parsing inverter data...: ' + str(e))
#        return ''
#    return data

def get_data():
    # CollectC data from Replus inverter
    try:
	inverter_time = serial_command('QT')
        total_production = serial_command('QET')

	charge_info = serial_command('QCHGS')
	charge_nums = charge_info.split(' ')
	if len(charge_nums) < 4:
	    return None

        qpigs_response = serial_command('QPIGS')
        nums = qpigs_response.split(' ')
        if len(nums) < 21:
            return None


        data = '{\n'

	# 
	# 214.9 100042 50.1 0000.3 214.6 00000 50.1 000.0 002 437.9 437.9 052.9 ---.- 097 00000 00000 ----- 050.2 ---.- ---.- 036.0 D---10020

        # QPIGS reports lots of info, with some unused fields filled with - . What I have gathered:
        #  -  0 (235.2) Mains voltage
        #  -  1 (100065) Power to / from mains. The high 1 show the direction. 1 is the inverter using mains power.
        #  -  2 (50.0) Mains frequency
        #  -  3 (0006.2) Load AC Ampere. Probably the max of real or apparent.
        #  -  4 (235.8) Load Voltage
        #  -  5 (01442) Load power (Watt)
        #  -  6 (50.0) Load frequency
        #  -  7 (006.1) Load AC current
        #  -  8 (049) Output load %
        #  -  9 (445.3) Internal DC bus voltage 1
        #  - 10 (445.3) Internal DC bus voltage 2
        #  - 11 (053.9) Battery Voltage
        #  - 12 (---.-)
        #  - 13 (069) Its guestimate of % battery capacity
        #  - 14 (02563) PV Input power (Watt)
        #  - 15 (01377) Inverter power (Watt) ie power coming from PV and/or battery
        #  - 16 (------)
        #  - 17 (343.9) PV Voltage
        #  - 18 (---.-)
        #  - 19 (---.-)
        #  - 20 (060.0) Highest of the 3 temperature sensors
        #  - 21 (A---101001) Flags. The first letter is about the battery and can be:
        #      D - discharging
        #      A - constant current charging
        #      B - constant voltage charging
        #      F - float charging

	gridSign = '' if int(nums[1]) == 0 else '-' if nums[1][0] == '1' else '+'
        data += '  "InverterTime":"' + inverter_time + '",\n'
        data += '  "TotalProduction":' + str(int(total_production)) + ',\n'
	data += '  "GridVolts":' + str(float(nums[0])) + ',\n'
        data += '  "GridWatts":' + gridSign + str(int(nums[1][1:99])) + ',\n'
	data += '  "GridHz":' + str(float(nums[2])) + ',\n'
	data += '  "LoadAmpsMaybe":' + str(float(nums[3])) + ',\n'
	data += '  "LoadVolts":' + str(float(nums[4])) + ',\n'
	data += '  "LoadWatts":' + str(int(nums[5])) + ',\n'
	data += '  "LoadHz":' + str(float(nums[6])) + ',\n'
	data += '  "LoadAmps":' + str(float(nums[7])) + ',\n'
	data += '  "LoadPercent":' + str(int(nums[8])) + ',\n'
	data += '  "DCBusVolts1":' + str(float(nums[9])) + ',\n'
	data += '  "DCBusVolts2":' + str(float(nums[10])) + ',\n'
	data += '  "BatteryVolts":' + str(float(nums[11])) + ',\n'
	data += '  "BatteryPercent":' + str(int(nums[13])) + ',\n'
	data += '  "PVWatts":' + str(float(nums[14])) + ',\n'
	data += '  "PVVolts":' + str(float(nums[17])) + ',\n'
	data += '  "InverterWatts":' + str(int(nums[15])) + ',\n'
	data += '  "Temp":' + str(float(nums[20])) + ',\n'
	data += '  "BatteryChargeMode":"' + nums[21][0] + '",\n'
	data += '  "BatteryChargeAmps":' + str(float(charge_nums[0])) + ',\n'
	data += '  "BatteryFloatWatts":' + str(float(charge_nums[1])) + ',\n'
	data += '  "BatteryMaxChargeAmps":' + str(float(charge_nums[2])) + ',\n'
	data += '  "BatteryBulkChargeVolts":' + str(float(charge_nums[3])) + '\n'
        data += '}'
	print data
        return data
    except Exception as e:
        print('error parsing inverter data...: ' + str(e))
        return ''

def send_data(data, topic):
    try:
        client.publish(topic, data)
    except Exception as e:
        print("error sending to emoncms...: " + str(e))
        return 0
    return 1

def main():
    connect();
    while True:
        start = time.time()
        data = get_data()
        if not data is None:
            send = send_data(data, 'power/replus_inverter')

        time.sleep(max(1,10-(time.time()-start)))

def xxmain():
    connect()
    response = serial_command('QT')		# Date/time
    response = serial_command('QET')		# Total energy generated
    response = serial_command('QED')		# Total energy generated
    response = serial_command('QTPR')		# Temperatures
    response = serial_command('QCHGS')		# Battery info
    response = serial_command('QPI')
    response = serial_command('QPIGS')
    response = serial_command('QVFW')
    response = serial_command('QVFW2')

if __name__ == '__main__':
    main()
