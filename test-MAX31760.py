#!/usr/bin/python

####
#
# test-MAX31760.py
#
####
#
# Sample code to test the MAX31760 via the Python library
#
####


import logging
import json
import os
import errno
import atexit
import socket
from sys import argv
from time import sleep
from datetime import datetime
from MAX31760 import MAX31760

def MAX31760_start():
    global device
    logging.info('MAX31760 Start')
    device = MAX31760()
    device.begin()


# Main loop

print 'Starting up'
MAX31760_start()

print '\nsetting some sane defaults'
device.setDefaults()

print '\nSetting remote diode ideality factor'
device.writeIdeality('1.0050')

print '\nReading remote diode ideality factor'
idf = device.readIdeality()
print 'Remote diode ideality factor=' + str(idf)

print '\nReading local temperature'
localtemp = device.readLocalTemp()
print 'Local Temp=' + str(localtemp)

print '\nReading remote temperature'
remotetemp = device.readRemoteTemp()
print 'Remote Temp=' + str(remotetemp)

#print '\nClearing fan fault'
#device.clearFF()

print '\nSetting the direct PWM duty cycle to 50%'
device.writeDirectSpeedControl(50.0)

print '\nReading the current duty cycle'
current_duty_percent = device.readCurrentDutyCycle()
print 'Current duty cycle=' + str(current_duty_percent)

print '\nReading the status register'
status = device.readStatus()
print 'program_corrupt=' + str(status['program_corrupt'])
print 'remote_diode_fault=' + str(status['remote_diode_fault'])
print 'local_high_temp_alarm=' + str(status['local_high_temp_alarm'])
print 'local_overtemp_alarm=' + str(status['local_overtemp_alarm'])
print 'remote_high_temp_alarm=' + str(status['remote_high_temp_alarm'])
print 'remote_overtemp_alarm=' + str(status['remote_overtemp_alarm'])
print 'tach2_alarm=' + str(status['tach2_alarm'])
print 'tach1_alarm=' + str(status['tach1_alarm'])

print '\nReading the alert mask register'
alert = device.readAlertMask()
print 'local_temp_high=' + str(alert['local_temp_high'])
print 'local_overtemp=' + str(alert['local_overtemp'])
print 'remote_temp_high=' + str(alert['remote_temp_high'])
print 'remote_overtemp=' + str(alert['remote_overtemp'])
print 'tach2=' + str(alert['tach2'])
print 'tach1=' + str(alert['tach1'])

print '\nReading control register 1'
cr1 = device.readControlRegister1()
print 'alert_mask=' + str(cr1['alert_mask'])
print 'software_POR=' + str(cr1['software_POR'])
print 'LUT_hysteresis=' + str(cr1['LUT_hysteresis'])
print 'PWM_frequency=' + str(cr1['PWM_frequency'])
print 'PWM_polarity=' + str(cr1['PWM_polarity'])
print 'max_temp_as_index=' + str(cr1['max_temp_as_index'])
print 'temp_index_source=' + str(cr1['temp_index_source'])

print '\nReading control register 2'
cr2 = device.readControlRegister2()
print 'standby_enable=' + str(cr2['standby_enable'])
print 'alert_selection=' + str(cr2['alert_selection'])
print 'spin_up_enable=' + str(cr2['spin_up_enable'])
print 'fan_fault_mode=' + str(cr2['fan_fault_mode'])
print 'FS_input_enable=' + str(cr2['FS_input_enable'])
print 'rotation_detection_polarity=' + str(cr2['rotation_detection_polarity'])
print 'fan_sense_signal_type=' + str(cr2['fan_sense_signal_type'])
print 'direct_fan_control_enable=' + str(cr2['direct_fan_control_enable'])

print '\nReading control register 3'
cr3 = device.readControlRegister3()
print 'clear_fan_fail=' + str(cr3['clear_fan_fail'])
print 'fan_fail_detect_enable=' + str(cr3['fan_fail_detect_enable'])
print 'PWM_ramp_rate=' + str(cr3['PWM_ramp_rate'])
print 'tach_full_enable=' + str(cr3['tach_full_enable'])
print 'pulse_stretch_enable=' + str(cr3['pulse_stretch_enable'])
print 'tach2_enable=' + str(cr3['tach2_enable'])
print 'tach1_enable=' + str(cr3['tach1_enable'])

#print '\nWriting local overtemp setpoint'
#device.writeLocalOvertempSetpoint(50.5)

print '\nReading local overtemp setpoint'
lots = device.readLocalOvertempSetpoint()
print 'Local overtemp setpoint=' + str(lots)

#print '\nWriting remote overtemp setpoint'
#device.writeRemoteOvertempSetpoint(50.5)

print '\nReading remote overtemp setpoint'
lots = device.readRemoteOvertempSetpoint()
print 'Remote overtemp setpoint=' + str(lots)

#print '\nWriting local temp high setpoint'
#device.writeLocalTempHighSetpoint(45.5)

print '\nReading local temp high setpoint'
lots = device.readLocalTempHighSetpoint()
print 'Local overtemp setpoint=' + str(lots)

#print '\nWriting remote temp high setpoint'
#device.writeRemoteTempHighSetpoint(45.5)

print '\nReading remote temp high setpoint'
lots = device.readRemoteTempHighSetpoint()
print 'Remote overtemp setpoint=' + str(lots)

print '\nReading tach count threshold'
tct = device.readTachCountThreshold()
print 'Tach count threshold=' + str(tct)

print '\nReading tach 1'
tach1 = device.readTach1()
print 'Tach 1=' + str(tach1)

print '\nReading tach 2'
tach2 = device.readTach2()
print 'Tach 2=' + str(tach2)

print '\nWriting fan LUT value for 110-125 to 99'
device.writeFanLUT(125, 99)

print '\nReading the fan LUT'
fan_LUT = device.readFanLUT()
for key in sorted(fan_LUT):
    print(str(key) + ': low=' + str(fan_LUT[key]['low']) + ", high=" + str(fan_LUT[key]['high']) + ', temp=' + str(fan_LUT[key]['temp']))

