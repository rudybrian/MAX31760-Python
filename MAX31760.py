#!/usr/bin/python

##########
##########
##
## Python class for MAX31760
##
##### Description #####
##
## Basic control functions for the MAX31760 fan speed controller. Extensive 
## code re-use from the Mikroe Fan 2 Click C library.
##
## Latest version can be found https://github.com/rudybrian/MAX31760-Python/
##
##### Additional References #####
##
## https://www.maximintegrated.com/en/products/sensors/MAX31760.html
## https://www.mikroe.com/fan-2-click
## 
##### Revision History #####
##
## v0.1 01/10/2021 Brian Rudy (brudyNO@SPAMpraecogito.com) First working version
##
##########
##########


from time import sleep
from Adafruit_I2C import Adafruit_I2C

class MAX31760(Adafruit_I2C):

    MAX31760_MIN_ADDR      = 0x50  # Default starting address
    MAX31760_MAX_ADDR      = 0x57  # Max address

    ## Register memory ranges ##
    MAX31760_ALLMEM_START  = 0x00
    MAX31760_ALLMEM_END    = 0x5F  # the end of addressable range is actually 0x5B
    MAX31760_USERMEM_START = 0x10  # 8 Bytes of General-Purpose User Memory
    MAX31760_USERMEM_END   = 0x17  #
    MAX31760_LUTMEM_START  = 0x20  # 48-Byte Lookup Table (LUT)
    MAX31760_LUTMEM_END    = 0x4F  #

    ## Register memory map ##
    MAX31760_CR1           = 0x00  # Control Register 1
    MAX31760_CR2           = 0x01  # Control Register 2
    MAX31760_CR3           = 0x02  # Control Register 3
    MAX31760_FFDC          = 0x03  # Fan Fault Duty Cycle
    MAX31760_MASK          = 0x04  # Alert Mask Register
    MAX31760_IFR           = 0x05  # Ideality Factor Register
    MAX31760_RHSH          = 0x06  # Remote High Set-point MSB
    MAX31760_RHSL          = 0x07  # Remote High Set-point LSB
    MAX31760_LOTSH         = 0x08  # Local Overtemperature Set-point MSB
    MAX31760_LOTSL         = 0x09  # Local Overtemperature Set-point LSB
    MAX31760_ROTSH         = 0x0A  # Remote Overtemperature Set-point MSB
    MAX31760_ROTSL         = 0x0B  # Remote Overtemperature Set-point LSB
    MAX31760_LHSH          = 0x0C  # Local High Set-point MSB
    MAX31760_LHSL          = 0x0D  # Local High Set-point LSB
    MAX31760_TCTH          = 0x0E  # TACH Count Threshold Register, MSB
    MAX31760_TCTL          = 0x0F  # TACH Count Threshold Register, LSB

    MAX31760_PWMR          = 0x50  # Direct Duty-Cycle Control Register
    MAX31760_PWMV          = 0x51  # Current PWM Duty-Cycle Register
    MAX31760_TC1H          = 0x52  # TACH1 Count Register, MSB
    MAX31760_TC1L          = 0x53  # TACH1 Count Register, LSB
    MAX31760_TC2H          = 0x54  # TACH2 Count Register, MSB
    MAX31760_TC2L          = 0x55  # TACH2 Count Register, LSB
    MAX31760_RTH           = 0x56  # Remote Temperature Reading Register, MSB
    MAX31760_RTL           = 0x57  # Remote Temperature Reading Register, MSB
    MAX31760_LTH           = 0x58  # Local Temperature Reading Register, MSB
    MAX31760_LTL           = 0x59  # Local Temperature Reading Register, MSB
    MAX31760_SR            = 0x5A  # Status Register

    ## Commands ##
    MAX31760_CMD_EEPROM    = 0x5B 

    ## Parameters ##
    MAX31760_EEPROM_READ   = 0x9F
    MAX31760_EEPROM_WRITE  = 0x1F

    ## Control Register 1 ##
    MAX31760_CTRL1_MASK_TEMP_ALERTS    = 0x80
    MAX31760_CTRL1_SW_POR              = 0x40
    MAX31760_CTRL1_LUT_HYST_2_CELS     = 0x00
    MAX31760_CTRL1_LUT_HYST_4_CELS     = 0x20
    MAX31760_CTRL1_PWM_FREQ_33_HZ      = 0x00
    MAX31760_CTRL1_PWM_FREQ_150_HZ     = 0x08
    MAX31760_CTRL1_PWM_FREQ_1500_HZ    = 0x10
    MAX31760_CTRL1_PWM_FREQ_25_KHZ     = 0x18
    MAX31760_CTRL1_PWM_POL_POS         = 0x00
    MAX31760_CTRL1_PWM_POL_NEG         = 0x04
    MAX31760_CTRL1_TEMP_IDX_TIS        = 0x00
    MAX31760_CTRL1_TEMP_IDX_GREATER    = 0x02
    MAX31760_CTRL1_LUT_IDX_LOCAL_TEMP  = 0x00
    MAX31760_CTRL1_LUT_IDX_REMOTE_TEMP = 0x01

    ## Control Register 2 ##
    MAX31760_CTRL2_NORMAL_MODE         = 0x00
    MAX31760_CTRL2_STANDBY_MODE        = 0x80
    MAX31760_CTRL2_ALERT_INTERR        = 0x00
    MAX31760_CTRL2_ALERT_COMP          = 0x40
    MAX31760_CTRL2_SPIN_UP_EN          = 0x20
    MAX31760_CTRL2_FF_OUTPUT_INTERR    = 0x00
    MAX31760_CTRL2_FF_OUTPUT_COMP      = 0x10
    MAX31760_CTRL2_FS_INPUT_EN         = 0x08
    MAX31760_CTRL2_RD_ACTIVE_LOW       = 0x00
    MAX31760_CTRL2_RD_ACTIVE_HIGH      = 0x04
    MAX31760_CTRL2_TACHO_SQUARE_WAVE   = 0x00
    MAX31760_CTRL2_TACHO_RD            = 0x02
    MAX31760_CTRL2_DIRECT_FAN_CTRL_EN  = 0x01

    ## Control Register 3 ##
    MAX31760_CTRL3_CLR_FAIL                  = 0x80
    MAX31760_CTRL3_FF_DETECT_EN              = 0x40
    MAX31760_CTRL3_PWM_RAMP_RATE_SLOW        = 0x00
    MAX31760_CTRL3_PWM_RAMP_RATE_SLOW_MEDIUM = 0x10
    MAX31760_CTRL3_PWM_RAMP_RATE_MEDIUM_FAST = 0x20
    MAX31760_CTRL3_PWM_RAMP_RATE_FAST        = 0x30
    MAX31760_CTRL3_TACHFULL_EN               = 0x08
    MAX31760_CTRL3_PULSE_STRETCH_EN          = 0x04
    MAX31760_CTRL3_TACH2_EN                  = 0x02
    MAX31760_CTRL3_TACH1_EN                  = 0x01

    ## Alarm Mask ##
    MAX31760_MASK_ALARM_LOCAL_TEMP_HIGH  = 0x20
    MAX31760_MASK_ALARM_LOCAL_OVERTEMP   = 0x10
    MAX31760_MASK_ALARM_REMOTE_TEMP_HIGH = 0x08
    MAX31760_MASK_ALARM_REMOTE_OVERTEMP  = 0x04
    MAX31760_MASK_ALARM_TACH2            = 0x02
    MAX31760_MASK_ALARM_TACH1            = 0x01
    MAX31760_MASK_ALARM_RESERVED_BITS    = 0xC0
    MAX31760_MASK_ALARM_NONE             = 0x00

    ## Status ##
    MAX31760_STAT_FLAG_PROGRAM_CORRUPT        = 0x80
    MAX31760_STAT_FLAG_REMOTE_DIODE_FAULT     = 0x40
    MAX31760_STAT_FLAG_LOCAL_HIGH_TEMP_ALARM  = 0x20
    MAX31760_STAT_FLAG_LOCAL_OVERTEMP_ALARM   = 0x10
    MAX31760_STAT_FLAG_REMOTE_HIGH_TEMP_ALARM = 0x08
    MAX31760_STAT_FLAG_REMOTE_OVERTEMP_ALARM  = 0x04
    MAX31760_STAT_FLAG_TACH2_ALARM            = 0x02
    MAX31760_STAT_FLAG_TACH1_ALARM            = 0x01

    ## Temperature ##
    MAX31760_MAX_TEMP_CELS  = 125
    MAX31760_ZERO_TEMP_CELS = 0
    MAX31760_MIN_TEMP_CELS  = -55

    ## LUT bytes ##
    MAX31760_LUT_NBYTES = 48

    ## Temp and PWM speed register resolution ##
    MAX31760_RESOL_TEMP_CELS      = 0.125
    MAX31760_FAN2_RESOL_SPEED_PER = 0.392156862745098

    ## Ideality LUT ##
    MAX31760_IDEALITY      = {
                             "0.9844": 0x00,
                             "0.9853": 0x01,
                             "0.9863": 0x02,
                             "0.9873": 0x03,
                             "0.9882": 0x04,
                             "0.9892": 0x05,
                             "0.9902": 0x06,
                             "0.9911": 0x07,
                             "0.9921": 0x08,
                             "0.9931": 0x09,
                             "0.9941": 0x0A,
                             "0.9950": 0x0B,
                             "0.9960": 0x0C,
                             "0.9970": 0x0D,
                             "0.9980": 0x0E,
                             "0.9990": 0x0F,
                             "1.0000": 0x10,
                             "1.0010": 0x11,
                             "1.0020": 0x12,
                             "1.0030": 0x13,
                             "1.0040": 0x14, # 2N3904 Manufacturers: ROHM Semiconductor, Diodes Incorporated, Chenmko Co, Infineon Tech, National Semi
                             "1.0050": 0x15, # 2N3904 Manufacturers: NXP, STMicroelectronics, On Semiconductor, Fairchild Semi
                             "1.0060": 0x16,
                             "1.0070": 0x17,
                             "1.0080": 0x18,
                             "1.0090": 0x19,
                             "1.0100": 0x1A,
                             "1.0110": 0x1B,
                             "1.0120": 0x1C,
                             "1.0130": 0x1D,
                             "1.0141": 0x1E,
                             "1.0151": 0x1F,
                             "1.0161": 0x20,
                             "1.0171": 0x21,
                             "1.0182": 0x22,
                             "1.0192": 0x23,
                             "1.0202": 0x24,
                             "1.0213": 0x25,
                             "1.0223": 0x26,
                             "1.0233": 0x27,
                             "1.0244": 0x28,
                             "1.0254": 0x29,
                             "1.0265": 0x2A,
                             "1.0275": 0x2B,
                             "1.0286": 0x2C,
                             "1.0296": 0x2D,
                             "1.0307": 0x2E,
                             "1.0317": 0x2F,
                             "1.0328": 0x30,
                             "1.0338": 0x31,
                             "1.0349": 0x32,
                             "1.0360": 0x33,
                             "1.0370": 0x34,
                             "1.0381": 0x35,
                             "1.0392": 0x36,
                             "1.0402": 0x37,
                             "1.0413": 0x38,
                             "1.0424": 0x39,
                             "1.0435": 0x3A,
                             "1.0445": 0x3B,
                             "1.0456": 0x3C,
                             "1.0467": 0x3D,
                             "1.0478": 0x3E,
                             "1.0489": 0x3F
                             }



    def __init__(self, addr = None, busnum = -1, debug = False):
        if addr is None:
                addr = self.MAX31760_MIN_ADDR

        self._addr = addr
        self._busnum = busnum
        self._debug = debug

    def begin(self):

        self.bus = Adafruit_I2C(self._addr, self._busnum, self._debug)

        return True


    def sendCommand(self, cmd, args):
        res = self.bus.writeList(cmd, args)

        sleep(0.05)
        #self.waitStatus()
        return res

    def readTemp(self, start_addr):
        TH = self.bus.readS8(start_addr)
        TL = self.bus.readU8(start_addr + 1)
        result = TH << 8
        result |= TL
        result *= self.MAX31760_RESOL_TEMP_CELS/(1 << 5)
        return result

    def writeTemp(self, start_addr, val):
        if ((start_addr != self.MAX31760_RHSH) and (start_addr != self.MAX31760_LOTSH) and (start_addr != self.MAX31760_ROTSH) and (start_addr != self.MAX31760_LHSH)):
            return False
        if ((val < -55) or (val > 125)):
            return False
        res = self.MAX31760_RESOL_TEMP_CELS
        res /= 1 << 5
        temp = val / res
        # 16 bit operations are having issues, so using two 8 bit operations for now. Will need to debug this later
        #self.bus.write16(start_addr, int(temp) & 0xFFE0)
        self.bus.write8(start_addr, int(temp) >> 8)
        self.bus.write8(start_addr + 1, int(temp) & 0xFF)
        return True

    def readTach(self, start_addr):
        if ((start_addr != self.MAX31760_TCTH) and (start_addr != self.MAX31760_TC1H) and (start_addr != self.MAX31760_TC2H)):
            return False
        tach_high = self.bus.readU8(start_addr)
        tach_low = self.bus.readU8(start_addr + 1)
        tach = (tach_high << 8) | tach_low
        res = 100000
        res /= tach
        #res /= 4
        res /= 2   # Two pulses per revolution is standard for four pin PC PWM cooling fans. We might need to make this configurable.
        res *= 60
        return res 

    # Write the tach threshold
    def writeTachThreshold(self, RPM):
        res = 100000
        res /= RPM
        res /= 2   # Two pulses per revolution is standard for four pin PC PWM cooling fans. We might need to make this configurable.
        tach = res * 60
        self.bus.write8(self.MAX31760_TCTH, tach)
        return True

    # Read the remote diode ideality factor
    def readIdeality(self):
        ideality_raw = self.bus.readU8(self.MAX31760_IFR)
        # Reverse the keys and values in the dictionary to make matching easier
        reversed_ideality_dict = {}
        for key, value in self.MAX31760_IDEALITY.items():
            reversed_ideality_dict[value] = key
        return reversed_ideality_dict[ideality_raw]
 
    # Write the remote diode ideality factor
    def writeIdeality(self, ideality):
        if (self.MAX31760_IDEALITY[str(ideality)]):
            self.bus.write8(self.MAX31760_IFR, self.MAX31760_IDEALITY[str(ideality)])
            return True
        else:
            return False

    # Write the fan duty cycle for direct speed control
    def writeDirectSpeedControl(self, duty_percent):
        duty = int(duty_percent / self.MAX31760_FAN2_RESOL_SPEED_PER)
        self.bus.write8(self.MAX31760_PWMR, duty)
        return True

    # Read the current duty cycle
    def readCurrentDutyCycle(self):
        duty = self.bus.readU8(self.MAX31760_PWMV)
        if (duty == 0xFF):
            duty_percent = 100.0
        else:
            duty_percent = duty * 100 / 256
        return duty_percent

    # Read the status register
    def readStatus(self):
        status_int = self.bus.readU8(self.MAX31760_SR)
        status_dict = {
                      "program_corrupt":        (status_int & self.MAX31760_STAT_FLAG_PROGRAM_CORRUPT) > 0,
                      "remote_diode_fault":     (status_int & self.MAX31760_STAT_FLAG_REMOTE_DIODE_FAULT) > 0,
                      "local_high_temp_alarm":  (status_int & self.MAX31760_STAT_FLAG_LOCAL_HIGH_TEMP_ALARM) > 0,
                      "local_overtemp_alarm":   (status_int & self.MAX31760_STAT_FLAG_LOCAL_OVERTEMP_ALARM) > 0,
                      "remote_high_temp_alarm": (status_int & self.MAX31760_STAT_FLAG_REMOTE_HIGH_TEMP_ALARM) > 0,
                      "remote_overtemp_alarm":  (status_int & self.MAX31760_STAT_FLAG_REMOTE_OVERTEMP_ALARM) > 0,
                      "tach2_alarm":            (status_int & self.MAX31760_STAT_FLAG_TACH2_ALARM) > 0,
                      "tach1_alarm":            (status_int & self.MAX31760_STAT_FLAG_TACH1_ALARM) > 0
                      }
        return status_dict

    # Read alert mask register
    def readAlertMask(self):
        alert_int = self.bus.readU8(self.MAX31760_MASK)
        alert_dict = {
                     "local_temp_high":         (alert_int & self.MAX31760_MASK_ALARM_LOCAL_TEMP_HIGH) > 0,
                     "local_overtemp":          (alert_int & self.MAX31760_MASK_ALARM_LOCAL_OVERTEMP) > 0,
                     "remote_temp_high":        (alert_int & self.MAX31760_MASK_ALARM_REMOTE_TEMP_HIGH) > 0,
                     "remote_overtemp":         (alert_int & self.MAX31760_MASK_ALARM_REMOTE_OVERTEMP) > 0,
                     "tach2":                   (alert_int & self.MAX31760_MASK_ALARM_TACH2) > 0,
                     "tach1":                   (alert_int & self.MAX31760_MASK_ALARM_TACH1) > 0,
                     }
        return alert_dict

    # Read control register 1
    def readControlRegister1(self):
        control_int = self.bus.readU8(self.MAX31760_CR1)
        control_dict = {
                     "alert_mask":              int((control_int & self.MAX31760_CTRL1_MASK_TEMP_ALERTS) > 0),
                     "software_POR":            control_int & self.MAX31760_CTRL1_SW_POR,
                     "LUT_hysteresis":          int((control_int & self.MAX31760_CTRL1_LUT_HYST_4_CELS) > 0),
                     "PWM_frequency":           (control_int & self.MAX31760_CTRL1_PWM_FREQ_25_KHZ) >> 3,
                     "PWM_polarity":            int((control_int & self.MAX31760_CTRL1_PWM_POL_NEG) > 0),
                     "max_temp_as_index":       int((control_int & self.MAX31760_CTRL1_TEMP_IDX_GREATER) > 0),
                     "temp_index_source":       int((control_int & self.MAX31760_CTRL1_LUT_IDX_REMOTE_TEMP) > 0)
                       }
        return control_dict

    # Read control register 2
    def readControlRegister2(self):
        control_int = self.bus.readU8(self.MAX31760_CR2)
        control_dict = {
                     "standby_enable":              (control_int & self.MAX31760_CTRL2_STANDBY_MODE) > 0,
                     "alert_selection":             int((control_int & self.MAX31760_CTRL2_ALERT_COMP) > 0),
                     "spin_up_enable":              (control_int & self.MAX31760_CTRL2_ALERT_COMP) > 0,
                     "fan_fault_mode":              int((control_int & self.MAX31760_CTRL2_FF_OUTPUT_COMP) > 0),
                     "FS_input_enable":             (control_int & self.MAX31760_CTRL2_FS_INPUT_EN) > 0,
                     "rotation_detection_polarity": int((control_int & self.MAX31760_CTRL2_RD_ACTIVE_HIGH) > 0),
                     "fan_sense_signal_type":       int((control_int & self.MAX31760_CTRL2_TACHO_RD) > 0),
                     "direct_fan_control_enable":   (control_int & self.MAX31760_CTRL2_DIRECT_FAN_CTRL_EN) > 0
                       }
        return control_dict
 
    # Read control register 3
    def readControlRegister3(self):
        control_int = self.bus.readU8(self.MAX31760_CR3)
        control_dict = {
                     "clear_fan_fail":             int((control_int & self.MAX31760_CTRL3_CLR_FAIL) > 0),
                     "fan_fail_detect_enable":     int((control_int & self.MAX31760_CTRL3_FF_DETECT_EN) > 0),
                     "PWM_ramp_rate":              (control_int & self.MAX31760_CTRL3_PWM_RAMP_RATE_FAST) >> 4,
                     "tach_full_enable":           (control_int & self.MAX31760_CTRL3_TACHFULL_EN) > 0,
                     "pulse_stretch_enable":       (control_int & self.MAX31760_CTRL3_PULSE_STRETCH_EN) > 0,
                     "tach2_enable":               (control_int & self.MAX31760_CTRL3_TACH2_EN) > 0,
                     "tach1_enable":               (control_int & self.MAX31760_CTRL3_TACH1_EN) > 0
                       }
        return control_dict

    # Read the local temperature sensor
    def readLocalTemp(self):
        result = self.readTemp(self.MAX31760_LTH)
        return result

    # Read the remote temperature sensor
    def readRemoteTemp(self):
        result = self.readTemp(self.MAX31760_RTH)
        return result

    # Write local overtemp setpoint
    def writeLocalOvertempSetpoint(self, val):
        result = self.writeTemp(self.MAX31760_LOTSH, val)
        return result

    # Read local overtemp setpoint
    def readLocalOvertempSetpoint(self):
        result = self.readTemp(self.MAX31760_LOTSH)
        return result

    # Write remote overtemp setpoint
    def writeRemoteOvertempSetpoint(self, val):
        result = self.writeTemp(self.MAX31760_ROTSH, val)
        return result

    # Read remote overtemp setpoint
    def readRemoteOvertempSetpoint(self):
        result = self.readTemp(self.MAX31760_ROTSH)
        return result

    # Write local temp high setpoint
    def writeLocalTempHighSetpoint(self, val):
        result = self.writeTemp(self.MAX31760_LHSH, val)
        return result

    # Read local temp high setpoint
    def readLocalTempHighSetpoint(self):
        result = self.readTemp(self.MAX31760_LHSH)
        return result

    # Write remote temp high setpoint
    def writeRemoteTempHighSetpoint(self, val):
        result = self.writeTemp(self.MAX31760_RHSH, val)
        return result

    # Read remote temp high setpoint
    def readRemoteTempHighSetpoint(self):
        result = self.readTemp(self.MAX31760_RHSH)
        return result

    # Set some sane defaults
    def setDefaults(self):
        self.bus.write8(self.MAX31760_CR1, self.MAX31760_CTRL1_PWM_FREQ_33_HZ | self.MAX31760_CTRL1_PWM_POL_POS)
        self.bus.write8(self.MAX31760_CR2, self.MAX31760_CTRL2_NORMAL_MODE | self.MAX31760_CTRL2_ALERT_COMP | self.MAX31760_CTRL2_FF_OUTPUT_COMP | self.MAX31760_CTRL2_DIRECT_FAN_CTRL_EN)
        self.bus.write8(self.MAX31760_CR3, self.MAX31760_CTRL3_CLR_FAIL | self.MAX31760_CTRL3_PWM_RAMP_RATE_FAST | self.MAX31760_CTRL3_PULSE_STRETCH_EN | self.MAX31760_CTRL3_TACH1_EN)
        return True

    # Clear fan fail status bits 
    def clearFF(self):
        cr3_val = self.bus.readU8(self.MAX31760_CR3)
        self.bus.write8(self.MAX31760_CR3, cr3_val | self.MAX31760_CTRL3_CLR_FAIL)
        return True

    # Read the Tach count threshold
    def readTachCountThreshold(self):
        tach = self.readTach(self.MAX31760_TCTH)
        return tach

    # Read tach 1
    def readTach1(self):
        tach = self.readTach(self.MAX31760_TC1H)
        return tach

    # Read tach 2
    def readTach2(self):
        tach = self.readTach(self.MAX31760_TC2H)
        return tach

