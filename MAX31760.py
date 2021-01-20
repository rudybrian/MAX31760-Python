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

    ## Alert Mask ##
    MAX31760_ALERT_MASK_LOCAL_TEMP_HIGH      = 0x20
    MAX31760_ALERT_MASK_LOCAL_OVERTEMP       = 0x10
    MAX31760_ALERT_MASK_REMOTE_TEMP_HIGH     = 0x08
    MAX31760_ALERT_MASK_REMOTE_OVERTEMP      = 0x04
    MAX31760_ALERT_MASK_TACH2                = 0x02
    MAX31760_ALERT_MASK_TACH1                = 0x01
    MAX31760_ALERT_MASK_RESERVED_BITS        = 0xC0
    MAX31760_ALERT_MASK_NONE                 = 0x00

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
    MAX31760_MAX_TEMP_CELS   = 125
    MAX31760_ZERO_TEMP_CELS  = 0
    MAX31760_MIN_TEMP_CELS   = -55

    ## LUT bytes ##
    MAX31760_LUT_NBYTES      = 48

    ## Temp and PWM speed register resolution ##
    MAX31760_RESOL_TEMP_CELS = 0.125
    MAX31760_RESOL_SPEED_PER = 0.392156862745098

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


    ## Ideality LUT ##
    MAX31760_FAN_LUT       = {
                             1: {'low': -55, 'high': 18, 'addr': 0x20},
                             2: {'low': 18, 'high': 20, 'addr': 0x21},
                             3: {'low': 20, 'high': 22, 'addr': 0x22},
                             4: {'low': 22, 'high': 24, 'addr': 0x23},
                             5: {'low': 24, 'high': 26, 'addr': 0x24},
                             6: {'low': 26, 'high': 28, 'addr': 0x25},
                             7: {'low': 28, 'high': 30, 'addr': 0x26},
                             8: {'low': 30, 'high': 32, 'addr': 0x27},
                             9: {'low': 32, 'high': 34, 'addr': 0x28},
                             10: {'low': 34, 'high': 36, 'addr': 0x29},
                             11: {'low': 36, 'high': 38, 'addr': 0x2A},
                             12: {'low': 38, 'high': 40, 'addr': 0x2B},
                             13: {'low': 40, 'high': 42, 'addr': 0x2C},
                             14: {'low': 42, 'high': 44, 'addr': 0x2D},
                             15: {'low': 44, 'high': 46, 'addr': 0x2E},
                             16: {'low': 46, 'high': 48, 'addr': 0x2F},
                             17: {'low': 48, 'high': 50, 'addr': 0x30},
                             18: {'low': 50, 'high': 52, 'addr': 0x31},
                             19: {'low': 52, 'high': 54, 'addr': 0x32},
                             20: {'low': 54, 'high': 56, 'addr': 0x33},
                             21: {'low': 56, 'high': 58, 'addr': 0x34},
                             22: {'low': 58, 'high': 60, 'addr': 0x35},
                             23: {'low': 60, 'high': 62, 'addr': 0x36},
                             24: {'low': 62, 'high': 64, 'addr': 0x37},
                             25: {'low': 64, 'high': 66, 'addr': 0x38},
                             26: {'low': 66, 'high': 68, 'addr': 0x39},
                             27: {'low': 68, 'high': 70, 'addr': 0x3A},
                             28: {'low': 70, 'high': 72, 'addr': 0x3B},
                             29: {'low': 72, 'high': 74, 'addr': 0x3C},
                             30: {'low': 74, 'high': 76, 'addr': 0x3D},
                             31: {'low': 76, 'high': 78, 'addr': 0x3E},
                             32: {'low': 78, 'high': 80, 'addr': 0x3F},
                             33: {'low': 80, 'high': 82, 'addr': 0x40},
                             34: {'low': 82, 'high': 84, 'addr': 0x41},
                             35: {'low': 84, 'high': 86, 'addr': 0x42},
                             36: {'low': 86, 'high': 88, 'addr': 0x43},
                             37: {'low': 88, 'high': 90, 'addr': 0x44},
                             38: {'low': 90, 'high': 92, 'addr': 0x45},
                             39: {'low': 92, 'high': 94, 'addr': 0x46},
                             40: {'low': 94, 'high': 96, 'addr': 0x47},
                             41: {'low': 96, 'high': 98, 'addr': 0x48},
                             42: {'low': 98, 'high': 100, 'addr': 0x49},
                             43: {'low': 100, 'high': 102, 'addr': 0x4A},
                             44: {'low': 102, 'high': 104, 'addr': 0x4B},
                             45: {'low': 104, 'high': 106, 'addr': 0x4C},
                             46: {'low': 106, 'high': 108, 'addr': 0x4D},
                             47: {'low': 108, 'high': 110, 'addr': 0x4E},
                             48: {'low': 110, 'high': 125, 'addr': 0x4F}
                             }

    # CR1 LUTs
    MAX31760_ALTMSK_MODE   = {
                             "enabled":     0,
                             "masked":      MAX31760_CTRL1_MASK_TEMP_ALERTS
                             }

    MAX31760_HYST_LUT      = {
                             2:             MAX31760_CTRL1_LUT_HYST_2_CELS,
                             4:             MAX31760_CTRL1_LUT_HYST_4_CELS
                             }

    MAX31760_PWM_FREQ_LUT  = {
                             "33Hz":        MAX31760_CTRL1_PWM_FREQ_33_HZ,
                             "150Hz":       MAX31760_CTRL1_PWM_FREQ_150_HZ,
                             "1500Hz":      MAX31760_CTRL1_PWM_FREQ_1500_HZ,
                             "25kHz":       MAX31760_CTRL1_PWM_FREQ_25_KHZ
                             }

    MAX31760_POL_LUT       = {
                             "positive":    MAX31760_CTRL1_PWM_POL_POS,
                             "negative":    MAX31760_CTRL1_PWM_POL_NEG
                             }

    MAX31760_MTI_LUT       = {
                             "temp_index_source": MAX31760_CTRL1_TEMP_IDX_TIS,
                             "local_remote_max":  MAX31760_CTRL1_TEMP_IDX_GREATER
                             }

    MAX31760_TIS_LUT       = {
                             "local":       MAX31760_CTRL1_LUT_IDX_LOCAL_TEMP,
                             "remote":      MAX31760_CTRL1_LUT_IDX_REMOTE_TEMP
                             }

    # CR2 LUTs
    MAX31760_INT_COMP_MODE = {
                             "interrupt":   0,
                             "comparator":  1
                             }

    MAX31760_RDPS_MODE     = {
                             "low":         MAX31760_CTRL2_RD_ACTIVE_LOW,
                             "high":        MAX31760_CTRL2_RD_ACTIVE_HIGH
                             }

    MAX31760_FSST_MODE     = {
                             "square-wave": MAX31760_CTRL2_TACHO_SQUARE_WAVE,
                             "RD-signal":   MAX31760_CTRL2_TACHO_RD
                             }

    # CR3 LUT
    MAX31760_PWM_RAMP_RATE = {
                             "slow":        MAX31760_CTRL3_PWM_RAMP_RATE_SLOW,
                             "slow-medium": MAX31760_CTRL3_PWM_RAMP_RATE_SLOW_MEDIUM,
                             "medium-fast": MAX31760_CTRL3_PWM_RAMP_RATE_MEDIUM_FAST,
                             "fast":        MAX31760_CTRL3_PWM_RAMP_RATE_FAST
                             }

    #### Functions ####

    def __init__(self, addr = None, busnum = -1, debug = False):
        if addr is None:
                addr = self.MAX31760_MIN_ADDR
        self._addr = addr
        self._busnum = busnum
        self._debug = debug

    def begin(self):
        self.bus = Adafruit_I2C(self._addr, self._busnum, self._debug)
        return True

    def invertDict(self, in_dict):
        out_dict = {}
        for key, value in in_dict.items():
            out_dict[value] = key
        return out_dict

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
        reversed_ideality_dict = self.invertDict(self.MAX31760_IDEALITY)
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
        duty = int(round(duty_percent / self.MAX31760_RESOL_SPEED_PER))
        self.bus.write8(self.MAX31760_PWMR, duty)
        return True

    # Read the current duty cycle
    def readCurrentDutyCycle(self):
        duty = self.bus.readU8(self.MAX31760_PWMV)
        duty_percent = int(round(duty * self.MAX31760_RESOL_SPEED_PER))
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
                     "local_temp_high":         (alert_int & self.MAX31760_ALERT_MASK_LOCAL_TEMP_HIGH) > 0,
                     "local_overtemp":          (alert_int & self.MAX31760_ALERT_MASK_LOCAL_OVERTEMP) > 0,
                     "remote_temp_high":        (alert_int & self.MAX31760_ALERT_MASK_REMOTE_TEMP_HIGH) > 0,
                     "remote_overtemp":         (alert_int & self.MAX31760_ALERT_MASK_REMOTE_OVERTEMP) > 0,
                     "tach2":                   (alert_int & self.MAX31760_ALERT_MASK_TACH2) > 0,
                     "tach1":                   (alert_int & self.MAX31760_ALERT_MASK_TACH1) > 0,
                     }
        return alert_dict

    # Write alert mask register
    def writeAlertMask(self, param, value):
        alert_int = self.bus.readU8(self.MAX31760_MASK)
        alert_dict = self.readAlertMask()
        if (param in alert_dict):
            if (param == "local_temp_high"):
                val_masked = self.MAX31760_ALERT_MASK_LOCAL_TEMP_HIGH
            elif (param == "local_overtemp"):
                val_masked = self.MAX31760_ALERT_MASK_LOCAL_OVERTEMP
            elif (param == "remote_temp_high"):
                val_masked = self.MAX31760_ALERT_MASK_REMOTE_TEMP_HIGH
            elif (param == "remote_overtemp"):
                val_masked = self.MAX31760_ALERT_MASK_REMOTE_OVERTEMP
            elif (param == "tach2"):
                val_masked = self.MAX31760_ALERT_MASK_TACH2
            elif (param == "tach1"):
                val_masked = self.MAX31760_ALERT_MASK_TACH1
            else:
                return False
            if (value):
                val_composite = val_masked | alert_int
            else:
                if (alert_int & val_masked):
                    val_composite = alert_int ^ val_masked
                else:
                    val_composite = alert_int
            self.bus.write8(self.MAX31760_MASK, val_composite)
            return True
        else:
            return False
            
    # Read the fan fault duty cycle register
    def readFanFaultDutyCycle(self):
        duty = self.bus.readU8(self.MAX31760_FFDC)
        duty_percent = int(round(duty * self.MAX31760_RESOL_SPEED_PER))
        return duty_percent

    # Write the fan fault duty cycle register
    def writeFanFaultDutyCycle(self, duty_percent):
        duty = int(round(duty_percent / self.MAX31760_RESOL_SPEED_PER))
        self.bus.write8(self.MAX31760_FFDC, duty)
        return True

    # Read control register 1
    def readControlRegister1(self):
        altmsk_mode_inverted = self.invertDict(self.MAX31760_ALTMSK_MODE)
        hyst_LUT_inverted = self.invertDict(self.MAX31760_HYST_LUT)
        PWM_freq_LUT_inverted = self.invertDict(self.MAX31760_PWM_FREQ_LUT)
        polarity_LUT_inverted = self.invertDict(self.MAX31760_POL_LUT)
        max_temp_index_LUT_inverted = self.invertDict(self.MAX31760_MTI_LUT)
        temp_index_source_LUT_inverted = self.invertDict(self.MAX31760_TIS_LUT)
        control_int = self.bus.readU8(self.MAX31760_CR1)
        control_dict = {
                     "alert_mask":              altmsk_mode_inverted[control_int & self.MAX31760_CTRL1_MASK_TEMP_ALERTS],
                     "software_POR":            (control_int & self.MAX31760_CTRL1_SW_POR) > 0,
                     "LUT_hysteresis":          hyst_LUT_inverted[control_int & self.MAX31760_CTRL1_LUT_HYST_4_CELS],
                     "PWM_frequency":           PWM_freq_LUT_inverted[control_int & self.MAX31760_CTRL1_PWM_FREQ_25_KHZ],
                     "PWM_polarity":            polarity_LUT_inverted[control_int & self.MAX31760_CTRL1_PWM_POL_NEG],
                     "max_temp_as_index":       max_temp_index_LUT_inverted[control_int & self.MAX31760_CTRL1_TEMP_IDX_GREATER],
                     "temp_index_source":       temp_index_source_LUT_inverted[control_int & self.MAX31760_CTRL1_LUT_IDX_REMOTE_TEMP]
                       }
        return control_dict

    # Write control register 1
    def writeControlRegister1(self, param, value):
        control_int = self.bus.readU8(self.MAX31760_CR1)
        if (param == "alert_mask"):
            if (value in self.MAX31760_ALTMSK_MODE):
                val_composite = ((control_int | self.MAX31760_CTRL1_MASK_TEMP_ALERTS) ^ self.MAX31760_CTRL1_MASK_TEMP_ALERTS) | self.MAX31760_ALTMSK_MODE[value]
            else:
                return False
        elif (param == "software_POR"):
            if (value):
                val_composite = MAX31760_CTRL1_SW_POR # We don't need special logic here, as this will clear everything in RAM on POR.
            else:
                val_composite = control_int
        elif (param == "LUT_hysteresis"):
            if (value in self.MAX31760_HYST_LUT):
                val_composite = ((control_int | self.MAX31760_CTRL1_LUT_HYST_4_CELS) ^ self.MAX31760_CTRL1_LUT_HYST_4_CELS) | self.MAX31760_HYST_LUT[value]
            else:
                return False
        elif (param == "PWM_frequency"):
            if (value in self.MAX31760_PWM_FREQ_LUT):
                val_composite = ((control_int | self.self.MAX31760_CTRL1_PWM_FREQ_25_KHZ) ^ self.self.MAX31760_CTRL1_PWM_FREQ_25_KHZ) | self.MAX31760_PWM_FREQ_LUT[value]
            else:
                return False
        elif (param == "PWM_polarity"):
            if (value in self.MAX31760_POL_LUT):
                val_composite = ((control_int | self.MAX31760_CTRL1_PWM_POL_NEG) ^ self.MAX31760_CTRL1_PWM_POL_NEG) | self.MAX31760_POL_LUT[value]
            else:
                return False
        elif (param == "max_temp_as_index"):
            if (value in self.MAX31760_MTI_LUT):
                val_composite = ((control_int | self.MAX31760_CTRL1_TEMP_IDX_GREATER) ^ self.MAX31760_CTRL1_TEMP_IDX_GREATER) | self.MAX31760_MTI_LUT[value]
            else:
                return False
        elif (param == "temp_index_source"):
            if (value in self.MAX31760_TIS_LUT):
                val_composite = ((control_int | self.MAX31760_CTRL1_LUT_IDX_REMOTE_TEMP) ^ self.MAX31760_CTRL1_LUT_IDX_REMOTE_TEMP) | self.MAX31760_TIS_LUT[value]
        else:
            return False
        self.bus.write8(self.MAX31760_CR1, val_composite)
        return True

    # Read control register 2
    def readControlRegister2(self):
        int_comp_LUT_inverted = self.invertDict(self.MAX31760_INT_COMP_MODE)
        rdps_mode_inverted = self.invertDict(self.MAX31760_RDPS_MODE)
        fsst_mode_inverted = self.invertDict(self.MAX31760_FSST_MODE)
        control_int = self.bus.readU8(self.MAX31760_CR2)
        control_dict = {
                     "standby_enable":              (control_int & self.MAX31760_CTRL2_STANDBY_MODE) > 0,
                     "alert_selection":             int_comp_LUT_inverted[(control_int & self.MAX31760_CTRL2_ALERT_COMP) > 0],
                     "spin_up_enable":              (control_int & self.MAX31760_CTRL2_ALERT_COMP) > 0,
                     "fan_fault_mode":              int_comp_LUT_inverted[(control_int & self.MAX31760_CTRL2_FF_OUTPUT_COMP) > 0],
                     "FS_input_enable":             (control_int & self.MAX31760_CTRL2_FS_INPUT_EN) > 0,
                     "rotation_detection_polarity": rdps_mode_inverted[control_int & self.MAX31760_CTRL2_RD_ACTIVE_HIGH],
                     "fan_sense_signal_type":       fsst_mode_inverted[control_int & self.MAX31760_CTRL2_TACHO_RD],
                     "direct_fan_control_enable":   (control_int & self.MAX31760_CTRL2_DIRECT_FAN_CTRL_EN) > 0
                       }
        return control_dict

    # Write control register 2
    def writeControlRegister2(self, param, value):
        control_int = self.bus.readU8(self.MAX31760_CR2)
        if (param == "standby_enable"):
            if (value):
                val_composite = control_int | self.MAX31760_CTRL2_STANDBY_MODE
            else:
                val_composite = (control_int | self.MAX31760_CTRL2_STANDBY_MODE) ^ self.MAX31760_CTRL2_STANDBY_MODE
        elif (param == "alert_selection"):
            if (value in self.MAX31760_INT_COMP_MODE):
                val_composite = ((control_int | self.MAX31760_CTRL2_ALERT_COMP) ^ self.MAX31760_CTRL2_ALERT_COMP) | self.MAX31760_INT_COMP_MODE[value]
            else:
                return False
        elif (param == "spin_up_enable"):
            if (value):
                val_composite = control_int | self.MAX31760_CTRL2_ALERT_COMP
            else:
                val_composite = (control_int | self.MAX31760_CTRL2_ALERT_COMP) ^  self.MAX31760_CTRL2_ALERT_COMP
        elif (param == "fan_fault_mode"):
            if (value in self.MAX31760_INT_COMP_MODE):
                val_composite = ((control_int | self.MAX31760_CTRL2_FF_OUTPUT_COMP) ^ self.MAX31760_CTRL2_FF_OUTPUT_COMP) | self.MAX31760_INT_COMP_MODE[value]
            else:
                return False
        elif (param == "FS_input_enable"):
            if (value):
                val_composite = control_int | self.MAX31760_CTRL2_FS_INPUT_EN
            else:
                val_composite = (control_int | self.MAX31760_CTRL2_FS_INPUT_EN) ^ self.MAX31760_CTRL2_FS_INPUT_EN
        elif (param == "rotation_detection_polarity"):
            if (value in self.MAX31760_RDPS_MODE):
                val_composite = ((control_int | self.MAX31760_CTRL2_RD_ACTIVE_HIGH) ^ self.MAX31760_CTRL2_RD_ACTIVE_HIGH) | self.MAX31760_RDPS_MODE[value]
            else:
                return False
        elif (param == "fan_sense_signal_type"):
            if (value in self.MAX31760_FSST_MODE):
                val_composite = ((control_int | self.MAX31760_CTRL2_TACHO_RD) ^ self.MAX31760_CTRL2_TACHO_RD) | self.MAX31760_FSST_MODE[value]
            else:
                return False
        elif (param == "direct_fan_control_enable"):
            if (value):
                val_composite = control_int | self.MAX31760_CTRL2_DIRECT_FAN_CTRL_EN
            else:
                val_composite = (control_int | self.MAX31760_CTRL2_DIRECT_FAN_CTRL_EN) ^ self.MAX31760_CTRL2_DIRECT_FAN_CTRL_EN
        else:
            return False
        self.bus.write8(self.MAX31760_CR2, val_composite)
        return True

    # Read control register 3
    def readControlRegister3(self):
        ramp_rate_LUT_inverted = self.invertDict(self.MAX31760_PWM_RAMP_RATE)
        control_int = self.bus.readU8(self.MAX31760_CR3)
        control_dict = {
                     "clear_fan_fail":             (control_int & self.MAX31760_CTRL3_CLR_FAIL) > 0,
                     "fan_fail_detect_enable":     (control_int & self.MAX31760_CTRL3_FF_DETECT_EN) > 0,
                     "PWM_ramp_rate":              ramp_rate_LUT_inverted[control_int & self.MAX31760_CTRL3_PWM_RAMP_RATE_FAST],
                     "tach_full_enable":           (control_int & self.MAX31760_CTRL3_TACHFULL_EN) > 0,
                     "pulse_stretch_enable":       (control_int & self.MAX31760_CTRL3_PULSE_STRETCH_EN) > 0,
                     "tach2_enable":               (control_int & self.MAX31760_CTRL3_TACH2_EN) > 0,
                     "tach1_enable":               (control_int & self.MAX31760_CTRL3_TACH1_EN) > 0
                       }
        return control_dict

    # Write control register 3

    # Read the values in the fan LUT
    def readFanLUT(self):
        fan_LUT_dict = {}
        for key, value in self.MAX31760_FAN_LUT.items():
            fan_LUT_dict[key] = {}
            fan_LUT_dict[key]['temp'] = int(round(self.bus.readU8(value['addr']) * self.MAX31760_RESOL_SPEED_PER))
            fan_LUT_dict[key]['low'] = value['low']
            fan_LUT_dict[key]['high'] = value['high']
        return fan_LUT_dict

    # Write a value into the fan LUT
    def writeFanLUT(self, temp_high, duty_percent):
        for key, value in self.MAX31760_FAN_LUT.items():
            if (value['high'] == temp_high):
                self.bus.write8(value['addr'], int(round(duty_percent / self.MAX31760_RESOL_SPEED_PER)))
                return True
        return False

    # Write EEPROM to RAM
    def writeEEPROMToRAM(self):
        self.bus.write8(self.MAX31760_CMD_EEPROM, self.MAX31760_EEPROM_READ)
        return True

    # Write RAM to EEPROM
    def writeRAMToEEPROM(self):
        self.bus.write8(self.MAX31760_CMD_EEPROM, self.MAX31760_EEPROM_WRITE)
        return True

    ### utility functions

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

    # Set some sane defaults for 4 pin PWM PC fan
    # 25kHz, positive polarity, pulse stretching disabled, tachful disabled
    def setDefaults(self):
        self.bus.write8(self.MAX31760_CR1, self.MAX31760_CTRL1_PWM_FREQ_25_KHZ | self.MAX31760_CTRL1_PWM_POL_POS)
        self.bus.write8(self.MAX31760_CR2, self.MAX31760_CTRL2_NORMAL_MODE | self.MAX31760_CTRL2_ALERT_COMP | self.MAX31760_CTRL2_FF_OUTPUT_COMP | self.MAX31760_CTRL2_DIRECT_FAN_CTRL_EN)
        self.bus.write8(self.MAX31760_CR3, self.MAX31760_CTRL3_CLR_FAIL | self.MAX31760_CTRL3_PWM_RAMP_RATE_FAST | self.MAX31760_CTRL3_TACH1_EN)
        return True

    # Clear fan fail status bits 
    def clearFF(self):
        cr3_val = self.bus.readU8(self.MAX31760_CR3)
        self.bus.write8(self.MAX31760_CR3, cr3_val | self.MAX31760_CTRL3_CLR_FAIL)
        return True

    # Send software Power-On Reset command
    def sendPOR(self):
        # We don't need to read CR1 and AND as POR will wipe everything else out
        self.bus.write8(self.MAX31760_CR1, self.MAX31760_CTRL1_SW_POR)
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

