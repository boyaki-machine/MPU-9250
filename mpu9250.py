#!/usr/bin/python -u
# -*- coding: utf-8 -*-

import smbus
import time

# A class (python2) that acquires data from I2C from Strawberry Linux's "MPU-9250"
# https://strawberry-linux.com/catalog/items?code=12250
#
# 2016-05-03 Boyaki Machine
class SL_MPU9250:
    # Constant definition
    REG_PWR_MGMT_1      = 0x6B
    REG_INT_PIN_CFG     = 0x37
    REG_GYRO_CONFIG     = 0x1B
    REG_ACCEL_CONFIG1   = 0x1C
    REG_ACCEL_CONFIG2   = 0x1D

    MAG_MODE_POWERDOWN  = 0         # Magnetometric sensor power down
    MAG_MODE_SERIAL_1   = 1         # Magnetometric sensor 8Hz Continuous measurement mode
    MAG_MODE_SERIAL_2   = 2         # Magnetometric sensor 100Hz Continuous measurement mode
    MAG_MODE_SINGLE     = 3         # Magnetometric sensor Single measurement mode
    MAG_MODE_EX_TRIGER  = 4         # Magnetometric sensor Trigger measurement mode
    MAG_MODE_SELF_TEST  = 5         # Magnetometric sensor Self test mode

    MAG_ACCESS          = False     # Magnetometric sensor access enable/disable
    MAG_MODE            = 0         # Magnetometric sensor mode
    MAG_BIT             = 14        # Magnetometric sensor output bit number of figures

    offsetRoomTemp      = 0
    tempSensitivity     = 333.87
    gyroRange           = 250       # 'dps' 00:250, 01:500, 10:1000, 11:2000
    accelRange          = 2         # 'g' 00:+-2, 01:+-4, 10:+-8, 11:+-16
    magRange            = 4912      # 'μT'  

    offsetAccelX        = 0.0
    offsetAccelY        = 0.0
    offsetAccelZ        = 0.0
    offsetGyroX         = 0.0
    offsetGyroY         = 0.0
    offsetGyroZ         = 0.0

    # Constructor
    def __init__(self, address, channel):
        self.address    = address
        self.channel    = channel
        self.bus        = smbus.SMBus(self.channel)
        self.addrAK8963 = 0x0C

        # Sensor initialization
        self.resetRegister()
        self.powerWakeUp()

        self.gyroCoefficient    = self.gyroRange  / float(0x8000)   # coefficient : sensed decimal val to dps val.
        self.accelCoefficient   = self.accelRange / float(0x8000)   # coefficient : sensed decimal val to g val.
        self.magCoefficient16   = self.magRange   / 32760.0         # confficient : sensed decimal val to μT val (16bit)
        self.magCoefficient14   = self.magRange   / 8190.0          # confficient : sensed decimal val to μT val (14bit)

    # Function to initialize registers.
    def resetRegister(self):
        if self.MAG_ACCESS == True:
            self.bus.write_i2c_block_data(self.addrAK8963, 0x0B, [0x01])    
        self.bus.write_i2c_block_data(self.address, 0x6B, [0x80])
        self.MAG_ACCESS = False
        time.sleep(0.1)     

    # Function to set registers as sensing enable.
    def powerWakeUp(self):
        # PWR_MGMT_1 clear.
        self.bus.write_i2c_block_data(self.address, self.REG_PWR_MGMT_1, [0x00])
        time.sleep(0.1)
        # Make the magnetic sensor function (AK 8963) accessible by I2C(BYPASS_EN=1)
        self.bus.write_i2c_block_data(self.address, self.REG_INT_PIN_CFG, [0x02])
        self.MAG_ACCESS = True
        time.sleep(0.1)

    # Function to set magnetometer sensor register.
    def setMagRegister(self, _mode, _bit):      
        if self.MAG_ACCESS == False:
            # Raise an exception because access to the magnetic sensor is not enabled.
            raise Exception('001 Access to a sensor is invalid.')

        _writeData  = 0x00
        # Setting measurement mode
        if _mode=='8Hz':            # Continuous measurement mode 1
            _writeData      = 0x02
            self.MAG_MODE   = self.MAG_MODE_SERIAL_1
        elif _mode=='100Hz':        # Continuous measurement mode 2
            _writeData      = 0x06
            self.MAG_MODE   = self.MAG_MODE_SERIAL_2
        elif _mode=='POWER_DOWN':   # Power down mode
            _writeData      = 0x00
            self.MAG_MODE   = self.MAG_MODE_POWERDOWN
        elif _mode=='EX_TRIGER':    # Trigger measurement mode
            _writeData      = 0x04
            self.MAG_MODE   = self.MAG_MODE_EX_TRIGER
        elif _mode=='SELF_TEST':    # self test mode
            _writeData      = 0x08
            self.MAG_MODE   = self.MAG_MODE_SELF_TEST
        else:   # _mode='SINGLE'    # single measurment mode
            _writeData      = 0x01
            self.MAG_MODE   = self.MAG_MODE_SINGLE

        #　output bit number 
        if _bit=='14bit':           # output 14bit
            _writeData      = _writeData | 0x00
            self.MAG_BIT    = 14
        else:   # _bit='16bit'      # output 16bit
            _writeData      = _writeData | 0x10
            self.MAG_BIT    = 16

        self.bus.write_i2c_block_data(self.addrAK8963, 0x0A, [_writeData])

    # Function to set measurement range of acceleration.Measurement granularity becomes rough in wide range.
    # val = 16, 8, 4, 2(default)
    def setAccelRange(self, val, _calibration=False):
        # +-2g (00), +-4g (01), +-8g (10), +-16g (11)
        if val==16 :
            self.accelRange     = 16
            _data               = 0x18
        elif val==8 :
            self.accelRange     = 8
            _data               = 0x10
        elif val==4 :
            self.accelRange     = 4
            _data               = 0x08
        else:
            self.accelRange     = 2
            _data               = 0x00

        self.bus.write_i2c_block_data(self.address, self.REG_ACCEL_CONFIG1, [_data])
        self.accelCoefficient   = self.accelRange / float(0x8000)
        time.sleep(0.1)

        # Reset offset value (so that the past offset value is not inherited)
        self.offsetAccelX       = 0
        self.offsetAccelY       = 0
        self.offsetAccelZ       = 0

        # In fact I think that it is better to calibrate. But it took time so gave up.
        if _calibration == True:
            self.calibAccel(1000)
        return

    # Function to set measurement range of gyro. Measurement granularity becomes rough in wide range.
    # val= 2000, 1000, 500, 250(default)
    def setGyroRange(self, val, _calibration=False):
        if val==2000:
            self.gyroRange      = 2000
            _data               = 0x18
        elif val==1000:
            self.gyroRange      = 1000
            _data               = 0x10
        elif val==500:
            self.gyroRange      = 500
            _data               = 0x08
        else:
            self.gyroRange      = 250
            _data               = 0x00

        self.bus.write_i2c_block_data(self.address, self.REG_GYRO_CONFIG, [_data])
        self.gyroCoefficient    = self.gyroRange / float(0x8000)
        time.sleep(0.1)

        # Reset offset value (so that the past offset value is not inherited)
        self.offsetGyroX        = 0
        self.offsetGyroY        = 0
        self.offsetGyroZ        = 0

        # In fact I think that it is better to calibrate. But it took time so gave up.
        if _calibration == True:
            self.calibGyro(1000)
        return

    # Function to set LowPassFilter of acceleration sensor.
    # def setAccelLowPassFilter(self,val):      

    #The data from the sensor is treated as unsigned. So, converted to signed. (limited to 16 bits)
    def u2s(self,unsigneddata):
        if unsigneddata & (0x01 << 15) : 
            return -1 * ((unsigneddata ^ 0xffff) + 1)
        return unsigneddata

    # Function to obtain acceleration value.
    def getAccel(self):
        data    = self.bus.read_i2c_block_data(self.address, 0x3B ,6)
        rawX    = self.accelCoefficient * self.u2s(data[0] << 8 | data[1]) + self.offsetAccelX
        rawY    = self.accelCoefficient * self.u2s(data[2] << 8 | data[3]) + self.offsetAccelY
        rawZ    = self.accelCoefficient * self.u2s(data[4] << 8 | data[5]) + self.offsetAccelZ
        return rawX, rawY, rawZ

    # Function to obtain gyro value.
    def getGyro(self):
        data    = self.bus.read_i2c_block_data(self.address, 0x43 ,6)
        rawX    = self.gyroCoefficient * self.u2s(data[0] << 8 | data[1]) + self.offsetGyroX
        rawY    = self.gyroCoefficient * self.u2s(data[2] << 8 | data[3]) + self.offsetGyroY
        rawZ    = self.gyroCoefficient * self.u2s(data[4] << 8 | data[5]) + self.offsetGyroZ
        return rawX, rawY, rawZ

    def getMag(self):
        if self.MAG_ACCESS == False:
            # Magnetometric sensor is disable.
            raise Exception('002 Access to a sensor is invalid.')

        # Pre-processing
        if self.MAG_MODE==self.MAG_MODE_SINGLE:
            # In the case of single measurement mode, it becomes Power Down simultaneously with the end of measurement. so, change the mode again.
            if self.MAG_BIT==14:                # output 14bit
                _writeData      = 0x01
            else:                               # output 16bit
                _writeData      = 0x11
            self.bus.write_i2c_block_data(self.addrAK8963, 0x0A, [_writeData])
            time.sleep(0.01)

        elif self.MAG_MODE==self.MAG_MODE_SERIAL_1 or self.MAG_MODE==self.MAG_MODE_SERIAL_2:
            status  = self.bus.read_i2c_block_data(self.addrAK8963, 0x02 ,1)
            if (status[0] & 0x02) == 0x02:
                # Sensing again as there is data overrun.
                self.bus.read_i2c_block_data(self.addrAK8963, 0x09 ,1)

        elif self.MAG_MODE==self.MAG_MODE_EX_TRIGER:
            # Unimplemented
            return

        elif self.MAG_MODE==self.MAG_MODE_POWERDOWN:
            raise Exception('003 Mag sensor power down')

        # Check the ST1 register. Check whether data can be read.
        status  = self.bus.read_i2c_block_data(self.addrAK8963, 0x02 ,1)
        while (status[0] & 0x01) != 0x01:
            # Wait until data ready state.
            time.sleep(0.01)
            status  = self.bus.read_i2c_block_data(self.addrAK8963, 0x02 ,1)

        # read data.
        data    = self.bus.read_i2c_block_data(self.addrAK8963, 0x03 ,7)
        rawX    = self.u2s(data[1] << 8 | data[0])  # Lower bit is ahead.
        rawY    = self.u2s(data[3] << 8 | data[2])  # Lower bit is ahead.
        rawZ    = self.u2s(data[5] << 8 | data[4])  # Lower bit is ahead.
        st2     = data[6]

        # check overflow.
        if (st2 & 0x08) == 0x08:
            # Since it is an overflow, the correct value is not obtained
            raise Exception('004 Mag sensor over flow')

        # Conversion to μT
        if self.MAG_BIT==16:    # output 16bit
            rawX    = rawX * self.magCoefficient16
            rawY    = rawY * self.magCoefficient16
            rawZ    = rawZ * self.magCoefficient16
        else:                   # output 14bit
            rawX    = rawX * self.magCoefficient14
            rawY    = rawY * self.magCoefficient14
            rawZ    = rawZ * self.magCoefficient14

        return rawX, rawY, rawZ

    def getTemp(self):
        data    = self.bus.read_i2c_block_data(self.address, 0x65 ,2)
        raw     = data[0] << 8 | data[1]
        return ((raw - self.offsetRoomTemp) / self.tempSensitivity) + 21

    def selfTestMag(self):
        print "start mag sensor self test"
        self.setMagRegister('SELF_TEST','16bit')
        self.bus.write_i2c_block_data(self.addrAK8963, 0x0C, [0x40])
        time.sleep(1.0)
        data = self.getMag()

        print data

        self.bus.write_i2c_block_data(self.addrAK8963, 0x0C, [0x00])
        self.setMagRegister('POWER_DOWN','16bit')
        time.sleep(1.0)
        print "end mag sensor self test"
        return

    # Calibrate the acceleration sensor
    # I think that you really need to consider latitude, altitude, terrain, etc., but I briefly thought about it.
    # It is a premise that gravity is correctly applied in the direction of the z axis and acceleration other than gravity is not generated.
    def calibAccel(self, _count=1000):
        print "Accel calibration start"
        _sum    = [0,0,0]

        # get data sample.
        for _i in range(_count):
            _data   = self.getAccel()
            _sum[0] += _data[0]
            _sum[1] += _data[1]
            _sum[2] += _data[2]

        # Make the average value an offset.
        self.offsetAccelX   = -1.0 * _sum[0] / _count
        self.offsetAccelY   = -1.0 * _sum[1] / _count
        self.offsetAccelZ   = -1.0 * ((_sum[2] / _count ) - 1.0)    # 重力分を差し引く

        # I want to register an offset value in a register. But I do not know the behavior, so I will put it on hold.

        print "Accel calibration complete"
        return self.offsetAccelX, self.offsetAccelY, self.offsetAccelZ

    # Calibrate the gyro sensor
    # Assumption that no rotation occurs on each axis
    def calibGyro(self, _count=1000):
        print "Gyro calibration start"
        _sum    = [0,0,0]

        # get data sample
        for _i in range(_count):
            _data   = self.getGyro()
            _sum[0] += _data[0]
            _sum[1] += _data[1]
            _sum[2] += _data[2]

        # Make the average value an offset.
        self.offsetGyroX    = -1.0 * _sum[0] / _count
        self.offsetGyroY    = -1.0 * _sum[1] / _count
        self.offsetGyroZ    = -1.0 * _sum[2] / _count

        # I want to register an offset value in a register. But I do not know the behavior, so I will put it on hold.

        print "Gyro calibration complete"
        return self.offsetGyroX, self.offsetGyroY, self.offsetGyroZ


if __name__ == "__main__":
    sensor  = SL_MPU9250(0x68,1)
    sensor.resetRegister()
    sensor.powerWakeUp()
    sensor.setAccelRange(8,True)
    sensor.setGyroRange(1000,True)
    sensor.setMagRegister('100Hz','16bit')
    # sensor.selfTestMag()

    while True:
        now     = time.time()
        acc     = sensor.getAccel()
        gyr     = sensor.getGyro()
        mag     = sensor.getMag()
        print "%+8.7f" % acc[0] + " ",
        print "%+8.7f" % acc[1] + " ",
        print "%+8.7f" % acc[2] + " ",
        print " |   ",
        print "%+8.7f" % gyr[0] + " ",
        print "%+8.7f" % gyr[1] + " ",
        print "%+8.7f" % gyr[2] + " ",
        print " |   ",
        print "%+8.7f" % mag[0] + " ",
        print "%+8.7f" % mag[1] + " ",
        print "%+8.7f" % mag[2]

        sleepTime       = 0.1 - (time.time() - now)
        if sleepTime < 0.0:
            continue
        time.sleep(sleepTime)
