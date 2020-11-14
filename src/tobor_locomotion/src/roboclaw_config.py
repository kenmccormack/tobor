#!/usr/bin/env python
import rospy
import roboclaw_driver.roboclaw_driver as rc

MTR_ADDRESS = 128 







def config():  

    rc.Open('/dev/ttyUSB1',115200)
    print rc.ReadMinMaxMainVoltages(MTR_ADDRESS)
    rc.SetPinFunctions(MTR_ADDRESS,2,2,2)
    aa = rc.ReadPinFunctions(MTR_ADDRESS)

    print aa
    rc.WriteNVM(MTR_ADDRESS)
  


if __name__ == '__main__':
    config()
