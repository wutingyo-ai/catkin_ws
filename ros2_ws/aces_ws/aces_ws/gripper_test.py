import minimalmodbus
import time
import ROS2_gripper as rq
#!/usr/bin/env python3

instrument = minimalmodbus.Instrument('/dev/ttyUSB0', 9, debug = False)
instrument.serial.baudrate = 115200 
instrument.debug = False  # 禁用 debug 輸出
myGripper = rq.RobotiqGripper(portname='/dev/ttyUSB0',slaveaddress=9)
print("Initializing gripper...")

myGripper.resetActivate()

myGripper.calibrate(0, 255)
myGripper.goTomm(255, 128, 255)   #行程 速度 力量