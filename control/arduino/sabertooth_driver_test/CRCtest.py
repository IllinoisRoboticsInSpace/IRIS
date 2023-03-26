import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import Joy

import serial



    ####################################################################################################

    #IMPORTANT FUNCTIONS:
def intToBytes(y):
    return y.to_bytes((y.bit_length()+7)//8, 'big')
def CRC_m(numMotor, power):      #converts self.power into bytes
    def largestOneIndex(curr:int):
        i = 15
        while (i >= 0):
            if (curr & (1 << i)):
                break
            i-=1
        return i
    x = abs(power) + 256 * numMotor
    #x is two bits, power must be between -127 and 127. probably.
    #first byte should be # of motor + CRC
    #second byte is power (most significant bit is negative sign)
    if(power < 0):
        x += 128
    CRC = 18
    msg = x % 2048
    checksum = int(x / 2048)
    curr = msg*32 + checksum
    a = 0
    index = 0
    while (curr >= 32):
        index = largestOneIndex(curr)
        a = CRC << (index - 4)
        curr ^= a
    return curr
def CRC(x):      #converts self.power into bytes
    def largestOneIndex(curr:int):
        i = 15
        while (i >= 0):
            if (curr & (1 << i)):
                break
            i-=1
        return i
    CRC = 18
    msg = x % 2048
    checksum = int(x / 2048)
    curr = msg*32 + checksum
    a = 0
    index = 0
    while (curr >= 32):
        index = largestOneIndex(curr)
        a = CRC << (index - 4)
        curr ^= a
    return curr
def attachCRC(numMotor, power):
    curr = CRC_m(numMotor, power)
    x = abs(power) + 256 * numMotor
    if(power < 0):
        x += 128
    return curr * 256 * 8 + x 
motor = 1
power = 30

x = abs(power) + 256 * motor
if(power < 0):
    x += 128
print(str(x) + ' | ' + intToBytes(x).hex())
print(str(CRC_m(motor, power)) + ' | ' + intToBytes(CRC_m(motor,power)).hex())
x = attachCRC(motor, power)
print(str(x) +  ' | ' + intToBytes(x).hex())
print(str(CRC(x)) + ' | ' + intToBytes(CRC(x)).hex())

motor = 1
power = 24

x = abs(power) + 256 * motor
if(power < 0):
    x += 128
print(str(x) + ' | ' + intToBytes(x).hex())
print(str(CRC_m(motor, power)) + ' | ' + intToBytes(CRC_m(motor,power)).hex())
x = attachCRC(motor, power)
print(str(x) +  ' | ' + intToBytes(x).hex())
print(str(CRC(x)) + ' | ' + intToBytes(CRC(x)).hex())

motor = 1
power = 0

x = abs(power) + 256 * motor
if(power < 0):
    x += 128
print(str(x) + ' | ' + intToBytes(x).hex())
print(str(CRC_m(motor, power)) + ' | ' + intToBytes(CRC_m(motor,power)).hex())
x = attachCRC(motor, power)
print(str(x) +  ' | ' + intToBytes(x).hex())
print(str(CRC(x)) + ' | ' + intToBytes(CRC(x)).hex())