import serial
import typing
import numpy as np
from ForceWindow import ForceWindow

def parseData(data: list[int]):

    sensorData = np.zeros((4, 5))
    for sensorindex in range(0, 4):
        for i in range(0, 5):
            signal = to_int16(data[sensorindex * 10 + 2 * i], data[sensorindex * 10 + 2 * i + 1])
            sensorData[sensorindex, i] = signal

    return sensorData

def calculateForce(singleSensorData):
    Fx = singleSensorData[0] - singleSensorData[2]
    Fy = singleSensorData[3] - singleSensorData[1]
    Fz = (singleSensorData[0] + singleSensorData[1] + singleSensorData[2] + singleSensorData[3]) / 4

    return np.array([round(Fx), round(Fy), round(Fz)])

def to_int16(i1: int, i2: int) -> int:
    return int(np.binary_repr(i1, 8) + np.binary_repr(i2, 8), 2)

def startDataTransfer(forceWindow: ForceWindow, isOpen):

    try:
        ser = serial.Serial("COM8", 115200)
        isOpen[0] = True
    except serial.SerialException:
        isOpen[0] = False
        return

    while True:
        i1 = int.from_bytes(ser.read())
        i2 = int.from_bytes(ser.read())

        if i1 == 55 and i2 == 94:

            dataList = []
            while True:
                data = int.from_bytes(ser.read())
                if data == 55:
                    data2 = int.from_bytes(ser.read())

                    if data2 == 94:
                        sensorData = parseData(dataList[1:])

                        forceWindow.addDataPoint(calculateForce(sensorData[0]), 0)
                        forceWindow.addDataPoint(calculateForce(sensorData[1]), 1)

                        dataList = []
                    else:
                        dataList.append(data)
                        dataList.append(data2)
                else:
                    dataList.append(data)
