import PySimpleGUI as sg
import numpy as np
import threading
import math
import serial

class ForceWindow:
    width = 1000
    height = 500
    calibrationStatus = threading.Event()
    calibrationPoints = 100
    currentCalibrationPoint = 0
    calibrationData = np.zeros((2, 3, calibrationPoints))
    calibratedForces = np.zeros((2, 3))

    dataPoints = 100
    currentDataPoints = 0
    forceData = np.zeros((2, 3, dataPoints))
    rectSize = 10
    window = None
    nucleo = None

    n = 0


    def addDataPoint(self, data: np.array, sensorIndex: int):
        if self.calibrationStatus.is_set():
            self.addCalibrationPoint(sensorIndex, self.currentCalibrationPoint, data)
        else:
            self.forceData[sensorIndex, :, self.currentDataPoints] = data - self.calibratedForces[sensorIndex]

            if self.n % 10 == 0 and sensorIndex == 1:
                currentForceValue = self.forceData[1, :, self.currentDataPoints]
                self.displayForce(1, 0, currentForceValue)

                sg.popup("Error, check ForceWindow.py")
                # if currentForceValue[1] > 100:
                #     self.nucleo.write('V003000'.encode())
                # elif currentForceValue[1] > 50:
                #     self.nucleo.write('V002000'.encode())
                # elif currentForceValue[1] > 10:
                #     self.nucleo.write('V001000'.encode())
                # else:
                #     self.nucleo.write('V000000'.encode())


            if self.n % 10 == 0 and sensorIndex == 0:
                currentForceValue = self.forceData[0, :, self.currentDataPoints]
                self.displayForce(0, 0, currentForceValue)

                # if currentForceValue[1] > 100:
                #     self.nucleo.write('V003004'.encode())
                # elif currentForceValue[1] > 50:
                #     self.nucleo.write('V002004'.encode())
                # elif currentForceValue[1] > 10:
                #     self.nucleo.write('V001004'.encode())
                # else:
                #     self.nucleo.write('V000004'.encode())


            self.n += sensorIndex
            self.currentDataPoints += sensorIndex
            if self.currentDataPoints >= self.dataPoints:
                self.currentDataPoints = 0

    def startCalibration(self):
        self.calibrationData = np.zeros((2, 3, self.calibrationPoints))
        self.currentCalibrationPoint = 0
        self.calibrationStatus.set()

    def addCalibrationPoint(self, sensorIndex: int, pointIndex: int, data: np.array):
        self.calibrationData[sensorIndex, :, pointIndex] = data
        self.currentCalibrationPoint += sensorIndex

        if self.currentCalibrationPoint >= self.calibrationPoints:
            self.finishCalibration()

    def finishCalibration(self):
        self.calibrationStatus.clear()

        for sensorIndex in range(0, 2):
            for forceIndex in range(0, 3):
                self.calibratedForces[sensorIndex, forceIndex] = round(np.mean(
                    self.calibrationData[sensorIndex, forceIndex, :]))


    def open(self, nucleo: serial.Serial):
        self.layout = [[sg.Button("Calibrate", key="STARTCALIBRATION")],
                       [sg.Text("", key="FORCE1")],
                       [sg.Text("", key="FORCE2")]]

        self.window = sg.Window("Stimwindow", layout=self.layout)
        self.window.Finalize()
        self.window.set_min_size((300, 300))
        self.nucleo = nucleo


    def close(self):
        self.window.close()

    def drawRect(self, pos: tuple[int, int], color: str):
        self.window["GRAPH"].DrawRectangle(pos, (pos[0] + self.rectSize, pos[1] - self.rectSize), fill_color=color,
                                           line_color="orange", line_width=5)

    def displayForce(self, sensorIndex: int, forceIndex: int, forceValue):

        self.window[f"FORCE{sensorIndex + 1}"].update(forceValue)


        # forceValue = (forceValue) / 1000
        #
        # for i in range(0, 10):
        #     if forceValue < i * self.rectSize:
        #         self.drawRect((sensorIndex * 100 + forceIndex * 30, self.height - i * self.rectSize), "gray")
        #     else:
        #         self.drawRect((sensorIndex * 100 + forceIndex * 30, self.height - i * self.rectSize), "green")
