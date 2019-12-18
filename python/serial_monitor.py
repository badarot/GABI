#!/usr/bin/env python

from threading import Thread
import serial
import time
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from glob import glob
import pandas as pd

ROBOT_NAME = 'GABI'


class serialPlot:
    def __init__(self, serialPort='/dev/ttyUSB0', serialBaud=115200, plotLength=100, nparams=1):
        self.port = serialPort
        self.baud = serialBaud
        self.plotMaxLength = plotLength
        # self.rawData = bytearray(dataNumBytes)
        self.nparams = nparams
        self.rawData = b''
        self.data = [collections.deque([0] * plotLength, maxlen=plotLength) for _ in range(nparams)]
        self.isRun = True
        self.isReceiving = False
        self.serialThread = None
        self.inputThread = None
        self.plotTimer = 0
        self.previousTimer = 0
        # self.csvData = []

        print('Trying to connect to: ' + str(serialPort) +
              ' at ' + str(serialBaud) + ' BAUD.')
        try:
            self.serialConnection = serial.Serial(
                serialPort, serialBaud, timeout=4)
            print('Connected to ' + str(serialPort) +
                  ' at ' + str(serialBaud) + ' BAUD.')
        except:
            print("Failed to connect with " + str(serialPort) +
                  ' at ' + str(serialBaud) + ' BAUD.')

    def threadStart(self):
        if self.serialThread is None:
            self.serialThread = Thread(target=self.readSerial)
            self.serialThread.start()
            # Block till we start receiving values
            while self.isReceiving != True:
                time.sleep(0.1)

        if self.inputThread is None:
            self.serialThread = Thread(target=self.sendSerial)
            self.serialThread.daemon = True
            self.serialThread.start()

    def getSerialData(self, frame, lines, lineValueText, parameters, timeText):
        if not self.isReceiving:
            return

        self.isReceiving = False

        currentTimer = time.perf_counter()
        # the first reading will be erroneous
        self.plotTimer = int((currentTimer - self.previousTimer) * 1000)
        self.previousTimer = currentTimer
        timeText.set_text('Plot Interval = ' + str(self.plotTimer) + 'ms')

        for l, d in zip(lines, self.data):
            l.set_data(range(self.plotMaxLength), d)
        lineValueText.set_text('[' + parameters[0] + '] = ' + str(self.data[0][-1]))
        # self.csvData.append(self.data[-1])

    def readSerial(self):       # retrieve data
        time.sleep(0.5)         # give some buffer time for retrieving data
        self.serialConnection.reset_input_buffer()
        while (self.isRun):
            # self.serialConnection.readinto(self.rawData)
            self.rawData = self.serialConnection.readline()
            # print(self.rawData)

            # use 'h' for a 2 byte integer
            # value,  = struct.unpack('f', self.rawData)
            try:
                for value, d in zip(self.rawData.split()[:self.nparams], self.data):
                    v = float(value)
                    d.append(v)
                self.isReceiving = True # Sinaliza que ha dados prontos
                # print(value)

            except ValueError:
                print(ROBOT_NAME, end=': ')
                try:
                    print(self.rawData.decode(), end='')
                except Exception:
                    print(self.rawData)

            except Exception as e:
                print('{}: {}'.format(e, self.rawData))
                # pass

    def sendSerial(self):
        while self.isRun:
            time.sleep(0.5)
            cmd = input() + '\n'
            self.serialConnection.write(cmd.encode('ascii'))

    def close(self):
        self.isRun = False
        self.serialThread.join()
        # self.inputThread.join()
        self.serialConnection.close()
        print('Disconnected...')
        # df = pd.DataFrame(self.csvData)
        # df.to_csv('data.csv')


def main():
    parameters = ['angle', 'targ. angle', 'speed', 'accel']
    param_used = ['angle', 'speed']

    portList = glob('/dev/ttyUSB*')
    portList.sort()
    # portName = 'COM5'     # for windows users
    portName = portList[0]
    baudRate = 115200
    maxPlotLength = 1000

    # initializes all required variables
    s = serialPlot(portName, baudRate, maxPlotLength, nparams=len(parameters))
    # starts background thread
    s.threadStart()

    # plotting starts below
    pltInterval = 50    # Period at which the plot animation updates [ms]
    xmin = 0
    xmax = maxPlotLength
    ymin = -25
    ymax = 25
    fig = plt.figure()
    ax = plt.axes(xlim=(xmin, xmax), ylim=(float(ymin - (ymax - ymin) / 10),
                  float(ymax + (ymax - ymin) / 10)))

    ax.set_title(ROBOT_NAME)
    ax.set_xlabel("Aquisição")
    ax.set_ylabel("Graus")

    # lineLabel = 'Inclicacao'
    timeText = ax.text(0.50, 0.95, '', transform=ax.transAxes)
    lines = []
    for p in parameters:
        if p in param_used:
            alpha = 1
        else:
            alpha = 0
        lines.append(ax.plot([], [], label=p, alpha=alpha)[0])
    # print(lines)
    lineValueText = ax.text(0.50, 0.90, '', transform=ax.transAxes)

    # fargs has to be a tuple
    _ = animation.FuncAnimation(fig, s.getSerialData, fargs=(lines, lineValueText,
                                parameters, timeText), interval=pltInterval)

    plt.legend(loc="upper left")
    #
    plt.show()
    s.close()


if __name__ == '__main__':
    main()
