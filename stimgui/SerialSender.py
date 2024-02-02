import typing

import serial
import PySimpleGUI as sg

RX_SIZE = 7


def parseInput(inputString: str, allowZero: bool = True) -> str:
    if not inputString.isdigit() or len(inputString) > 12 or (int(inputString) == 0 and allowZero):
        return ""

    inputString = str(int(inputString))

    if len(inputString) <= 3:
        return "0" * (3 - len(inputString)) + inputString + "0"
    result = inputString[0:3] + str(len(inputString) - 3)
    return result


class SerialSender:


    def __init__(self):
        self.ser = serial.Serial()

    def connect(self, port):
        try:
            self.ser.close()
            self.ser = serial.Serial(port, baudrate=115200, timeout=2)
            sg.popup("Connected", keep_on_top=True, no_titlebar=True, grab_anywhere=True)
        except serial.SerialException:
            sg.popup("No device connected", keep_on_top=True, no_titlebar=True, grab_anywhere=True)


    # parses command based on programCode, does not pad the command to be RX_SIZE in length
    def parseCommand(self, programCode: str, *args) -> bytes:

        match programCode:
            case 'system reset':
                return bytes([0x00])

            case  'stop stimulation':
                return bytes([0x01])

            case 'stimulate once':
                return bytes([0x02])

            case 'stimulate continuously':
                return bytes([0x02, 0xff])

            case 'intensity':
                raise Exception('not implemented')

            case 'add channel':
                index = args[0][0]
                channelData = args[0][1]

                return bytes([0x0e, index, int(channelData[2]), int(channelData[0], 16) >> 8, int(channelData[0], 16) & 0xff, int(channelData[1])])

            case 'valid channels':

                return bytes([0x0f, args[0][0]])

            case 'command line instruction':

                data = args[0].split(' ')


                for index, i in data:
                    if 'x' in i and i[0] != '0':
                        data[index] = int('0' + data[index], 16)
                    else:
                        data[index] = int(data)

                    if data[index] > 255 or data[index] < 0:
                        raise ValueError('invalid data')

                return bytes(data)

            case 'AWIDTH':
                value = int(args[0][0])
                return bytes([0x05, (value & 0xffff) >> 8, value & 0xff])

            case 'BWIDTH':
                value = int(args[0][0])
                return bytes([0x06, (value & 0xffff) >> 8, value & 0xff])

            case 'CWIDTH':
                value = int(args[0][0])
                return bytes([0x07, (value & 0xffff) >> 8, value & 0xff])

            case 'DWIDTH':
                value = int(args[0][0])
                return bytes([0x08, (value & 0xffff) >> 8, value & 0xff])

            case 'NBURST':
                value = int(args[0][0])
                return bytes([0x09, (value & 0xffff) >> 8, value & 0xff])

            case 'PAUSETIME':

                value = round(int(args[0][0]) / int(args[0][1]))
                print(value)
                return bytes([0x0b, (value & 0xffff) >> 8, value & 0xff])

            case "PWM":
                value = parseInput(args[0][0])
                return bytes([0x0c]) + value.encode()


            case _:
                    raise NameError('not implemented')




    def send(self, command, *data) -> None:

        try:
            command = self.parseCommand(command, data)
        except ValueError:
            sg.popup("could not send: invalid data", keep_on_top=True, no_titlebar=True, grab_anywhere=True)
            return
        except NameError:
            sg.popup("could not send: not implemented", keep_on_top=True, no_titlebar=True, grab_anywhere=True)
            return

        #pad command with zeros until it is RX_SIZE - 1
        command = bytes(list(command) + [0x0] * (RX_SIZE - len(command) - 1))

        if len(command) > RX_SIZE - 1:
            raise Exception('command length exceeds max length')

        try:
            self.ser.write(self.appendCRC(command))

            print('command length: ', len(self.appendCRC(command)))
        except serial.SerialException:
            sg.popup("could not send: serial error", keep_on_top=True, no_titlebar=True, grab_anywhere=True)



    def appendCRC(self, data: bytes) -> bytes:

        data = list(data)
        crc = sum(data) % 256
        data.append(crc)
        return bytes(data)


    def readAll(self):
        try:
            result = self.ser.readall()
        except serial.SerialException:
            sg.popup("could not read")
            return 'could not read'
        return result

