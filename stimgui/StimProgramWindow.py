import PySimpleGUI as sg
import serial
import time


# convert "12300" to "1234" (123 * 10e4)
def parseInput(inputString: str, allowZero: bool = True) -> str:
    if not inputString.isdigit() or len(inputString) > 12 or (int(inputString) == 0 and allowZero):
        return ""

    inputString = str(int(inputString))

    if len(inputString) <= 3:
        return "0" * (3 - len(inputString)) + inputString + "0"
    result = inputString[0:3] + str(len(inputString) - 3)
    return result


class StimProgramWindow:
    width = 500
    height = 500
    programOrder = []
    window = None

    # def __init__(self, keys):

    def open(self):
        self.layout = [

            [sg.Button("reset", key="-RESET-")],
            [sg.Checkbox(key=f'CHECKBOX{15 - i}', text=f'{15 - i}') for i in range(16)],
            [sg.Text("#of bursts: "),
             sg.Input(key="-CHANNELBURSTNUMBER-", size=(10, 1), default_text='1'), sg.Text(expand_x=True),
             sg.Button(button_text="+", key="-CHANNELINDEXADD-")],
            [
                sg.Multiline(key="PROGRAMCODE", expand_x=True, expand_y=True)
            ],
            [sg.Button(button_text="Set program", key="-SETSTIMPROGRAM-")]
        ]

        self.window = sg.Window("Stimwindow", layout=self.layout)
        self.window.Finalize()
        self.window.set_min_size((self.width, self.height))

    def setProgram(self, nucleo: serial.Serial):
        if len(self.programOrder) > 30:
            raise Exception("Program too long!")

        try:
            for index, i in enumerate(self.programOrder):
                commandIndex = '{:0>2}'.format(index)
#                channelIndex = '{:0>2}'.format(i[0])
                channelBurstNumber = '{:0>2}'.format(i[1])

                # if len(commandIndex) != 2 or len(channelIndex) != 2 or len(channelBurstNumber) != 2:
                #     raise Exception(f'Error while creating command for element {i} in list')


                nucleo.write('s'.encode() + bytes([index]) + bytes([int(i[2])]) + bytes([int(i[0], 16) >> 8]) + bytes([int(i[0], 16) & 0xff]) +
                             channelBurstNumber.encode())

                print(len('s'.encode() + bytes([index]) + bytes([int(i[2])]) + bytes([int(i[0], 16) >> 8]) + bytes([int(i[0], 16) & 0xff]) +
                             channelBurstNumber.encode()))
                #channelIndex = int(i[0], 16)

                # for j in range(16):
                #     if (channelIndex >> j) % 2 == 1:
                #         command = 'V0' + '{:0>3}'.format(i[2]) + '{:0>2}'.format(j)
                #         print(command)
                #         nucleo.write(command.encode())
                #         time.sleep(0.01)
                # nucleo.write(command.encode())

            time.sleep(0.5)
            command = 'v' + parseInput(inputString=str(len(self.programOrder)), allowZero=False) + '00'
            print(command)
            nucleo.write(command.encode())
        except serial.SerialException:
            sg.popup("No device connected", keep_on_top=True, no_titlebar=True, grab_anywhere=True)
        except:
            sg.popup("Error while sending command", keep_on_top=True, no_titlebar=True, grab_anywhere=True)

    def addToProgram(self, burst: str, values: dict):

        if not burst.isdigit():
            sg.popup("Wrong format!", keep_on_top=True, no_titlebar=True, grab_anywhere=True)
        else:

            sum = 0
            for i in range(16):
                sum += pow(2, i) * values[f'CHECKBOX{i}']

            channelsToOpen = '0x' + '{:0>2}'.format(hex(bytes([sum >> 8])[0])[2:]) + '{:0>2}'.format(hex(bytes([sum & 0xff])[0])[2:])
            self.window['PROGRAMCODE'].update(channelsToOpen + ' ' + burst + ' 127\n', append=True)

    def removeFromProgram(self, index):
        self.programOrder.pop(index)

    def resetProgram(self):
        self.programOrder.clear()

    def getDataFromText(self, text: str):

        self.programOrder.clear()
        text = text.split('\n')
        for i in text:
            i = i.split(' ')

            self.programOrder.append((i[0], i[1], i[2]))

    def close(self):
        self.window.close()
