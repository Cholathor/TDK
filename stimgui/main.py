import time

import PySimpleGUI as sg
import serial
from Stimwindow import StimWindow
import GripperControl
from ForceWindow import ForceWindow
import optoForce
import threading
from DataReceiver import DataReceiver
from StimProgramWindow import StimProgramWindow
from StimProgramWindow import parseInput

# convert "1234" to 123 * 10e4
def convertToNumber(inputString: str) -> int:
    return int(inputString[0:3]) * (10 ** int(inputString[3]))


def change_focus(event):
    event.widget.focus_set()


class Application:
    keys = "qwertzuiasdfghjk"
    currentChannel = ''
    stimWindow = StimWindow(keys)
    forceWindow = ForceWindow()
    dataReceiver = DataReceiver()
    stimProgramWindow = StimProgramWindow()

    def __init__(self):

        self.gripper = GripperControl.GripperControl()
        self.gripper.connect()
        self.parameters = {"AWIDTH": 250, "BWIDTH": 250, "CWIDTH": 250, "DWIDTH": 2500, "NBURST": 10, "mWIDTH": 125000,
                           "PWM": 30, "OFFSET": 1550}

        self.nucleo = serial.Serial()
        try:
            self.nucleo = serial.Serial('COM4', 115200)
        except serial.SerialException:
            sg.popup("No device connected", keep_on_top=True, no_titlebar=True, grab_anywhere=True)

    def make_window(self, theme):
        sg.theme(theme)

        # menu_def = [['Device', ['Connect']]]
        right_click_menu_def = [[], ['Edit Me', 'Versions', 'Nothing', 'More Nothing', 'Exit']]

        # layout = [[sg.MenubarCustom(menu_def, key='-MENU-', font='Courier 15', tearoff=False, background_color='#fff')],
        #         [sg.Text('StimGUI', size=(20, 1), justification='center', font=("Courier 15", 16),
        #                 relief=sg.RELIEF_RIDGE, k='-TEXT HEADING-', enable_events=True)]]

        layout = self.getLayouts()

        layout[-1].append(sg.Sizegrip())
        window = sg.Window('StimGUI', layout, right_click_menu=right_click_menu_def,
                           right_click_menu_tearoff=True, grab_anywhere=False, resizable=True, margins=(0, 0),
                           use_custom_titlebar=False, finalize=True, keep_on_top=False, icon='icon.ico',
                           return_keyboard_events=True)

        window.set_min_size(window.size)

        return window

    def getLayouts(self):
        """
        To add a layout, define it as a list, and add elements to it. This list will be drawn vertically. To add
        elements horizontally, you can add them in a list (inside the list)
        :return: inner layout of the application
        """

        parameter_layout = [
            [sg.Image(source="waveformv2.png"), sg.Slider(range=(6, 127), default_value=0, resolution=1,
                                                          orientation='vertical', enable_events=True,
                                                          disable_number_display=False, key='SLIDER', expand_y=True)],

            [sg.Text("A"), sg.Input(size=(5, 1), key="AWIDTH", default_text="250"), sg.Text("[µs]"),
             sg.Text("B"), sg.Input(size=(5, 1), key="BWIDTH", default_text="250"), sg.Text("[µs]"),
             sg.Text("C"), sg.Input(size=(5, 1), key="CWIDTH", default_text="250"), sg.Text("[µs]"),
             sg.Text("D"), sg.Input(size=(5, 1), key="DWIDTH", default_text="2500"), sg.Text("[µs]"),
             sg.Text(expand_x=True), sg.Button("Default", key="DEFAULT")],

            [sg.Text("Number of bursts"), sg.Input(size=(5, 1), key="NBURST", default_text="10")],

            [sg.Text("Time between bursts"),
             sg.Input(size=(10, 1), key="mWIDTH", default_text="125000"), sg.Text("[µs]")],

            [sg.Text("PWM duty cycle"), sg.Input(size=(5, 1), key="PWM", default_text="30"), sg.Text("%")],

            [sg.Text("Amplitude"), sg.Input(size=(5, 1), key="VAMPLITUDE", default_text="1500"), sg.Text("mV")],

            [sg.Text("Offset"), sg.Input(size=(5, 1), key="OFFSET", default_text="1550"), sg.Text("mV"),
             sg.Text(expand_x=True), sg.Button("Program", key="-STIMPROGRAMBTN-")],

            [sg.Button("Adjust", key="SETDATA"), sg.Text(expand_x=True),
             sg.Button("Stimulate with keyboard", key="KEYBOARDSTIM", disabled=False), sg.Button("Show forces", key="SHOWFORCES")],

            [sg.Button("Stimulate", key="STIMULATE"), sg.Text(expand_x=True), sg.Button("Connect to nucleo", key="CONNECT")],
        ]

        command_layout = [[sg.Multiline(key="COMMANDLINE", expand_x=True, expand_y=True)],
                          [sg.Button("Send", key="COMMANDSEND")]]

        layout = [[sg.TabGroup([[sg.Tab('Parameter setup', parameter_layout),
                                 ],
                                [sg.Tab('Commandline', command_layout)]
                                ], key='-TAB GROUP-', expand_x=True, expand_y=True),
                   ]]

        return layout

    def main(self):
        s = sg.theme('default1')
        window = self.make_window(s)

        window.TKroot.bind('<Button>', change_focus)

        # for key in self.keys:
        #     window.bind(key, f'KEY_DOWN_{key}')
        # window.bind("<Up>", "UPBUTTON")
        # window.bind("<Down>", "DOWNBUTTON")
        window['SLIDER'].bind('<ButtonRelease-1>', ' Release')

        # This is an Event Loop
        while True:
            event, values = window.read(timeout=100)


            if self.forceWindow.window is not None and not self.forceWindow.window.is_closed():
                forceWindowEvent, _ = self.forceWindow.window.read(timeout=100)

                if forceWindowEvent == "STARTCALIBRATION":
                    self.forceWindow.startCalibration()
                    print("calibration started...")

            if self.stimProgramWindow.window is not None and not self.stimProgramWindow.window.is_closed():
                programWindowEvent, programWindowValues = self.stimProgramWindow.window.read(timeout=100)

                if programWindowEvent == "-SETSTIMPROGRAM-":
                    self.stimProgramWindow.getDataFromText(programWindowValues['PROGRAMCODE'])
                    self.stimProgramWindow.setProgram(self.nucleo)
                elif programWindowEvent == '-CHANNELINDEXADD-':
                    self.stimProgramWindow.addToProgram(programWindowValues['-CHANNELBURSTNUMBER-'], programWindowValues)

            if self.stimWindow.window is not None and not self.stimWindow.window.is_closed():
                keyboardEvent, keyboardValues = self.stimWindow.window.read(timeout=100)

                print(keyboardEvent)
            # keep an animation running so show things are happening

            if event != '__TIMEOUT__':
                print(event)

            if event in (None, 'Exit'):
                print("[LOG] Clicked Exit!")
                break

            elif event == 'SLIDER Release':
                self.nucleo.write(f'R{round(values["SLIDER"]):03}000'.encode())
                print(f'R{round(values["SLIDER"]):03}000')

            elif event == '-STIMPROGRAMBTN-':
                if self.stimProgramWindow.window is not None:
                    self.stimProgramWindow.close()
                self.stimProgramWindow.open()

            elif event == 'CONNECT':
                try:
                    self.nucleo.close()
                    self.nucleo = serial.Serial('COM4', 115200)
                    sg.popup("Connected", keep_on_top=True, no_titlebar=True, grab_anywhere=True)
                except serial.SerialException:
                    sg.popup("No device connected", keep_on_top=True, no_titlebar=True, grab_anywhere=True)
            elif event == "DEFAULT":
                for param in self.parameters:
                    window[param].update(value=self.parameters[param])
            elif event == 'STIMULATE':
                try:
                    self.nucleo.write('ENTER00'.encode())
                    print('ENTER00')
                except serial.SerialException:
                    sg.popup("No device connected", keep_on_top=True, no_titlebar=True, grab_anywhere=True)
            elif event == 'KEYBOARDSTIM':
                self.stimWindow.open(self.nucleo)

            elif event == "SHOWFORCES":
                self.forceWindow.open(self.nucleo)
                isOpen = [None]
                t1 = threading.Thread(target=optoForce.startDataTransfer, args=[self.forceWindow, isOpen])
                t1.start()
                time.sleep(0.2)
                if not isOpen[0]:
                    sg.popup("Optoforce not connected", keep_on_top=True, no_titlebar=True, grab_anywhere=True)
                    self.forceWindow.window.close()

            elif event[0:9] == "KEY_DOWN_":
                try:
                    switchIndex = self.keys.find(event[-1])
                    if self.currentChannel != event[-1]:
                        self.currentChannel = event[-1]
                        print(f'S{parseInput(inputString=str(switchIndex), allowZero=False)}')
                        self.nucleo.write(f'S{parseInput(inputString=str(switchIndex), allowZero=False)}00'.encode())
                        print(f'S{parseInput(inputString=str(switchIndex), allowZero=False)}00'.encode())
                        self.stimWindow.reset()
                        self.stimWindow.selectChannel(self.currentChannel, "green")
                except serial.SerialException:
                    sg.popup("No device connected", keep_on_top=True, no_titlebar=True, grab_anywhere=True)

            elif event == "UPBUTTON":
                self.dataReceiver.openConnection(5555, self.gripper)

            # elif event == "DOWNBUTTON":
            #     if self.dataReceiver.isConnected():
            #         self.dataReceiver.startReceiving(self.gripper)
            #     else:
            #         sg.popup("Connection not established with MATLAB", keep_on_top=True, no_titlebar=True, grab_anywhere=True)

            elif event == 'SETDATA':
                try:
                    badInputs = []
                    for param in self.parameters:
                        if parseInput(values[param]) == "" or \
                                (param == "PWM" and convertToNumber(parseInput(values[param])) >= 100):
                            badInputs.append(param[0])

                    if len(badInputs) == 0:
                        for param in self.parameters:
                            value = parseInput(values[param])
                            if param == "mWIDTH":
                                value = parseInput(str(round(int(values["mWIDTH"]) / int(values["DWIDTH"]) - 1)),
                                                   allowZero=False)
                            command = param[0] + value + '00'
                            self.nucleo.write(command.encode())
                            print(command)
                    else:
                        sg.popup(f"Wrong parameter format for: {badInputs}", keep_on_top=True, grab_anywhere=True)

                except serial.SerialException:
                    sg.popup("No device connected", keep_on_top=True, no_titlebar=True, grab_anywhere=True)

            elif event == 'COMMANDSEND':
                text = values['COMMANDLINE']
                text = text.split('\n')

                for i in text:
                    self.nucleo.write(i.encode())

        window.close()
        exit(0)


myApp = Application()
myApp.main()

