import time

import PySimpleGUI as sg
import serial.tools.list_ports
from serial.tools import list_ports

from Stimwindow import StimWindow
import GripperControl
from ForceWindow import ForceWindow
import optoForce
import threading
from DataReceiver import DataReceiver
from StimProgramWindow import StimProgramWindow
from SerialSender import parseInput
from SerialSender import SerialSender



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
    serialSender = SerialSender()

    def __init__(self):

        self.gripper = GripperControl.GripperControl()
        #self.gripper.connect()
        self.parameters = {"AWIDTH": 250, "BWIDTH": 250, "CWIDTH": 250, "DWIDTH": 2500, "NBURST": 10, "PAUSETIME": 125000,
                           "PWM": 30}



    def make_window(self, theme):
        sg.theme(theme)

        right_click_menu_def = [[], ['Edit Me', 'Versions', 'Nothing', 'More Nothing', 'Exit']]

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

        waveformImage = [sg.Image(source="waveformv2.png"), sg.Slider(range=(6, 127), default_value=0, resolution=1,
                                                          orientation='vertical', enable_events=True,
                                                          disable_number_display=False, key='SLIDER', expand_y=True)]

        setA = [sg.Text("A"), sg.Input(size=(5, 1), key="AWIDTH", default_text="250"), sg.Text("[µs]")]
        setB = [sg.Text("B"), sg.Input(size=(5, 1), key="BWIDTH", default_text="250"), sg.Text("[µs]")]
        setC = [sg.Text("C"), sg.Input(size=(5, 1), key="CWIDTH", default_text="250"), sg.Text("[µs]")]
        setD = [sg.Text("D"), sg.Input(size=(5, 1), key="DWIDTH", default_text="2500"), sg.Text("[µs]")]

        setDefault = [sg.Text(expand_x=True), sg.Button("Default", key="DEFAULT")]


        setNumberOfBursts = [sg.Text("Number of bursts"), sg.Input(size=(5, 1), key="NBURST", default_text="10")]

        resetButton = [sg.Text(expand_x=True), sg.Button("Reset", key="SYSTEMRESET")]

        setTimeBetweenBursts = [sg.Text("Time between bursts"), sg.Input(size=(10, 1), key="PAUSETIME", default_text="125000"), sg.Text("[µs]")]

        setPWMdutyCycle = [sg.Text("PWM duty cycle"), sg.Input(size=(5, 1), key="PWM", default_text="30"), sg.Text("%")]

        setAmplitude = [sg.Text("Amplitude"), sg.Input(size=(5, 1), key="VAMPLITUDE", default_text="1500"), sg.Text("mV")]

        openStimprogram = [sg.Text(expand_x=True), sg.Button("Program", key="-STIMPROGRAMBTN-")]

        adjust = [sg.Button("Adjust", key="SETDATA")]

        openKeyboardStim = [sg.Text(expand_x=True), sg.Button("Stimulate with keyboard", key="KEYBOARDSTIM", disabled=False), sg.Button("Show forces", key="SHOWFORCES")]

        stimulateButton = [sg.Button("Stimulate", key="STIMULATE")]

        stopButton = [sg.Button("Stop", key="STOP")]

        continuousStimulationCheckmark = [sg.Checkbox('c:', key='CONTINUOUS_STIMULATION')]

        connectButton = [sg.Text(expand_x=True), sg.Button("Connect to nucleo", key="CONNECT")]

        readButton = [sg.Button("Read serial", key="READSERIAL")]

        comportDisplay = [sg.Text(expand_x=True), sg.Text('not connected',key='COMPORTDISPLAY')]

        parameter_layout = [
            waveformImage,

            setA + setB + setC + setD + setDefault,

            setNumberOfBursts + resetButton,

            setTimeBetweenBursts,

            setPWMdutyCycle,

            setAmplitude + openStimprogram,

            adjust + openKeyboardStim,

            stimulateButton + continuousStimulationCheckmark + connectButton,

            stopButton + comportDisplay + readButton,
        ]

        command_layout = [[sg.Multiline(key="COMMANDLINE", expand_x=True, expand_y=True)],
                          [sg.Button("Send", key="COMMANDSEND")]]

        layout = [[sg.TabGroup([[sg.Tab('Parameter setup', parameter_layout),
                                 ],
                                [sg.Tab('Commandline', command_layout)]
                                ], key='-TAB GROUP-', expand_x=True, expand_y=True),
                   ]]

        return layout

    def popupGetComPorts(self):
        ports = list(list_ports.comports())
        ports_dict = {ports[i].description: ports[i].device for i in range(len(ports))}

        ports = [p.description for p in ports]

        max_len = max([len(i) for i in ports])


        layout = [[sg.Listbox(ports, select_mode=sg.LISTBOX_SELECT_MODE_SINGLE, expand_x=True, expand_y=True, key='COMLIST')],
                  [sg.Button("OK", key="COMSELECTED")]
                  ]

        window = sg.Window('Choose COM port', layout, use_default_focus=False, finalize=True, modal=True, size=(max_len * 10, len(ports)*50 + 50))
        event, values = window.read()
        window.close()
        return ports_dict[values['COMLIST'][0]], values['COMLIST'][0]



    def handleEvents(self, event, values, window):

        if self.forceWindow.window is not None and not self.forceWindow.window.is_closed():
            forceWindowEvent, _ = self.forceWindow.window.read(timeout=100)

            if forceWindowEvent == "STARTCALIBRATION":
                self.forceWindow.startCalibration()
                print("calibration started...")

        if self.stimProgramWindow.window is not None and not self.stimProgramWindow.window.is_closed():
            programWindowEvent, programWindowValues = self.stimProgramWindow.window.read(timeout=100)

            if programWindowEvent == "-SETSTIMPROGRAM-":
                self.stimProgramWindow.getDataFromText(programWindowValues['PROGRAMCODE'])
                self.stimProgramWindow.setProgram(self.serialSender)
            elif programWindowEvent == '-CHANNELINDEXADD-':
                self.stimProgramWindow.addToProgram(programWindowValues['-CHANNELBURSTNUMBER-'], programWindowValues)

        if self.stimWindow.window is not None and not self.stimWindow.window.is_closed():
            keyboardEvent, keyboardValues = self.stimWindow.window.read(timeout=100)

            print(keyboardEvent)


        match event:
            case 'SYSTEMRESET':
                self.serialSender.send('system reset')

            case 'READSERIAL':
                print(self.serialSender.readAll())

            case 'SLIDER Relese':
                self.serialSender.send('intensity', round(values['SLIDER']))

            case 'STOP':
                self.serialSender.send('stop stimulation')

            case '-STIMPROGRAMBTN-':
                if self.stimProgramWindow.window is not None:
                    self.stimProgramWindow.close()
                self.stimProgramWindow.open()

            case 'CONNECT':
                try:
                    port, description = self.popupGetComPorts()
                except:
                    print('Could not open port')
                    return
                if self.serialSender.connect(port) is not None:
                    window["COMPORTDISPLAY"].update(description)


            case 'DEFAULT':
                for param in self.parameters:
                    window[param].update(value=self.parameters[param])

            case 'STIMULATE':
                if values['CONTINUOUS_STIMULATION']:
                    self.serialSender.send('stimulate continuously')
                else:
                    self.serialSender.send('stimulate once')

            #case 'KEYBOARDSTIM':
            #    self.stimWindow.open(self.nucleo)

            case 'SHOWFORCES':
                self.forceWindow.open(self.serialSender)
                isOpen = [None]
                t1 = threading.Thread(target=optoForce.startDataTransfer, args=[self.forceWindow, isOpen])
                t1.start()
                time.sleep(0.2)
                if not isOpen[0]:
                    sg.popup("Optoforce not connected", keep_on_top=True, no_titlebar=True, grab_anywhere=True)
                    self.forceWindow.window.close()

            case 'UPBUTTON':
                self.dataReceiver.openConnection(5555, self.gripper)

            case 'SETDATA':

                if self.serialSender.ser is None or not self.serialSender.ser.is_open:
                    sg.popup("could not send: serial error", keep_on_top=True, no_titlebar=True, grab_anywhere=True)
                    return

                badInputs = []
                for param in self.parameters:
                    if parseInput(values[param]) == "" or \
                            (param == "PWM" and convertToNumber(parseInput(values[param])) >= 100):
                        badInputs.append(param[0])

                if len(badInputs) == 0:
                    for param in self.parameters:
                        if param != 'PAUSETIME':
                            self.serialSender.send(param, values[param])
                        else:
                            self.serialSender.send(param, values[param], values['DWIDTH'])


                else:
                    sg.popup(f"Wrong parameter format for: {badInputs}", keep_on_top=True, grab_anywhere=True)



            case 'COMMANDSEND':
                text = values['COMMANDLINE']
                text = text.split('\n')

                for i in text:
                    self.serialSender.send('command line instruction', i)


    def main(self):
        s = sg.theme('default1')
        window = self.make_window(s)

        ports = list(list_ports.comports())

        nucleo = [port for port in ports if 'STMicroelectronics STLink Virtual COM Port' in port.description][0]

        if nucleo is not None and self.serialSender.connect(nucleo.device) is not None:
            window["COMPORTDISPLAY"].update(nucleo.description)


        window.TKroot.bind('<Button>', change_focus)

        # for key in self.keys:
        #     window.bind(key, f'KEY_DOWN_{key}')
        # window.bind("<Up>", "UPBUTTON")
        # window.bind("<Down>", "DOWNBUTTON")
        window['SLIDER'].bind('<ButtonRelease-1>', ' Release')

        # This is an Event Loop
        while True:
            event, values = window.read(timeout=100)

            if event in (None, 'Exit'):
                print("[LOG] Clicked Exit!")
                break

            self.handleEvents(event, values, window)

            if event != '__TIMEOUT__':
                print(event)

        window.close()
        exit(0)


myApp = Application()
myApp.main()
