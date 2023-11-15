import PySimpleGUI as sg
import serial


# convert "1234" to 123 * 10e4
def convertToNumber(inputString: str) -> int:
    return int(inputString[0:3]) * (10 ** int(inputString[3]))


# convert "12300" to "1234" (123 * 10e4)
def parseInput(inputString: str) -> str:
    if not inputString.isdigit() or len(inputString) > 12 or int(inputString) == 0:
        return ""

    inputString = str(int(inputString))

    if len(inputString) <= 3:
        return "0" * (3 - len(inputString)) + inputString + "0"
    result = inputString[0:3] + str(len(inputString) - 3)
    return result


class Application:

    def __init__(self):
        self.parameters = {"AWIDTH": 250, "BWIDTH": 250, "CWIDTH": 250, "DWIDTH": 2500, "NBURST": 10, "MWIDTH": 125000, "PWM": 30, "VAMPLITUDE": 1500,
                                  "OFFSET": 1550}

        self.ser = serial.Serial()
        try:
            self.ser = serial.Serial('COM4', 115200)
        except serial.SerialException:
            sg.popup("No device connected", keep_on_top=True, no_titlebar=True, grab_anywhere=True)

    def make_window(self, theme):
        sg.theme(theme)

        #menu_def = [['Device', ['Connect']]]
        right_click_menu_def = [[], ['Edit Me', 'Versions', 'Nothing', 'More Nothing', 'Exit']]

        #layout = [[sg.MenubarCustom(menu_def, key='-MENU-', font='Courier 15', tearoff=False, background_color='#fff')],
         #         [sg.Text('StimGUI', size=(20, 1), justification='center', font=("Courier 15", 16),
          #                 relief=sg.RELIEF_RIDGE, k='-TEXT HEADING-', enable_events=True)]]

        layout = self.getLayouts()

        layout[-1].append(sg.Sizegrip())
        window = sg.Window('StimGUI', layout, right_click_menu=right_click_menu_def,
                           right_click_menu_tearoff=True, grab_anywhere=False, resizable=True, margins=(0, 0),
                           use_custom_titlebar=False, finalize=True, keep_on_top=False, icon='icon.ico')

        window.set_min_size(window.size)

        return window

    def getLayouts(self):
        """
        To add a layout, define it as a list, and add elements to it. This list will be drawn vertically. To add
        elements horizontally, you can add them in a list (inside the list)
        :return: inner layout of the application
        """

        parameter_layout = [
            [sg.Image(source="waveformv2.png")],

            [sg.Text("A"), sg.Input(size=(5, 1), key="AWIDTH", default_text="250"), sg.Text("[µs]"),
             sg.Text("B"), sg.Input(size=(5, 1), key="BWIDTH", default_text="250"), sg.Text("[µs]"),
             sg.Text("C"), sg.Input(size=(5, 1), key="CWIDTH", default_text="250"), sg.Text("[µs]"),
             sg.Text("D"), sg.Input(size=(5, 1), key="DWIDTH", default_text="2500"), sg.Text("[µs]")],

            [sg.Text("Number of bursts"), sg.Input(size=(5, 1), key="NBURST", default_text="10")],

            [sg.Text("Time between bursts"),
             sg.Input(size=(10, 1), key="MWIDTH", default_text="125000"), sg.Text("[µs]")],


            [sg.Text("PWM duty cycle"), sg.Input(size=(5, 1), key="PWM", default_text="30"), sg.Text("%")],

            [sg.Text("Amplitude"), sg.Input(size=(5, 1), key="VAMPLITUDE", default_text="1500"), sg.Text("mV")],

            [sg.Text("Offset"), sg.Input(size=(5, 1), key="OFFSET", default_text="1550"), sg.Text("mV")],

            [sg.Button("Adjust", key="SETDATA"), sg.Text(expand_x=True), sg.Button("Connect", key="CONNECT")],

            [sg.Button("Stimulate", key="STIMULATE"), sg.Text(expand_x=True), sg.Button("Default", key="DEFAULT")],
        ]

        layout = [[sg.TabGroup([[sg.Tab('Parameter setup', parameter_layout),
                                 ]], key='-TAB GROUP-', expand_x=True, expand_y=True),
                   ]]

        return layout

    def main(self):
        s = sg.theme('default1')
        window = self.make_window(s)

        # This is an Event Loop
        while True:
            event, values = window.read(timeout=100)
            # keep an animation running so show things are happening

            if event in (None, 'Exit'):
                print("[LOG] Clicked Exit!")
                break
            elif event == 'CONNECT':
                try:
                    self.ser.close()
                    self.ser = serial.Serial('COM4', 115200)
                    sg.popup("Connected", keep_on_top=True, no_titlebar=True, grab_anywhere=True)
                except serial.SerialException:
                    sg.popup("No device connected", keep_on_top=True, no_titlebar=True, grab_anywhere=True)
            elif event == "DEFAULT":
                for param in self.parameters:
                    window[param].update(value=self.parameters[param])
            elif event == 'STIMULATE':
                try:
                    self.ser.write('ENTER'.encode())
                except serial.SerialException:
                    sg.popup("No device connected", keep_on_top=True, no_titlebar=True, grab_anywhere=True)
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
                            if param == "MWIDTH":
                                value = parseInput(str(round(int(values["MWIDTH"]) / int(values["DWIDTH"]) - 1)))
                            command = param[0] + value
                            self.ser.write(command.encode())
                    else:
                        sg.popup(f"Wrong parameter format for: {badInputs}", keep_on_top=True, grab_anywhere=True)

                except serial.SerialException:
                    sg.popup("No device connected", keep_on_top=True, no_titlebar=True, grab_anywhere=True)
        window.close()
        exit(0)


myApp = Application()
myApp.main()