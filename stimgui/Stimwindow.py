import PySimpleGUI as sg
import serial


def change_focus(event):
    event.widget.focus_set()


class StimWindow:
    width = 1000
    height = 500
    lastChannel = ''
    nucleo = None
    window = None

    def __init__(self, keys):
        self.keys = keys
        self.rectGap = 5
        self.rectSize = self.width * 2 / len(keys) - self.rectGap*2
    def open(self, nucleo: serial.Serial):
        self.nucleo = nucleo

        self.layout = [
            [sg.Input(key="-INTENSITY-")],
            [sg.Graph(canvas_size=(self.width, self.height), graph_bottom_left=(0, 0), graph_top_right=(self.width, self.height), key="GRAPH")]]

        self.window = sg.Window("Stimwindow", layout=self.layout)
        self.window.Finalize()

        self.window.TKroot.bind('<Button>', change_focus)
        for key in self.keys:
            self.selectChannel(key, "gray")

    def close(self):
        self.window.close()

    def selectChannel(self, channel: str, color: str):

        self.lastChannel = channel
        index = self.keys.find(channel)
        if index < 8:
            pos = (index * self.width * 2 / len(self.keys) + self.rectGap, self.height - self.rectGap)
        else:
            pos = ((index-8) * self.width * 2 / len(self.keys) + self.rectGap,  0 + self.rectGap + self.rectSize * 2)

        self.drawRect(pos, color)
        self.window["GRAPH"].draw_text(text=channel, location=(pos[0]+self.rectSize/2, pos[1] - self.rectSize/2), font=("Courier New", 30))

        channelindex = '{:0>2}'.format(index)
        command = 's00' + channelindex + '01'
        print(command)
        #self.nucleo.write(command.encode())

        command = 'v001000'
        print(command)
        #self.nucleo.write(command.encode())

        if self.window["-INTENSITY-"].get().isdigit():
            intensityValue = int(self.window["-INTENSITY-"].get())
            command = 'V0' + '{:0>3}'.format(intensityValue) + channelindex
            print(command)
            #self.nucleo.write(command.encode())

    def drawRect(self, pos: tuple[int, int], color: str):
        self.window["GRAPH"].DrawRectangle(pos, (pos[0] + self.rectSize, pos[1] - self.rectSize), fill_color=color, line_color="orange", line_width=5)

    def reset(self):
        index = self.keys.find(self.lastChannel)
        if index < 8:
            pos = (index * self.width * 2 / len(self.keys) + self.rectGap, self.height - self.rectGap)
        else:
            pos = ((index - 8) * self.width * 2 / len(self.keys) + self.rectGap, 0 + self.rectGap + self.rectSize * 2)

        self.drawRect(pos, "gray")
        self.window["GRAPH"].draw_text(text=self.lastChannel, location=(pos[0] + self.rectSize / 2, pos[1] - self.rectSize / 2),
                                       font=("Courier New", 30))
