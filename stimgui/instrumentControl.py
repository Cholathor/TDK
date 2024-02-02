import pyvisa
import serial
import time
from StimProgramWindow import parseInput
import datetime
import pandas

resistorControl = {'10k': 0x03,
                   '33k': 0x05,
                   '68k': 0x09,
                   '100k': 0x11,
                   'inf': 0x00}
def takeMeasurement(nucleo, scope, supply, resistor, waitTime, frequency, t_on):

    f = parseInput(str(int(80000 / frequency)))
    D = parseInput(str(int(t_on * frequency * 0.1)))

    if t_on * frequency * 0.1 > 50.2 or t_on*frequency * 0.1 < 1:
        return 0, -1

    command = 'p' + f + '00'
    nucleo.write(command.encode())

    command = 'P' + D + '00'
    nucleo.write(command.encode())

    command = bytearray(7)
    command[0] = 0x10
    command[2] = resistorControl[resistor]

    nucleo.write(command)

    time.sleep(waitTime)

    V_out = float(scope.query(":MEASure:VAVerage?"))
    I_in = float(supply.query("MEAS:CURR?"))


    #print(f'R = {resistor}, V_out = {scope.query(":MEASure:VAVerage?")}, I_in = {supply.query("MEAS:CURR?")}')

    #if resistor != 'inf':
    #    print(resistor, "duty cycle: ", D)

    return V_out, I_in


nucleo = serial.Serial('COM4', baudrate=115200)

resistorControl = {'10k': 0x03,
                   '33k': 0x05,
                   '68k': 0x09,
                   '100k': 0x11,
                   'inf': 0x00}



rm = pyvisa.ResourceManager()

scope = rm.open_resource('USB0::0x0957::0x17A4::MY51361143::0::INSTR')
scope.read_termination = '\n'
scope.write_termination = '\n'

supply = rm.open_resource('supply')
supply.read_termination = '\n'
supply.write_termination = '\n'

command = bytearray(7)
command[0] = 0x10
nucleo.write(command)


#freqs = [40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 160, 165, 170, 175, 180, 185, 190, 195, 200]

#freqs = [260, 250, 240, 230, 210, 200, 190, 180, 170, 160, 150, 140, 130, 120, 110, 100, 90, 80, 70, 60, 50, 40]

freqs = [55, 50, 45, 40, 35, 30, 25, 20, 15, 10]

#freqs=[40]
#tons = [0.25, 0.5, 0.75, 1, 1.25, 1.5, 1.75, 2, 2.25, 2.5, 2.75, 3, 3.25, 3.5, 3.75, 4, 4.25, 4.5, 5]

wait = 3

results = []

resistors = ['33k']
#resistors = ['10k', '33k', '68k', '100k']

print('start: ', datetime.datetime.now())

limits = supply.query("APP:CURR?")
currentLimit = float(limits.strip().split(',')[0]) * 0.95


expected_seconds = len(freqs) * len(resistors) * wait * 25
print('expected end time: ', datetime.datetime.now() + datetime.timedelta(seconds=expected_seconds))

for f in freqs:

    us = 1000 / f
    maxD = us / 2

    tons = [(i + 1) * maxD / 10 for i in range(0, 10)]

    for ton in tons:
        #print(f'frequency: {f}, Ton: {ton}')

        I_in = 0
        for R in resistors:

            V_out, I_in = takeMeasurement(nucleo, scope, supply, R, wait, f, ton)

            results.append({'R': R,
                            'V_out': V_out,
                            'I_in': I_in,
                            'f': f,
                            't_on': ton
                            })

            print(f'R: {R}; V_out: {V_out}; I_in: {I_in}; f: {f}; t_on: {ton}')
            if I_in > currentLimit or I_in == -1:
                break
            V_out, I_in = takeMeasurement(nucleo, scope, supply, 'inf', wait*1.5, f, ton)

            results.append({'R': 'inf',
                            'V_out': V_out,
                            'I_in': I_in,
                            'f': f,
                            't_on': ton
                            })
            if I_in > currentLimit or I_in == -1:
                break
            print(f'R: inf; V_out: {V_out}; I_in: {I_in}; f: {f}; t_on: {ton}')


        if I_in > currentLimit or I_in == -1:
            break



# turn duty cycle way down to make current draw at end smaller
command = 'P001000'
nucleo.write(command.encode())
data = pandas.DataFrame(results)
data.to_excel('measurements/lowerfreqs.xlsx')

#takeMeasurement(nucleo, scope, supply, resistorControl['10k'], 0.1, 80, 1.5)