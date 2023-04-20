import serial
from Force_Task import Force_Move_Task
from Force_Task import Force_Static_Task


# setup
ser = serial.Serial('COM5')
ser_trigger = serial.Serial('COM4')

# experimental parameters
freqs = [0.4, 0.45, 0.48, 0.59, 0.67]  # movement speed in Hz (top to bottom)
demands = [0.092, 0.094, 0.096, 0.098, 0.100]  # of max force (adjusted)

task = Force_Static_Task(n_trials=5, inlet=ser, outlet=ser_trigger, demand=0.05)
# task = Force_Move_Task(freqs, demands, n_trials=10, inlet=ser, outlet=ser_trigger)

task.run()
