# Instructions for using the Force_Task in Python

## Prerequisites
- Python 3.x installed on your system.
- The "Force_Task.py" file in the same directory as your Python script.
- In my setup I use an Arduino board that streams the voltage signals of a force sensor via USB to the experiment computer. Upload the "send_smooth_data.ino" code to the Arduino.

## Getting started
1. Open your Python script in your code editor.
2. Import the "Force_Static_Task" class using the following command:

```python
from Force_Task import Force_Static_Task
```

3. You can now create an instance of the "Force_Static_Task" class and use its methods in your code.

Example:
```python
from Force_Task import Force_Static_Task
import serial

ser_sensor = serial.Serial('your/sensor/device')  # read stream from force sensor
ser_trigger = serial.Serial('your/trigger/device')  # write stream to EEG trigger box
task = Force_Static_Task(n_trials=10, inlet=ser_sensor, outlet=ser_trigger)
task.run()
```
