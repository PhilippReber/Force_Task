# Instructions for using the Force_Task in Python

## Prerequisites
- Python 3.x installed on your system.
- The "Force_Task.py" class file in the same directory as your Python script.

## Loading the Class
1. Open your Python script in your preferred code editor.
2. Import the "Force_Move_Task" class using the following command:

```python
from Force_Task import Force_Move_Task
```

3. You can now create an instance of the "Force_Move_Task" class and use its methods in your code.

Example:
```python
from Force_Task import Force_Static_Task
import serial

ser = serial.Serial('your/usb/device')
task = Force_Static_Task(n_trials=10, inlet=ser)
task.run()
```

This will create a new instance of the "Force_Static_Task" class and call its "run" method.
