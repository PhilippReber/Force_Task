from psychopy import event, core
from psychopy.visual import TextStim, Window, Circle, Line
import serial
from platform import system
import numpy as np
from numpy.random import choice, permutation
import time


class Force_Move_Task:

    def __init__(self, freqs, demands, n_trials, inlet, break_time=5, calibrate=True,
                 monitor='testMonitor', y_max=7, radius_target=0.8, radius_subject=0.5):
        self.win = Window(fullscr=True, allowGUI=True, monitor=monitor, units='deg', color=0)
        self.y_max = y_max
        self.radius_target = radius_target
        self.radius_subject = radius_subject
        self.target = Circle(self.win, radius=radius_target, fillColor='yellow')
        self.subject = Circle(self.win, radius=radius_subject, fillColor='blue')
        self.line_up = Line(self.win, start=(-4,y_max+radius_target), end=(4, y_max+radius_target), lineWidth=1)
        self.line_low = Line(self.win, start=(-4,-y_max-radius_target), end=(4,-y_max-radius_target), lineWidth=1)
        self.msg = TextStim(self.win, text='', color=1, height=1, wrapWidth=20, pos=(-4.5,0), alignHoriz='right')
        self.msg_calibrate = TextStim(self.win, text='', color=1, height=1, wrapWidth=20, pos=(0,0))
        self.frame_rate = int(1 / self.win.monitorFramePeriod)
        self.calibrate = calibrate
        self.freqs = freqs
        self.n_trials = n_trials
        self.demands = demands
        self.inlet = inlet
        self.baseline = 25
        self.max_force = 450  # standard value if not calibrated
        self.break_time = break_time

    
    def make_moves(self):
        mesh = np.array(np.meshgrid(self.freqs, self.demands)).T.reshape(-1,2)
        ixs = np.arange(len(mesh))
        moves = mesh[choice(ixs, 5)]
        while len(np.unique(moves[:,0])) < 5 or len(np.unique(moves[:,1])) < 5:
            moves = mesh[choice(ixs, 5)]
        return permutation(list(moves) * 3)  # 3 repetitions of 5 random moves (per trial)
    
    def get_force(self):
        self.inlet.write(bytes('a\n', 'utf-8'))
        line = self.inlet.readline()
        if line:
            force = float(line.decode())
            return force

    def make_subject_pos(self, demand):
        y_max = self.y_max
        force = self.get_force()
        if force:
            gradient = 2 * y_max / ((self.max_force - self.baseline) * demand)
            sub_y = -y_max + (force - 0.5 * self.baseline) * gradient 
            if sub_y > y_max:
                sub_y = y_max
            if sub_y < -y_max:
                sub_y = -y_max
            return (0, sub_y)
    
    def calibrate(self): 
        self.msg_calibrate.text = 'Apply maximal force for 10 seconds\n(only index finger, no weight)\n\nPress k to start'
        self.msg_calibrate.draw()
        self.win.flip()
        event.waitKeys(keyList=['k'])
        self.msg_calibrate.text = 'Apply maximal force...'
        self.msg_calibrate.draw()
        self.win.flip()
        forces = []
        t0 = time.time()
        while time.time() - t0 < 10:
            force = self.get_force()
            forces.append(force)
        self.max_force = np.mean(sorted(forces, reverse=True)[:50])
        print('max force estimated at', self.max_force)
        self.msg_calibrate.text = 'Done'
        self.msg_calibrate.draw()
        self.win.flip()
        core.wait(3)

    def run(self):
        if self.calibrate:
            self.calibrate()
        moves = self.make_moves()
        y_max = self.y_max
        n_trials = self.n_trials
        tar_x, tar_y = (0,-y_max)
        sub_x, sub_y = (0,-y_max)
        self.target.pos = (tar_x,tar_y)
        self.msg.text = 'Press k to start'

        # play before start
        event.clearEvents()
        while not 'k' in event.getKeys(keyList=['k']):
            sub_x, sub_y = self.make_subject_pos(demand=0.090)
            self.subject.pos = sub_x, sub_y
            if np.logical_and(sub_y > (tar_y - self.radius_target), sub_y < (tar_y + self.radius_target)):
                self.target.fillColor = 'green'
            else:
                self.target.fillColor = 'yellow'
            self.target.draw()
            self.subject.draw()
            self.line_up.draw()
            self.line_low.draw()
            self.msg.draw()
            self.win.flip()

        # start trial
        accuracy_all_trials = []
        for trial_ix in range(n_trials):
            accuracy = []
            for freq, demand in moves:
                seconds_per_combination = 2  # how long each freq-demand combination should run
                step = y_max * 2 * freq / self.frame_rate
                for _ in range(self.frame_rate * seconds_per_combination):
                    # compute target location
                    if tar_y <= -y_max:
                        direction = 'up'
                    elif tar_y >= y_max:
                        direction = 'down'
                    elif tar_y < y_max and direction == 'up': 
                        pass
                    else:
                        direction = 'down'

                    if direction == 'up':
                        tar_y += step
                    else:
                        tar_y -= step

                    # compute subject location
                    sub_x, sub_y = self.make_subject_pos(demand=demand)
                    self.subject.pos = sub_x, sub_y
                    accuracy.append(abs(tar_y - sub_y))

                    # place markers on screen
                    self.target.pos = (tar_x, tar_y)
                    if np.logical_and(sub_y > (tar_y - self.radius_target), sub_y < (tar_y + self.radius_target)):
                        self.target.fillColor = 'green'
                    else:
                        self.target.fillColor = 'yellow'
                    self.target.draw()
                    self.subject.draw()
                    self.line_up.draw()
                    self.line_low.draw()
                    self.win.flip()

            accuracy_all_trials.append(accuracy)
            
            if trial_ix + 1 == n_trials: 
                break

            # break
            tar_y = -y_max
            sub_y = -y_max
            self.target.pos = 0, tar_y
            self.target.fillColor = 'yellow'
            self.subject.pos = 0, sub_y
            for t in range(self.break_time):
                self.msg.text = 'break %ss...' % (self.break_time - int(t))
                self.msg.draw()
                self.target.draw()
                self.subject.draw()
                self.line_up.draw()
                self.line_low.draw()
                self.win.flip()
                core.wait(1)

        self.inlet.close()
        np.save('accuracy_all_trials', accuracy_all_trials); print('Data saved to root dir')


class Force_Static_Task:

    def __init__(self, n_trials, inlet, demand=0.1, break_time=5, calibrate=True,
                 monitor='testMonitor', y_max=7, radius_target=0.6, radius_subject=0.5):
        self.win = Window(fullscr=True, allowGUI=True, monitor=monitor, units='deg', color=0)
        self.y_max = y_max
        self.radius_target = radius_target
        self.radius_subject = radius_subject
        self.target = Circle(self.win, radius=radius_target, fillColor='yellow')
        self.subject = Circle(self.win, radius=radius_subject, fillColor='blue')
        self.line_up = Line(self.win, start=(-4,y_max+radius_target), end=(4, y_max+radius_target), lineWidth=1)
        self.line_low = Line(self.win, start=(-4,-y_max-radius_target), end=(4,-y_max-radius_target), lineWidth=1)
        self.msg = TextStim(self.win, text='', color=1, height=1, wrapWidth=20, pos=(-4.5,0), alignHoriz='right')
        self.msg_calibrate = TextStim(self.win, text='', color=1, height=1, wrapWidth=20, pos=(0,0))
        self.frame_rate = int(1 / self.win.monitorFramePeriod)
        self.n_trials = n_trials
        self.demand = demand
        self.inlet = inlet
        self.baseline = 25
        self.max_force = 450
        self.break_time = break_time
    
    def get_force(self):
        self.inlet.write(bytes('a\n', 'utf-8'))
        line = self.inlet.readline()
        if line:
            force = float(line.decode())
            return force
    
    def make_subject_pos(self, demand):
        y_max = self.y_max
        force = self.get_force()
        if force:
            gradient = 2 * y_max / ((self.max_force - self.baseline) * demand)
            sub_y = -y_max + (force - 0.5 * self.baseline) * gradient 
            if sub_y > y_max:
                sub_y = y_max
            if sub_y < -y_max:
                sub_y = -y_max
            return (0, sub_y)
    
    def calibrate(self): 
        self.msg_calibrate.text = 'Apply maximal force for 10 seconds\n(only index finger, no weight)\n\nPress k to start'
        self.msg_calibrate.draw()
        self.win.flip()
        event.waitKeys(keyList=['k'])
        self.msg_calibrate.text = 'Apply maximal force...'
        self.msg_calibrate.draw()
        self.win.flip()
        forces = []
        t0 = time.time()
        while time.time() - t0 < 10:
            force = self.get_force()
            forces.append(force)
        self.max_force = np.mean(sorted(forces, reverse=True)[:50])
        print('max force estimated at', self.max_force)
        self.msg_calibrate.text = 'Done'
        self.msg_calibrate.draw()
        self.win.flip()
        core.wait(3)

    def run(self):
        if self.calibrate:
            self.calibrate()
        y_max = self.y_max
        n_trials = self.n_trials
        tar_x, tar_y = (0,0)
        sub_x, sub_y = (0,-y_max)
        self.target.pos = (tar_x,tar_y)
        self.subject.pos = (sub_x,sub_y)
        self.msg.text = 'Press k to start'

        # play before start
        event.clearEvents()
        while not 'k' in event.getKeys(keyList=['k']):
            sub_x, sub_y = self.make_subject_pos(demand=self.demand)
            self.subject.pos = sub_x, sub_y 
            if np.logical_and(sub_y > (tar_y - self.radius_target), sub_y < (tar_y + self.radius_target)):
                self.target.fillColor = 'green'
            else:
                self.target.fillColor = 'yellow'
            self.target.draw()
            self.subject.draw()
            self.line_up.draw()
            self.line_low.draw()
            self.msg.draw()
            self.win.flip()

        # start trial
        accuracy_all_trials = []
        for trial_ix in range(n_trials):
            accuracy = []
            trial_len = 10  # how long each freq-demand combination should run
            for _ in range(self.frame_rate * trial_len):

                # compute subject location
                sub_x, sub_y = self.make_subject_pos(demand=self.demand)
                self.subject.pos = sub_x, sub_y
                accuracy.append(abs(tar_y - sub_y))

                # place markers on screen
                self.target.pos = (tar_x, tar_y)
                if np.logical_and(sub_y > (tar_y - self.radius_target), sub_y < (tar_y + self.radius_target)):
                    self.target.fillColor = 'green'
                else:
                    self.target.fillColor = 'yellow'
                self.target.draw()
                self.subject.draw()
                self.line_up.draw()
                self.line_low.draw()
                self.win.flip()

            accuracy_all_trials.append(accuracy)
            
            if trial_ix + 1 == n_trials: 
                break

            # break
            sub_y = -y_max
            self.target.pos = 0, tar_y
            self.target.fillColor = 'yellow'
            self.subject.pos = 0, sub_y
            for t in range(self.break_time):
                self.msg.text = 'break %is...' % (self.break_time - int(t))
                self.msg.draw()
                self.target.draw()
                self.subject.draw()
                self.line_up.draw()
                self.line_low.draw()
                self.win.flip()
                core.wait(1)

        self.inlet.close()
        np.save('accuracy_all_trials', accuracy_all_trials); print('Data saved to root dir')

def send_trigger(trigger, device):
    if not system == 'Darwin':
        device.write(str.encode(chr(0)))
        device.write(str.encode(chr(trigger)))
        device.flush()
        device.reset_output_buffer()

def main():
    # global settings 
    ser = serial.Serial('/dev/cu.usbmodem101')
    event.globalKeys.add("escape", func=core.quit)
    
    # experimental parameters
    freqs = [0.4, 0.45, 0.48, 0.59, 0.67]  # movement speed in Hz (top to bottom)
    # demands = [0.05, 0.08, 0.1, 0.12, 0.15]  # of max force (original)
    demands = [0.092, 0.094, 0.096, 0.098, 0.100]  # of max force (adjusted)
    task = Force_Move_Task(freqs=freqs, demands=demands, n_trials=10, inlet=ser, trigger='1')  
    # task = Force_Static_Task(n_trials=10, inlet=ser, demand=0.1)  
    task.run()

if __name__ == '__main__':
    main()