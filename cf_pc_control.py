from __future__ import print_function

import sys
import time
import termios
import logging
import threading

import numpy as np
import transformations as trans
from cflib import crazyflie, crtp
from cflib.crazyflie.log import LogConfig

# Set a channel - if set to None, the first available crazyflie is used
#URI = 'radio://0/101/2M'
URI = None

def read_input(file=sys.stdin):
    """Registers keystrokes and yield these every time one of the
    *valid_characters* are pressed."""
    old_attrs = termios.tcgetattr(file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
        while True:
            try:
                yield sys.stdin.read(1)
            except (KeyboardInterrupt, EOFError):
                break
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)

class ControllerThread(threading.Thread):
    period_in_ms = 20  # Control period. [ms]
    thrust_step = 5e3   # Thrust step with W/S. [65535 = 100% PWM duty cycle]
    thrust_initial = 0
    thrust_limit = (0, 65535)
    roll_limit   = (-30.0, 30.0)
    pitch_limit  = (-30.0, 30.0)
    yaw_limit    = (-200.0, 200.0)
    enabled = False

    def __init__(self, cf):
        super(ControllerThread, self).__init__()
        self.cf = cf

        # Reset state
        self.disable(stop=False)

        # Keeps track of when we last printed
        self.last_time_print = 0.0

        # Connect some callbacks from the Crazyflie API
        self.cf.connected.add_callback(self._connected)
        self.cf.disconnected.add_callback(self._disconnected)
        self.cf.connection_failed.add_callback(self._connection_failed)
        self.cf.connection_lost.add_callback(self._connection_lost)
        self.send_setpoint = self.cf.commander.send_setpoint

        # Pose estimate from the Kalman filter
        self.pos = np.r_[0.0, 0.0, 0.0]
        self.vel = np.r_[0.0, 0.0, 0.0]
        self.attq = np.r_[0.0, 0.0, 0.0, 1.0]
        self.R = np.eye(3)

        # Attitide (roll, pitch, yaw) from stabilizer
        self.stab_att = np.r_[0.0, 0.0, 0.0]

        # This makes Python exit when this is the only thread alive.
        self.daemon = True

    def _connected(self, link_uri):
        print('Connected to', link_uri)

        log_stab_att = LogConfig(name='Stabilizer', period_in_ms=self.period_in_ms)
        log_stab_att.add_variable('stabilizer.roll', 'float')
        log_stab_att.add_variable('stabilizer.pitch', 'float')
        log_stab_att.add_variable('stabilizer.yaw', 'float')
        self.cf.log.add_config(log_stab_att)
    
        log_pos = LogConfig(name='Kalman Position', period_in_ms=self.period_in_ms)
        log_pos.add_variable('kalman.stateX', 'float')
        log_pos.add_variable('kalman.stateY', 'float')
        log_pos.add_variable('kalman.stateZ', 'float')
        self.cf.log.add_config(log_pos)
        
        log_vel = LogConfig(name='Kalman Velocity', period_in_ms=self.period_in_ms)
        log_vel.add_variable('kalman.statePX', 'float')
        log_vel.add_variable('kalman.statePY', 'float')
        log_vel.add_variable('kalman.statePZ', 'float')
        self.cf.log.add_config(log_vel)

        log_att = LogConfig(name='Kalman Attitude',
                            period_in_ms=self.period_in_ms)
        log_att.add_variable('kalman.q0', 'float')
        log_att.add_variable('kalman.q1', 'float')
        log_att.add_variable('kalman.q2', 'float')
        log_att.add_variable('kalman.q3', 'float')
        self.cf.log.add_config(log_att)
        
        if log_stab_att.valid and log_pos.valid and log_vel.valid and log_att.valid:
            log_stab_att.data_received_cb.add_callback(self._log_data_stab_att)
            log_stab_att.error_cb.add_callback(self._log_error)
            log_stab_att.start()

            log_pos.data_received_cb.add_callback(self._log_data_pos)
            log_pos.error_cb.add_callback(self._log_error)
            log_pos.start()

            log_vel.error_cb.add_callback(self._log_error)
            log_vel.data_received_cb.add_callback(self._log_data_vel)
            log_vel.start()

            log_att.error_cb.add_callback(self._log_error)
            log_att.data_received_cb.add_callback(self._log_data_att)
            log_att.start()
        else:
            raise RuntimeError('One or more of the variables in the configuration was not'
                               'found in log TOC. Will not get any position data.')

    def _connection_failed(self, link_uri, msg):
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        print('Disconnected from %s' % link_uri)

    def _log_data_stab_att(self, timestamp, data, logconf):
        self.stab_att = np.r_[data['stabilizer.roll'],
                              data['stabilizer.pitch'],
                              data['stabilizer.yaw']]
    
    def _log_data_pos(self, timestamp, data, logconf):
        self.pos = np.r_[data['kalman.stateX'],
                         data['kalman.stateY'],
                         data['kalman.stateZ']]

    def _log_data_vel(self, timestamp, data, logconf):
        vel_bf = np.r_[data['kalman.statePX'],
                       data['kalman.statePY'],
                       data['kalman.statePZ']]
        self.vel = np.dot(self.R, vel_bf)

    def _log_data_att(self, timestamp, data, logconf):
        # NOTE q0 is real part of Kalman state's quaternion, but
        # transformations.py wants it as last dimension.
        self.attq = np.r_[data['kalman.q1'], data['kalman.q2'],
                          data['kalman.q3'], data['kalman.q0']]
        # Extract 3x3 rotation matrix from 4x4 transformation matrix
        self.R = trans.quaternion_matrix(self.attq)[:3, :3]
        #r, p, y = trans.euler_from_quaternion(self.attq)

    def _log_error(self, logconf, msg):
        print('Error when logging %s: %s' % (logconf.name, msg))

    def make_position_sanity_check(self):
      # We assume that the position from the LPS should be
      # [-20m, +20m] in xy and [0m, 5m] in z
      if np.max(np.abs(self.pos[:2])) > 20 or self.pos[2] < 0 or self.pos[2] > 5:
        raise RuntimeError('Position estimate out of bounds', self.pos)

    def run(self):
        """Control loop definition"""
        while not self.cf.is_connected():
            time.sleep(0.2)

        print('Waiting for position estimate to be good enough...')
        self.reset_estimator()

        self.make_position_sanity_check();

        # Set the current reference to the current positional estimate, at a
        # slight elevation
        self.pos_ref = np.r_[self.pos[:2], 1.0]
        self.yaw_ref = 0.0
        print('Initial positional reference:', self.pos_ref)
        print('Initial thrust reference:', self.thrust_r)
        print('Ready! Press e to enable motors, h for help and Q to quit')
        log_file_name = 'flightlog_' + time.strftime("%Y%m%d_%H%M%S") + '.csv'
        with open(log_file_name, 'w') as fh:
            t0 = time.time()
            while True:
                time_start = time.time()
                self.calc_control_signals()
                if self.enabled:
                    sp = (self.roll_r, self.pitch_r, self.yawrate_r, int(self.thrust_r))
                    self.send_setpoint(*sp)
                    # Log data to file for analysis
                    ld = np.r_[time.time() - t0]
                    ld = np.append(ld, np.asarray(sp))
                    ld = np.append(ld, self.pos_ref)
                    ld = np.append(ld, self.yaw_ref)
                    ld = np.append(ld, self.pos)
                    ld = np.append(ld, self.vel)
                    ld = np.append(ld, self.attq)
                    ld = np.append(ld, (np.reshape(self.R, -1)))
                    ld = np.append(ld, trans.euler_from_quaternion(self.attq))
                    ld = np.append(ld, self.stab_att)
                    fh.write(','.join(map(str, ld)) + '\n')
                    fh.flush()
                self.loop_sleep(time_start)

    def calc_control_signals(self):
        # THIS IS WHERE YOU SHOULD PUT YOUR CONTROL CODE
        # THAT OUTPUTS THE REFERENCE VALUES FOR
        # ROLL PITCH, YAWRATE AND THRUST
        # WHICH ARE TAKEN CARE OF BY THE ONBOARD CONTROL LOOPS
        roll, pitch, yaw  = trans.euler_from_quaternion(self.attq)

        # Compute control errors in position
        ex,  ey,  ez  = self.pos_ref - self.pos

        # The code below will simply send the thrust that you can set using
        # the keyboard and put all other control signals to zero. It also
        # shows how, using numpy, you can threshold the signals to be between
        # the lower and upper limits defined by the arrays *_limit
        self.roll_r    = np.clip(0.0, *self.roll_limit)
        self.pitch_r   = np.clip(0.0, *self.pitch_limit)
        self.yawrate_r = np.clip(0.0, *self.yaw_limit)
        self.thrust_r  = np.clip(self.thrust_r, *self.thrust_limit)
    
        message = ('ref: ({}, {}, {}, {})\n'.format(self.pos_ref[0], self.pos_ref[1], self.pos_ref[2], self.yaw_ref)+
                   'pos: ({}, {}, {}, {})\n'.format(self.pos[0], self.pos[1], self.pos[2], yaw)+
                   'vel: ({}, {}, {})\n'.format(self.vel[1], self.vel[1], self.vel[2])+
                   'error: ({}, {}, {})\n'.format(ex, ey, ez)+
                   'control: ({}, {}, {}, {})\n'.format(self.roll_r, self.pitch_r, self.yawrate_r, self.thrust_r))
        self.print_at_period(2.0, message)

    def print_at_period(self, period, message):
        """ Prints the message at a given period """
        if (time.time() - period) >  self.last_time_print:
            self.last_time_print = time.time()
            print(message)

    def reset_estimator(self):
        self.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self.cf.param.set_value('kalman.resetEstimation', '0')
        # Sleep a bit, hoping that the estimator will have converged
        # Should be replaced by something that actually checks...
        time.sleep(1.5)

    def disable(self, stop=True):
        if stop:
            self.send_setpoint(0.0, 0.0, 0.0, 0)
        if self.enabled:
            print('Disabling controller')
        self.enabled = False
        self.roll_r    = 0.0
        self.pitch_r   = 0.0
        self.yawrate_r = 0.0
        self.thrust_r  = self.thrust_initial

    def enable(self):
        if not self.enabled:
            print('Enabling controller')
        # Need to send a zero setpoint to unlock the controller.
        self.send_setpoint(0.0, 0.0, 0.0, 0)
        self.enabled = True

    def loop_sleep(self, time_start):
        """ Sleeps the control loop to make it run at a specified rate """
        delta_time = 1e-3*self.period_in_ms - (time.time() - time_start)
        if delta_time > 0:
            time.sleep(delta_time)
        else:
            print('Deadline missed by', -delta_time, 'seconds. '
                  'Too slow control loop!')

    def increase_thrust(self):
        self.thrust_r += self.thrust_step
        self.thrust_r = min(self.thrust_r, 0xffff)

    def decrease_thrust(self):
        self.thrust_r -= self.thrust_step
        self.thrust_r = max(0, self.thrust_r)

def handle_keyboard_input(control):
    pos_step = 0.1 # [m]
    yaw_step = 5   # [deg]
    
    for ch in read_input():
        if ch == 'h':
            print('Key map:')
            print('>: Increase thrust (non-control mode)')
            print('<: Decrease thrust (non-control mode)')
            print('Q: quit program')
            print('e: Enable motors')
            print('q: Disable motors')
            print('w: Increase x-reference by ', pos_step, 'm.')
            print('s: Decrease x-reference by ', pos_step, 'm.')
            print('a: Increase y-reference by ', pos_step, 'm.')
            print('d: Decrease y-reference by ', pos_step, 'm.')
            print('i: Increase z-reference by ', pos_step, 'm.')
            print('k: Decrease z-reference by ', pos_step, 'm.')
            print('j: Increase yaw-reference by ', yaw_step, 'm.')
            print('l: Decrease yaw-reference by ', yaw_step, 'deg.')
        elif ch == '>':
            control.increase_thrust()
            print('Increased thrust to', control.thrust_r)
        elif ch == '<':
            control.decrease_thrust()
            print('Decreased thrust to', control.thrust_r)
        elif ch == 'w':
            control.pos_ref[0] += pos_step
            print('Reference position changed to :', control.pos_ref)
        elif ch == 's':
            control.pos_ref[0] -= pos_step
            print('Reference position changed to :', control.pos_ref)
        elif ch == 'a':
            control.pos_ref[1] += pos_step
            print('Reference position changed to :', control.pos_ref)
        elif ch == 'd':
            control.pos_ref[1] -= pos_step
            print('Reference position changed to :', control.pos_ref)
        elif ch == 'i':
            control.pos_ref[2] += pos_step
            print('Reference position changed to :', control.pos_ref)
        elif ch == 'k':
            control.pos_ref[2] -= pos_step
            print('Reference position changed to :', control.pos_ref)
        elif ch == 'j':
            control.yaw_ref += np.radians(yaw_step)
            print('Yaw reference changed to :',
                    np.degrees(control.yaw_ref), 'deg.')
        elif ch== 'l':
            control.yaw_ref -= np.radians(yaw_step)
            print('Yaw reference changed to :',
                    np.degrees(control.yaw_ref), 'deg.')
        elif ch == ' ':
            control.pos_ref[2] = 0.0
            print('Reference position changed to :', control.pos_ref)
        elif ch == 'e':
            control.enable()
        elif ch == 'q':
            if not control.enabled:
                print('Uppercase Q quits the program')
            control.disable()
        elif ch == 'Q':
            control.disable()
            print('Bye!')
            break
        else:
            print('Unhandled key', ch, 'was pressed')

if __name__ == "__main__":
    logging.basicConfig()
    crtp.init_drivers(enable_debug_driver=False)
    cf = crazyflie.Crazyflie(rw_cache='./cache')
    control = ControllerThread(cf)
    control.start()

    if URI is None:
        print('Scanning for Crazyflies...')
        available = crtp.scan_interfaces()
        if available:
            print('Found Crazyflies:')
            for i in available:
                print('-', i[0])
            URI = available[0][0]
        else:
            print('No Crazyflies found!')
            sys.exit(1)

    print('Connecting to', URI)
    cf.open_link(URI)

    handle_keyboard_input(control)

    cf.close_link()
