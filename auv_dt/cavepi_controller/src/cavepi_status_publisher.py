#!/usr/bin/env python3

# =====================================
# Author : Alakrit Gupta
# Email: gupta.alankrit@ufl.edu
# =====================================

"""
Keyboard tele‑operation interface for Cavepi.
 ‑ Manual planar movement with WASD‑like keys
 ‑ Vertical motion (“heave”) is overridden by /heave_control_input unless
   the operator presses i (up) or k (down)
"""

from __future__ import print_function

import sys
import select
import termios
import tty

import rospy
from std_msgs.msg import Float32, Bool
from cavepi_interfaces.msg import CavepiInput

# On‑screen banner
BANNER = """
Reading from the keyboard
---------------------------
Planar Movement:
   q    w    e
   a         d
   z    x    c

i : up  (+z)
k : down (-z)

Anything else : stop

r / v : increase / decrease thruster power by 10 %%
---------------------------
CTRL‑C to quit
"""

# (surge, sway, heave, roll, pitch, yaw)
MOVE_BINDINGS = {
    'q': ( (2**0.5)/2, 0, 0, 0, 0, -(2**0.5)/2),
    'w': ( 1,          0, 0, 0, 0,  0),
    'e': ( (2**0.5)/2, 0, 0, 0, 0,  (2**0.5)/2),
    'a': ( 0,          0, 0, 0, 0, -1),
    'd': ( 0,          0, 0, 0, 0,  1),
    'z': (-(2**0.5)/2, 0, 0, 0, 0, -(2**0.5)/2),
    'x': (-1,          0, 0, 0, 0,  0),
    'c': (-(2**0.5)/2, 0, 0, 0, 0,  (2**0.5)/2),

    'i': ( 0, 0, -1, 0, 0, 0),   # ascend
    'k': ( 0, 0,  1, 0, 0, 0),   # descend
}

SPEED_BINDINGS = {
    'r': +0.10,     # faster
    'v': -0.10,     # slower
}

# Node implementation
class CavepiTeleop:
    def __init__(self):
        # Save terminal state so we can restore it on exit
        self._term_settings = termios.tcgetattr(sys.stdin)

        # Publishers
        self.cmd_pub  = rospy.Publisher("/cavepi/user_input",
                                        CavepiInput, queue_size=1)
        self.ind_pub  = rospy.Publisher("/depth_change_indicator",
                                        Bool, queue_size=1)

        # Subscriber: automated heave control
        rospy.Subscriber("/heave_control_input",
                         Float32, self._heave_control_cb, queue_size=1)

        # Parameters / state
        self.speed = rospy.get_param("~speed", 0.05)   # 5 % default
        self.surge = self.heave = self.roll = self.yaw = 0.0
        self.controlled_heave = 0.0                    # from topic

        rospy.loginfo("Thruster power initialised to %.0f %%", self.speed * 100)

    
    # I/O helpers
    def _get_key(self, timeout=0.05):
        """
        Return the next key press, or '' if none is available within *timeout*.
        Non‑blocking thanks to a short select() timeout.
        """
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._term_settings)
        return key

    
    # Subscriber callback
    def _heave_control_cb(self, msg: Float32):
        """Update automatic heave set‑point."""
        self.controlled_heave = msg.data
        rospy.logdebug("Heave override updated to %.3f", self.controlled_heave)

    
    # Message publishing
    def _publish_current_cmd(self):
        """
        Publish CavepiInput with either manual heave (if i/k pressed)
        or the latest automatic heave from /heave_control_input.
        """
        depth_changed = (self.heave != 0)

        cmd = CavepiInput()
        cmd.surge = self.surge * self.speed
        cmd.heave = (self.heave if depth_changed else self.controlled_heave) * self.speed
        cmd.roll  = self.roll  * self.speed
        cmd.yaw   = self.yaw   * self.speed
        self.cmd_pub.publish(cmd)

        self.ind_pub.publish(Bool(data=depth_changed))

    def _stop_robot(self):
        self.surge =  self.roll = self.yaw = 0.0
        self.heave = self.controlled_heave
        self._publish_current_cmd()

    
    # Main loop
    def run(self):
        print(BANNER)
        rate = rospy.Rate(20)           # 20 Hz → 50 ms loop period
        try:
            while not rospy.is_shutdown():
                key = self._get_key()

                # Movement keys
                if key in MOVE_BINDINGS:
                    (self.surge, _,
                     self.heave, self.roll, _,
                     self.yaw) = MOVE_BINDINGS[key]

                # Speed adjustment
                elif key in SPEED_BINDINGS:
                    self.speed = max(0.0, min(1.0, self.speed + SPEED_BINDINGS[key]))
                    rospy.loginfo("Thruster power now %.0f %%", self.speed * 100)

                # Graceful quit on CTRL‑C
                elif key == '\x03':
                    self._stop_robot()
                    break

                # Any other pressed key stops the robot
                elif key != '':
                    self._stop_robot()

                # (If key == '' → no key this cycle → keep previous command)

                self._publish_current_cmd()
                rate.sleep()

        finally:
            self._stop_robot()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._term_settings)



if __name__ == '__main__':
    rospy.init_node('cavepi_teleop_keyboard')
    CavepiTeleop().run()
