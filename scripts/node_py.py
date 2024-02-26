#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from phasespace_msgs.msg import Rigid, Rigids
import time

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander

from cflib.crazyflie.log import LogConfig

# TODO make this a parameter
uri = "radio://0/80/2M/E7E7E7E7E7"
iters = 0

class CFNode(Node):
    def __init__(self):
        super().__init__("drone_node")
        self.declare_parameter("drone_ids", rclpy.Parameter.Type.INTEGER_ARRAY)
        self.drone_ids = list(self.get_parameter("drone_ids").value)
        print(f"the drone id is {self.drone_ids}")
        self.drone_position = [0, 0, 0]
        self.drone_setpoint = [0, 0, 0, 0]

        timer_period = 1.0 / 150.0

        print("initializing drivers")
        cflib.crtp.init_drivers()

        self._cf = []
        self.drone_id_dict = dict()
        self.drone_idx_dict = dict()
        self.cf_links_established = []
        self.initial_pos_sent = []
        self.pos_commanders = []
        for (idx, id) in enumerate(self.drone_ids):
            self._cf.append(SyncCrazyflie(uri))
            self.drone_id_dict[id] = idx
            self.drone_idx_dict[idx] = id
            self.cf_links_established.append(False)
            self.initial_pos_sent.append(False)
            self._cf[idx].open_link()
            self.cf_links_established[idx] = True
            self.pos_commanders.append(PositionHlCommander(self._cf[idx], controller=PositionHlCommander.CONTROLLER_MELLINGER))
            # TODO allow takeoff only if position has been sent
            print("taking off")
            self.pos_commanders[idx].take_off()


        print('starting logging')
        ### This is for testing
        self._lg_stab = LogConfig(name='stab_logger', period_in_ms=100)
        self._lg_stab.add_variable('ctrltarget.x', 'float')
        self._lg_stab.add_variable('ctrltarget.z', 'float')
        # self._lg_stab.add_variable('stateEstimate.z', 'float')
        # self._lg_stab.add_variable('ctrlMel.cmd_thrust', 'float')
        # self._lg_stab.add_variable('ctrlMel.cmd_pitch', 'float')
        # self._lg_stab.add_variable('posCtl.Zp', 'float')
        # self._lg_stab.add_variable('posCtl.Xp', 'float')
        self._lg_stab.add_variable('motor.m1', 'float')
        self._lg_stab.add_variable('motor.m2', 'float')
        self._lg_stab.add_variable('motor.m3', 'float')
        self._lg_stab.add_variable('motor.m4', 'float')
        # self._lg_stab.add_variable('stabilizer.yaw', 'float')
        self._cf[0].cf.log.add_config(self._lg_stab)
        self._cf[0].cf.param.set_value("stabilizer.controller", 2)

        self.x = 0.2
        self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
        self._lg_stab.error_cb.add_callback(self._stab_log_error)

        self._lg_stab.start()

        # self._cf[idx].cf.high_level_commander.takeoff(1, 2)
        ### END Testing

        self.position_subscription = self.create_subscription(
            Rigids,
            "/phasespace/rigids",
            self.position_callback,
            5)

        self.radio_timer = self.create_timer(timer_period, self.radio_callback)
        # self.setpoint_subscription = self.create_subscription()


    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        print(f'[{timestamp}][{logconf.name}]: ', end='')
        for name, value in data.items():
            print(f'{name}: {value:3.3f} ', end='')
        print()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def position_callback(self, msg):
        for rigid_msg in msg.rigids:
            if rigid_msg.id == self.drone_id:
                # TODO store the drone positions
                print("do stuff")

    def setpoint_callback(self, msg):
        pass


    def radio_callback(self):
        global iters
        iters += 1
        if self.x <= 0.7:
            self.x += 0.001
        for (idx, link) in enumerate(self.cf_links_established):
            if link:
                # TODO get position from the positions dict and send to the corresponding drone
                self._cf[idx].cf.extpos.send_extpos(0, 0, self.x)
                # self._cf[idx].commander.send_setpoint(0, 0, 0, 10001)
                if not self.initial_pos_sent[idx]:
                    print("sending position")
                    self.initial_pos_sent[idx] = True
                    # TODO issue takeoff command for the drone
                else:
                    if iters >= 2000:
                        # TODO receive drone setpoints and send them
                        self.pos_commanders[idx].go_to(2, 0, 0.7)
                    else:
                        self.pos_commanders[idx].go_to(0, 0, 1)

def main():
    rclpy.init()

    node = CFNode()
    rclpy.spin(node)
    rclpy.shutdown()

    return True


if __name__ == "__main__":
    main()
