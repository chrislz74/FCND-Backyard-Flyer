import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        size = 7.0
        alt = 30.0
        self.all_waypoints = [[size, 0.0, alt, 0.0],[size, size, alt, 0.0], [0.0, size, alt, 0.0], [0.0, 0.0, alt, 0.0]]
        self.wp = -1
        self.nextWP = []
        self.tol = 0.1
        self.in_mission = True
        self.check_phase = {}    

        # initial state
        self.flight_phase = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        if self.flight_phase == States.TAKEOFF:
            # coordinate conversion 
            altitude = -1.0 * self.local_position[2]
            # check if altitude is within 95% of target
            if altitude > 0.95 * self.target_position[2]:
                self.waypoint_transition()        
        
        if self.flight_phase == States.WAYPOINT:
            # pruefe ob self.local_position = WP
            dist = np.sqrt((self.nextWP[0]-self.local_position[0])**2 + (self.nextWP[1]-self.local_position[1])**2)
            speed = np.sqrt(self.local_velocity[0]**2 + self.local_velocity[1]**2 + self.local_velocity[2]**2)
            print("Distance: ", dist, ",  Position: ", self.local_position, ",  Speed: ", speed)
            # wenn wp=3 und position erreicht, dann nach landing_transition
            if dist < self.tol:
                if self.wp == 3:
                    self.landing_transition()
                else:
                    self.waypoint_transition()
        
        if self.flight_phase == States.LANDING:
            if ((self.global_position[2] - self.global_home[2] < 0.1) and abs(self.local_position[2]) < 0.05):
                self.disarming_transition()        

        pass

    def velocity_callback(self):
        # brauch ich nicht
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """ 
        pass

    def state_callback(self):
        if not self.in_mission:
            return
        if self.flight_phase == States.MANUAL:
            self.arming_transition()
        elif self.flight_phase == States.ARMING:
            if self.armed:
                self.takeoff_transition()
        elif self.flight_phase == States.DISARMING:
            if not self.armed:
                self.manual_transition()
        pass

    def calculate_box(self):
        """TODO: Fill out this method
        
        1. Return waypoints to fly a box
        """
        self.wp = self.wp + 1
        self.nextWP = self.all_waypoints[self.wp]
        print(self.nextWP)
      
        pass

    def arming_transition(self):
        self.take_control()
        self.arm()

        # set the current location to be the home position
        self.set_home_position(self.global_position[0], self.global_position[1], self.global_position[2])

        self.flight_phase = States.ARMING
        print("arming transition")

    def takeoff_transition(self):
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_phase = States.TAKEOFF
        print("takeoff transition")

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        self.calculate_box()
        self.cmd_position(self.nextWP[0], self.nextWP[1], self.nextWP[2], self.nextWP[3])
        self.flight_phase = States.WAYPOINT
        
        print("waypoint transition")

    def landing_transition(self):
        self.land()
        self.flight_phase = States.LANDING
        print("landing transition")

    def disarming_transition(self):
        self.disarm()
        self.flight_phase = States.DISARMING
        print("disarm transition")

    def manual_transition(self):
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_phase = States.MANUAL
        print("manual transition")

    def start(self):
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
