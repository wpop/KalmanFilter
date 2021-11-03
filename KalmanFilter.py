import numpy as np
import geohash2
import queue
import time
import logging

# from threading import Thread
from multiprocessing import Process, Lock

import IMU_GPS_Data as data
import GPS_Parser as prs

queue = queue.Queue(2)  # Global queue

logging.basicConfig(level=logging.DEBUG,
                    format='[%(levelname)s] (%(threadName)-10s) %(message)s',
                    )


# class Rover(Thread):
class Rover:
    """This class defines rover coordinates"""
    def __init__(self, name):
        # Thread.__init__(self)
        self.port = 5556
        self.msg = None
        self.time = None
        self.name = name
        self.rover = data.Sensors()
        self.parser = prs.Parser(self.port)
        self.filter = Filter()
        self.object = []

    def run_kalman(self, lock):

        logging.debug(f'Got some data for the Kalman Rover {len(self.object)}')

        while True:
            # Get data from the parser
            self.object = self.parser.get_data()

            logging.debug(f'Got some data for the Kalman Rover {len(self.object)}')

            # Wait since we only get a complete frame in 1s.
            time.sleep(0.1)

    def output(self, data):
        for indx, val in enumerate(data):
            print(indx, len(data))
            # print (val)
            s = f'deque indx: {indx}\t time: {val.time}-------------------------------->\n' \
                f'latitude:  {val.latitude}\n' \
                f'longitude: {val.longitude}\n' \
                f'pdop: {val.pdop}\n' \
                f'hdop: {val.hdop}\n' \
                f'vdop: {val.vdop}\n' \
                f'altitude: {val.altitude}\n' \
                f'vel_knot: {val.vel_knot}\n' \
                f'sv_prn_num_gps: {val.sv_prn_num_gps}\n' \
                f'elevation_deg_gps: {val.elevation_deg_gps}\n' \
                f'azimuth_gps: {val.azimuth_gps}\n' \
                f'snr_gps: {val.snr_gps}\n' \
                f'sv_prn_num_glonass: {val.sv_prn_num_glonass}\n' \
                f'elevation_deg_glonass: {val.elevation_deg_glonass}\n' \
                f'azimuth_glonass: {val.azimuth_glonass}\n' \
                f'snr_glonass: {val.snr_glonass}\n' \
                f'acceleration_x: {val.acceleration_x}\t acceleration_y: {val.acceleration_y}\t acceleration_z: {val.acceleration_z}\n' \
                f'gyr_x: {val.gyr_x}\t gyr_y: {val.gyr_y}\t gyr_z: {val.gyr_z}\n' \
                f'mag_pole_x: {val.dir_x}\t mag_pole_y: {val.dir_y}\t mag_pole_z: {val.dir_z}\n' \
                f'end deque indx {indx} : port {self.port} <--------------------------------\n'
            print(s)

    def get_data(self):
        pass

    def get_thread_id(self):
        pass

    def run(self):
        logging.debug(f'Got some data for the Kalman Rover {len(self.object)}')

        while True:
            # Get data from the parser
            self.object = self.parser.get_data()

            logging.debug(f'Got some data for the Kalman Rover {len(self.object)}')

            # Wait since we only get a complete frame in 1s.
            time.sleep(0.1)

# class Stationary(Thread):
class Stationary:
    """This class defines base coordinates"""
    def __init__(self, name):
        # Thread.__init__(self)
        self.port = 5555
        self.msg =  None
        self.time = None
        self.name = name
        self.base = data.Sensors()
        self.parser = prs.Parser(self.port)
        self.myfilter = Filter()
        self.object = []

    def output(self, data):
        for indx, val in enumerate(data):
            print(indx, len(data))
            # print (val)
            s = f'deque indx: {indx}\t time: {val.time}-------------------------------->\n' \
                f'latitude:  {val.latitude}\n' \
                f'longitude: {val.longitude}\n' \
                f'pdop: {val.pdop}\n' \
                f'hdop: {val.hdop}\n' \
                f'vdop: {val.vdop}\n' \
                f'altitude: {val.altitude}\n' \
                f'vel_knot: {val.vel_knot}\n' \
                f'sv_prn_num_gps: {val.sv_prn_num_gps}\n' \
                f'elevation_deg_gps: {val.elevation_deg_gps}\n' \
                f'azimuth_gps: {val.azimuth_gps}\n' \
                f'snr_gps: {val.snr_gps}\n' \
                f'sv_prn_num_glonass: {val.sv_prn_num_glonass}\n' \
                f'elevation_deg_glonass: {val.elevation_deg_glonass}\n' \
                f'azimuth_glonass: {val.azimuth_glonass}\n' \
                f'snr_glonass: {val.snr_glonass}\n' \
                f'acceleration_x: {val.acceleration_x}\t acceleration_y: {val.acceleration_y}\t acceleration_z: {val.acceleration_z}\n' \
                f'gyr_x: {val.gyr_x}\t gyr_y: {val.gyr_y}\t gyr_z: {val.gyr_z}\n' \
                f'mag_pole_x: {val.dir_x}\t mag_pole_y: {val.dir_y}\t mag_pole_z: {val.dir_z}\n' \
                f'end deque indx {indx} : port {self.port} <--------------------------------\n'
            print(s)

    def get_data_from_parser(self):
        return self.parser.get_data()

    def run_kalman(self, lock):

        while True:
            self.parser.parser_gps(lock)

            self.object = self.parser.get_data(lock)

            logging.debug(f'Got some data for the Kalman Base {len(self.object)}')

    def process_data(self, lock):
        logging.debug(f'Got some data for the Kalman Base {len(self.object)}')

    def run(self):
        pass
        #self.t = Thread(target=self.parser.parser_gps(),
        #                args=())
        # self.daemon = True
        # self.start()
        # self.t.start(self)
        # self.t.setDaemon(True)


class Filter:
    """TODO: in this class we compute position by using Kalman Filter"""
    def __init__(self):
        self.Xk = None  # System state vector
        self.Fk = None  # State-transition model
        self.Bk = None  # Control-input mode
        self.uk = None  # Control Vector
        self.Hk = None  # Observation model
        self.Rk = None  # Covariance of observation noise
        self.Qk = None  # Covariance of noise process

    def get_data(self, obj):
        pass

    def set_state_vector(self, x, y, vx, vy):
        """Establishe Xk -- X, Y coordinates and X', Y' velocities"""
        self.Xk = np.array(x, y, vx, vy).reshape(4, 1)  # vertical

    def set_transition_matrix(self, vel: float):
        self.Fk = np.eye(4, 4)
        self.Fk[0][2] = vel
        self.Fk[1][3] = vel

    def set_control_matrix(self, vel: float, acc: float):
        self.Bk = np.zeros((4, 2))
        self.Bk[0][0] = acc
        self.Bk[1][1] = acc
        self.Bk[2][0] = vel
        self.Bk[3][1] = vel

    def set_control_vector(self, acc: np.array):
        self.uk = np.zeros((2, 1))
        self.uk[0] = acc[0]
        self.uk[1] = acc[1]

    def observation_model(self, flag_signal: bool):
        if flag_signal == True:
            self.Hk = np.eye(4, 4)
        else:
            self.Hk = np.eye(2, 4)

    def set_covariance_matrix(self, sigma_pos: float, sigma_vel: bool, vel_flag: bool):
        if vel_flag == True:
            self.Rk = np.eye(4, 4) * sigma_pos
        else:
            self.Rk = np.eye(4, 4)
            self.Rk[0][0] = sigma_pos
            self.Rk[1][1] = sigma_pos
            self.Rk[2][2] = sigma_vel
            self.Rk[3][3] = sigma_vel

    def set_covariance_noise(self, sigma_acc: np.array, delta_time: float):
        sigma_vel_x = sigma_acc[0] * delta_time
        sigma_vel_y = sigma_acc[1] * delta_time

        sigma_pos_x = sigma_acc[0] * np.power(delta_time, 2) * 0.5
        sigma_pos_y = sigma_acc[1] * np.power(delta_time, 2) * 0.5

        self.Qk = np.eye(4, 4)

        self.Qk[0][0] = np.power(sigma_pos_x, 2)
        self.Qk[0][2] = sigma_pos_x * sigma_vel_x
        self.Qk[1][1] = np.power(sigma_pos_y, 2)
        self.Qk[1][3] = sigma_pos_y * sigma_vel_y
        self.Qk[2][2] = np.power(sigma_vel_x, 2)
        self.Qk[3][3] = np.power(sigma_vel_y, 2)

    def proceed(self):
        pass


rover = Rover("Rover")
base = Stationary("Stationary")


def start_my_thread(name):
    if(name == "rover_parser"):
        rover.parser.parser_gps()
        print("ROVER")
    if name == "base_parser":
        base.parser.parser_gps()
        print("BASE")

# Helper functions to start threads for read and processing
def rover_parser(l):
    logging.debug('Starting')
    rover.parser.parser_gps(l)
    logging.debug('Exiting')


def base_parser(l):
    logging.debug('Starting Base Parsers')
    base.parser.parser_gps(l)
    logging.debug('Exiting')


def rover_kalman(l):
    logging.debug('Starting Rover Kalman')
    rover.run_kalman(l)
    logging.debug('Exiting')


def base_kalman(l):
    logging.debug('Starting Base Kalman')
    base.run_kalman(l)
    logging.debug('Exiting')


def read_base(l):
    logging.debug('Starting read base')
    data_base = base.parser.get_data(l)
    base.output(data_base)
    logging.debug('Exiting from base reading')


def read_rover(l):
    logging.debug('Starting read rover')
    data_rover = rover.parser.get_data(l)
    rover.output(data_rover)
    logging.debug('Exiting from rover reading')


if __name__ == "__main__":
    print('Geohash for 42.6, -5.6:', geohash2.encode(37.571309, 55.767190))

    lock_b = Lock()
    lock_r = Lock()

    # Start threads for parsing serial stream.
    Process(target=base_kalman, args=(lock_b,)).start()
