import numpy as np
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
    # Imported lazily: geohash2 is only needed for this debug print and may fail to build
    import geohash2

    print('Geohash for 42.6, -5.6:', geohash2.encode(37.571309, 55.767190))

    lock_b = Lock()
    lock_r = Lock()

    # Start threads for parsing serial stream.
    Process(target=base_kalman, args=(lock_b,)).start()
