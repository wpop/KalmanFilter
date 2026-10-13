import collections
import pynmea2
# import threading
import time
import logging
import copy


from multiprocessing import Process, Lock, Queue

import Sensor_Data as sen
import connection as cn

logging.basicConfig(level=logging.DEBUG,
                    format='[%(levelname)s] (%(threadName)-10s) %(message)s',
                    )


class Parser(sen.Sensors):
    def __init__(self, port):
        sen.Sensors.__init__(self)
        # self.lock = Lock()
        # self.lock = threading.Lock()
        self.frames = collections.deque()  # We keep the last 30 sentences in deque
        self.nmea_message = None
        self.port = port

        self.KNOT = 1.852                                 # to concert velocity into kp/hour
        self.NUMQUEELEMENTS = 31                          # Maximum Number of element in Deque; deque.size == 30
        self.NMEA_IGNORE_PREFIXES = ["$PGLOR", "$QZGSV"]  # prefixes to be ignored

        self.sensors = collections.deque()
        self.connection_mgr = cn.Connection(self.port)

        self.current = pynmea2.datetime.time(0, 0, 1)

        self.new_frame = sen.Sensors()

    def get_data(self, l):
        with l:
            fix_data = copy.copy(self.frames)
        # traceback.print_exc()
        # Remove the current working frame (new_frame) since it contains incomplete data.
        if fix_data:
            fix_data.popleft()
        logging.debug(f'Get Data? {len(self.frames)}')
        return fix_data

    def process_data(self, lock):
        logging.debug('GPS Parser Version')

    def output(self):
        for indx, val in enumerate(self.frames):
            print(indx)
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

    def _is_new_epoch(self, timestamp):
        """True if a sentence timestamp differs from the one of the current frame."""
        return timestamp is not None and (not self.frames or timestamp != self.current)

    def _start_frame(self, l):
        """Create a new frame, make it the current one and trim the queue to its size limit."""
        logging.debug(f'A New frame {len(self.frames)}')
        self.new_frame = sen.Sensors()
        with l:
            self.frames.appendleft(self.new_frame)

        if len(self.frames) > self.NUMQUEELEMENTS:
            logging.debug('popping')
            with l:
                self.frames.pop()

    def parser_gps(self, l):
        """
        Receive one UDP packet and write its sentences and IMU values into the current frame.

        Assumption: a frame starts with the first sentence that carries a timestamp (RMC or GGA)
        that differs from the timestamp of the current frame. The frame is created when that
        sentence is seen, before its data is stored, and its time is taken from that sentence.
        A later RMC or GGA with the same timestamp does not start a new frame. All other
        sentences (GSA, GSV, ...) and IMU values belong to the current frame until the next
        timestamp change. Data that arrives before the very first RMC or GGA cannot be
        attributed to any epoch; it is written into a placeholder frame that is not stored in
        self.frames and is discarded. Sentences of an epoch that arrive before its first
        RMC or GGA (e.g. GSA first) would be attributed to the previous epoch.
        """
        time.sleep(0.001)

        try:
            logging.debug(len(self.frames))
            # this is blocking.  It waits until a message is received.
            message, address = self.connection_mgr.m_socket.recvfrom(8192)
            logging.debug(message)

            if message is None:
                logging.debug("Empty - Check connection to phone")
            else:
                msg = message.decode("utf-8")                         # convert from bytes into a string format
                msg_list = msg.rstrip('\n').split(',')                # convert from string into list
                # TODO: NMEA message is using the same date time code. Update existing IMU object
                n = 1
                while (n + 3) < len(msg_list):
                    # print("msg.name_to_idx: ", type(msg))
                    if 1 == int(msg_list[n]):
                        msg_nmea = msg_list[n + 1].lstrip()                 # Remove spaces to the left of the string
                        msg_nmea = [x.replace('\n', '') for x in msg_nmea]  # TODO: Victor has to remove '\n' in JAVA
                        msg_nmea = [x.replace(';', ',') for x in msg_nmea]  # Replace delimeter in GPA - string
                        msg_nmea_str = "".join(msg_nmea)                    # Convert back to string

                        # Cellphone insert own messages. Eleminate these sentences
                        if not any(msg_nmea_str.find(s) >= 0 for s in self.new_frame.NMEA_IGNORE_PREFIXES):
                            parser = pynmea2.parse(msg_nmea_str)
                            logging.debug(f'parser: {parser.name_to_idx}')
                            ident = parser.identifier().strip(',')  # Use the identifier to chose GPS or GLONASS

                            if (type(parser) in (pynmea2.types.talker.RMC, pynmea2.types.talker.GGA)
                                    and self._is_new_epoch(parser.timestamp)):
                                self._start_frame(l)
                                self.current = parser.timestamp
                                self.new_frame.set_time(parser)

                            if type(parser) is pynmea2.types.talker.RMC:
                                # print("msg.name_to_idx: ", msg.name_to_idx)
                                self.new_frame.set_time(parser)
                                # self.new_frame.time = parser.timestamp
                                self.new_frame.set_latitude(parser)
                                self.new_frame.set_longitude(parser)

                                self.new_frame.set_date(parser)
                                self.new_frame.set_vel_knots(parser)

                            if type(parser) is pynmea2.types.talker.GGA:
                                self.new_frame.set_altitude(parser)

                            if type(parser) is pynmea2.types.talker.GSA:
                                self.new_frame.set_pdop(parser)
                                self.new_frame.set_hdop(parser)
                                self.new_frame.set_vdop(parser)

                            if type(parser) is pynmea2.types.talker.GSV:
                                self.new_frame.set_num_sv_in_view(parser)
                                self.new_frame.set_prn_num(parser, ident)
                                self.new_frame.set_elevation(parser, ident)
                                self.new_frame.set_azimuth(parser, ident)
                                self.new_frame.set_snr(parser, ident)
                                self.new_frame.set_available_info()

                    if 3 == int(msg_list[n]):
                        a = msg_list[n + 1:n + 4]
                        self.new_frame.acceleration_x = a[0]
                        self.new_frame.acceleration_y = a[1]
                        self.new_frame.acceleration_z = a[2]

                    if 4 == int(msg_list[n]):
                        g = msg_list[n + 1:n + 4]
                        self.new_frame.gyr_x = g[0]
                        self.new_frame.gyr_y = g[1]
                        self.new_frame.gyr_z = g[2]

                    if 5 == int(msg_list[n]):
                        m = msg_list[n + 1:n + 4]
                        self.new_frame.dir_x = m[0]
                        self.new_frame.dir_y = m[1]
                        self.new_frame.dir_z = m[2]

                    n = n + 4  # update substring position here

                    if len(self.sensors) > 0:
                        logging.debug("Data!")

        except (KeyboardInterrupt, SystemExit):
            raise
        except Exception:
            logging.exception('Failed to receive or parse a message on port %s', self.port)


if __name__ == "__main__":
    gps = Parser(5555)
    # gps.parser_gps()
    # gps.output()