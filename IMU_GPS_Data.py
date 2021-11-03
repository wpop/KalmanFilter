import pynmea2
import collections
import socket


class Sensors:
    def __init__(self):
        # self.frames = collections.deque()  # We keep the last 30 sentences in deque
        # self.message = None
        self.NMEA_IGNORE_PREFIXES = ["$PGLOR", "$QZGSV"]


        self.acceleration_x = 0.0
        self.acceleration_y = 0.0
        self.acceleration_z = 0.0
        self.gyr_x = None               # Gyroscope X-slope
        self.gyr_y = None
        self.gyr_z = None
        self.dir_x = 0.0               # Direction to Magnetic Pole through X-axes
        self.dir_y = None
        self.dir_z = None

        self.nmea_msg = None             # Read NMEA message. In our application we use ';' to separate values.
        self.date = None
        self.time = None                 # e.g. Format taken at 12:35:19 UTC
        self.latitude = None             # latitude
        self.longitude = None            # longitude
        self.num_satellites = None       # Number of satellites being tracked
        self.pdop = None                 # Dilution of position
        self.hdop = None                 # Horizontal dilution of precision (HDOP)
        self.vdop = None                 # Vertical dilution of precision (VDOP)
        self.altitude = None             # Altitude, Meters, above mean sea level
        self.vel_knot = []               # Ground speed, knots
        # self.vel_km = []                 # Ground speed, Kilometers per hour
        self.num_sv_in_view = None
        self.sv_prn_num_gps = []
        self.sv_prn_num_glonass = []
        self.elevation_deg_gps = []      # data gained from GPS America
        self.elevation_deg_glonass = []  # data gained from GLONASS Russia
        self.azimuth_gps = []
        self.azimuth_glonass = []
        self.snr_gps = []
        self.snr_glonass = []
        self.available = None


    def set_time(self, msg):
        """extract times"""
        self.time = msg.timestamp

    def set_vel_knots(self, msg):
        self.vel_knot = msg.spd_over_grnd

    def set_date(self, msg):
        """extract date: year-month-day"""
        self.date = msg.datestamp

    def set_latitude(self, msg):
        self.latitude = '%02d°%02d′%07.4f″' % (msg.latitude,
                                               msg.latitude_minutes, msg.latitude_seconds)

    def set_longitude(self, msg):
        """Extract Longitude"""
        self.longitude = '%02d°%02d′%07.4f″' % (msg.longitude,
                                                msg.longitude_minutes, msg.longitude_seconds)

    def set_azimuth(self, msg, identifier):
        """extract all azimuths available in the sentence"""
        arr = msg.data[5:]
        temp = [arr[index] for index in range(len(arr)) if index % 4 == 0]
        if identifier == 'GPGSV':
            self.azimuth_gps.extend(temp)
        else:
            self.azimuth_glonass.extend(temp)

    def set_altitude(self, msg):
        """Altitude, Meters, above mean sea level"""
        # print('Tracing info: ', self.msg.data[8])
        self.altitude = msg.altitude

    def set_pdop(self, msg):
        """extract dilution of precision"""
        # print('Tracing info: ', self.msg.data[14])
        self.pdop = msg.pdop

    def set_hdop(self, msg):
        """Instantinate Horizontal dilution of precision"""
        self.hdop = msg.hdop

    def set_vdop(self, msg):
        """Set Vertical dilution of precision """
        self.vdop = msg.vdop

    def set_elevation(self, msg, identifier):
        """extract all elevation_deg available in the sentence"""
        arr = msg.data[4:]
        # print([arr[index] for index in range(len(arr)) if index % 4 == 0])
        temp = [arr[index] for index in range(len(arr)) if index % 4 == 0]
        # del self.elevation_deg[:]
        if identifier == 'GPGSV':
            self.elevation_deg_gps.extend(temp)
            # self.elevation_deg_gps.insert(0, temp)
        else:
            self.elevation_deg_glonass.extend(temp)  # add glonass data set


    def set_prn_num(self, msg, identifier):
        """extract the particular number of satellite in view available in the sentence"""
        # ident = msg.identifier().strip(',')
        # print("--------", msg.data)
        arr = msg.data[3:]
        if identifier == 'GPGSV':
            self.sv_prn_num_gps.extend([arr[index] for index in range(len(arr)) if index % 4 == 0])
        else:
            self.sv_prn_num_glonass.extend([arr[index] for index in range(len(arr)) if index % 4 == 0])

    # def set_prn_num(self, msg):
    #     """extract the particular number of satellite in view available in the sentence"""
    #     # print("--------", msg.data)
    #     arr = msg.data[3:]
    #     self.sv_prn_num = [arr[index] for index in range(len(arr)) if index % 4 == 0]

    def set_num_sv_in_view(self, msg):
        """Total number of satellites in view"""
        self.num_sv_in_view = msg.num_sv_in_view

    def set_available_info(self):
        """Return the number of satelites that provide info"""
        self.available = len(self.azimuth_gps + self.azimuth_glonass)

    def set_snr(self, msg, identifier):
        """extract all SNR available in the sentence; higher is better"""
        arr = msg.data[6:]
        temp = [arr[index] for index in range(len(arr)) if index % 4 == 0]
        if identifier == 'GPGSV':
            self.snr_gps.extend(temp)
        else:
            self.snr_glonass.extend(temp)
