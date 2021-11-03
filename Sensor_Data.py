# import pynmea2
# import collections
# import socket
import datetime
import re
import copy
#from datetime import tzinfo


class Sensors:
    def __init__(self):
        self.NMEA_IGNORE_PREFIXES = ["$PGLOR", "$QZGSV"]

        # IMU Data
        self.acceleration_x = 0.0       # Acceleration from IMU
        self.acceleration_y = 0.0
        self.acceleration_z = 0.0
        self.gyr_x = None               # Gyroscope angular velocities
        self.gyr_y = None
        self.gyr_z = None
        self.dir_x = 0.0                # Direction to Magnetic Pole through X-axes
        self.dir_y = None
        self.dir_z = None

        self.nmea_msg = None             # Read NMEA message. In our application we use ';' to separate values.
        self.date = None
        self.time = None                 # e.g. Format taken at 12:35:19 UTC
        self.latitude = []            # latitude
        # self.lat_dir = None              # latitude direction
        self.longitude = None            # longitude
        # self.lon_dir = None              # longitude direction
        self.num_satellites = None       # Number of satellites being tracked
        self.pdop = None                 # Dilution of position
        self.hdop = None                 # Horizontal dilution of precision (HDOP)
        self.vdop = None                 # Vertical dilution of precision (VDOP)
        self.altitude = None             # Altitude, Meters, above mean sea level
        self.vel_knot = []               # Ground speed, knots
        # self.vel_km = []                 # Ground speed, Kilometers per hour

        self.num_sv_in_view = None       # Total number of satellites sending info
        self.sv_prn_num_gps = []         # number GPS of satellites used for fix
        self.sv_prn_num_glonass = []     # number GLONASS of satellites used for fix
        self.elevation_deg_gps = []      # data gained from GPS America
        self.elevation_deg_glonass = []  # data gained from GLONASS Russia
        self.azimuth_gps = []            # azimuth angles gained from gps
        self.azimuth_glonass = []        # azimuth angles gained from GLONASS
        self.snr_gps = []                # signal strength from gps sputniks
        self.snr_glonass = []            # signal strength from GLONASS sputniks
        # self.available = None
        # self.lat_degrees = None

    # def lat_list(self, lat):
    #     """Gain the list of latitudes converted from GPS standart into decimal degrees"""
    #     # print("lat_list:", lat)
    #     latitudes = []
    #     for l in lat:
    #         val = self.convert_lat_to_deg(l)
    #         latitudes.append(val)
    #     self.lat_degrees = copy.copy(latitudes)
    #
    # def convert_lat_to_deg(self, lat):
    #     """This method convert latitude into decimal degree."""
    #     print("latitudes: ", type(lat), lat)
    #     deg, minutes, seconds, direction = re.split('[°\′″]', lat)
    #     val = (float(deg) + float(minutes) / 60 + float(seconds) / (60 * 60)) * (-1 if direction in ['W', 'S'] else 1)
    #     return val

    def set_time(self, msg):
        """extract times"""
        self.time = msg.timestamp
        self.time = self.time.replace(tzinfo=datetime.timezone.utc)  # gmt0

    def set_vel_knots(self, msg):
        self.vel_knot = msg.spd_over_grnd

    def set_date(self, msg):
        """extract date: year-month-day"""
        self.date = msg.datestamp

    def set_latitude(self, msg):
        self.latitude = '%02d°%02d′%07.4f″%s' % (msg.latitude,
                                               msg.latitude_minutes, msg.latitude_seconds, msg.lat_dir)

    def set_longitude(self, msg):
        """Extract Longitude"""
        self.longitude = '%02d°%02d′%07.4f″%s' % (msg.longitude,
                                                msg.longitude_minutes, msg.longitude_seconds, msg.lon_dir)

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
        self.altitude = msg.altitude

    def set_pdop(self, msg):
        """extract dilution of precision"""
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
        if identifier == 'GPGSV':
            self.elevation_deg_gps.extend(temp)
        else:
            self.elevation_deg_glonass.extend(temp)  # add glonass data set

    def set_prn_num(self, msg, identifier):
        """extract the particular number of satellite in view available in the sentence"""
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
