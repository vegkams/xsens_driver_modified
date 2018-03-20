#!/usr/bin/env python

##############################################################
#
#   Edited to make xsens publish yaw angle to topic 'state'
#
#   line 116 and 370
#################################################################

import roslib
import rospy
import select
import mtdevice
import math
import pdb
from serial.tools import list_ports

from std_msgs.msg import Header
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import QuaternionStamped
from gps_common.msg import GPSFix
from gps_common.msg import GPSStatus
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from custom_msgs.msg import sensorSample
from custom_msgs.msg import baroSample
from custom_msgs.msg import gnssSample
from custom_msgs.msg import ImuSensor
from custom_msgs.msg import positionEstimate
from custom_msgs.msg import velocityEstimate
from custom_msgs.msg import orientationEstimate
from custom_msgs.msg import UtcTime

# transform Euler angles or matrix into quaternions
from math import pi
from math import radians
from tf.transformations import quaternion_from_matrix
from tf.transformations import quaternion_from_euler
from tf.transformations import identity_matrix

import numpy

roslib.load_manifest('xsens_driver')

# Function that builds a GPRMC NMEA string from available GNSS data
def nmea_string(gnss_data):
    hour = gnss_data['hour']
    minute = gnss_data['min']
    second = gnss_data['sec']
    centisec = gnss_data['nano']*1e-7
    #print('microsec is %f\n' % microsec)
    #print('nano is %f\n' % gnss_data['nano'])
    lat = gnss_data['lat'] * 1e2
    lon = gnss_data['lon'] * 1e2
    sog = gnss_data['gSpeed']
    course = gnss_data['headMot']
    day = gnss_data['day']
    month = gnss_data['month']
    year = gnss_data['year'] % 100
    # Time stamp (UTC), Lat, Lon, SoG knots, True course, date stamp
    s = '$GPRMC,{:=02d}{:=02d}{:=02d}.{:=02d},A,{:=09.4f},N,{:=09.4f},E,{:=05.1f},{:=05.1f},{:=02d}{:=02d}{:=02d},0.0,E,A*'.format(hour,
        minute, second, abs(int(math.floor(centisec))), lat, lon, sog, course, day, month, year)
    return s + NMEA_CRC(s)


# Compute the NMEA checksum
def NMEA_CRC(input_string):
    myCopy = input_string[input_string.find('$')+1:input_string.find('*')]
    crc = ord(myCopy[0:1])
    for n in range(1,len(myCopy)):
        crc = crc ^ ord(myCopy[n:n+1])
    return '{:=02X}'.format(crc)

# Function to find the correct port for the xsens. This will only work for this device.
def find_xsens():
    ports = list_ports.comports()
    for port in ports:
        if port[0].startswith('/dev/ttyUSB'):
            if 'SER=077011CF' in port[2]:
                return(port[0])


def get_param(name, default):
    try:
        v = rospy.get_param(name)
        rospy.loginfo("Found parameter: %s, value: %s" % (name, str(v)))
    except KeyError:
        v = default
        rospy.logwarn("Cannot find value for parameter: %s, assigning "
                      "default: %s" % (name, str(v)))
    return v


class XSensDriver(object):

    ENU = numpy.identity(3)
    NED = numpy.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    NWU = numpy.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])

    def __init__(self):

        self.Status_Trigger_Active = 0x400000
        self.output_trigger = False
        self.pub_nmea = False
        self.prev_secs = 0
        #device = find_xsens()
        #device = 'auto'
        # ~ device = get_param('~device', 'auto')
        baudrate = get_param('~baudrate', 0)
        #if device == 'auto':
        devs = mtdevice.find_devices()
        if devs:
            device, baudrate = devs[0]
            rospy.loginfo("Detected MT device on port %s @ %d bps" % (device,
                                                                      baudrate))
        else:
            rospy.logerr("Fatal: could not find proper MT device.")
            rospy.signal_shutdown("Could not find proper MT device.")
            return
        if not baudrate:
            baudrate = mtdevice.find_baudrate(device)
        if not baudrate:
            rospy.logerr("Fatal: could not find proper baudrate.")
            rospy.signal_shutdown("Could not find proper baudrate.")
            return

        rospy.loginfo("MT node interface: %s at %d bd." % (device, baudrate))
        self.mt = mtdevice.MTDevice(device, baudrate)

        self.frame_id = get_param('~frame_id', '/mti/data')

        frame_local = get_param('~frame_local', 'NED')
        frame_local_imu = get_param('~frame_local_imu', 'NED')

        if frame_local == 'ENU':
            R = XSensDriver.ENU
        elif frame_local == 'NED':
            R = XSensDriver.NED
        elif frame_local == 'NWU':
            R = XSensDriver.NWU

        if frame_local_imu == 'ENU':
            R_IMU = XSensDriver.ENU
        elif frame_local_imu == 'NED':
            R_IMU = XSensDriver.NED
        elif frame_local_imu == 'NWU':
            R_IMU = XSensDriver.NWU

        self.R = R.dot(R_IMU.transpose())

        # ~ self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
        self.diag_msg = DiagnosticArray()
        self.stest_stat = DiagnosticStatus(name='mtnode: Self Test', level=1,
                                           message='No status information')
        self.xkf_stat = DiagnosticStatus(name='mtnode: XKF Valid', level=1,
                                         message='No status information')
        self.gps_stat = DiagnosticStatus(name='mtnode: GPS Fix', level=1,
                                         message='No status information')
        self.diag_msg.status = [self.stest_stat, self.xkf_stat, self.gps_stat]

        self.imu_pub = rospy.Publisher('xsens/imu', ImuSensor, queue_size=10)  # IMU message

        self.utc_pub = rospy.Publisher('xsens/utc_time', UtcTime, queue_size=10) # Utc time from imu
        # self.ss_pub = rospy.Publisher('xsens/sensor/sample', sensorSample, queue_size=10) # sensorSample
        # ~ self.mag_pub = rospy.Publisher('xsens/sensor/magnetic', Vector3Stamped, queue_size=10) # magnetic
        # ~ self.baro_pub = rospy.Publisher('xsens/sensor/pressure', baroSample, queue_size=10) # baro
        self.gnssPvt_pub = rospy.Publisher('xsens/gnssPvt', gnssSample, queue_size=10)  # GNSS PVT
        # self.gnssSatinfo_pub = rospy.Publisher('xsens/sensor/gnssStatus', GPSStatus, queue_size=10) # GNSS SATINFO
        # self.ori_pub = rospy.Publisher('xsens/filter/orientation', orientationEstimate, queue_size=10)
        # XKF/XEE orientation
        self.vel_pub = rospy.Publisher('xsens/velocity', velocityEstimate, queue_size=10)  # XKF/XEE velocity
        self.pos_pub = rospy.Publisher('xsens/position', positionEstimate, queue_size=10)  # XKF/XEE position
        # self.temp_pub = rospy.Publisher('temperature', Float32, queue_size=10) # decide type
        # ~ #################    EDITED #################################################################
        # ~
        # ~ self.yaw_pub = rospy.Publisher('state', Float64, queue_size=1)
        # ~
        # ~ ############################################################################################

    def spin(self):
        try:
            while not rospy.is_shutdown():
                self.spin_once()
        # Ctrl-C signal interferes with select with the ROS signal handler
        # should be OSError in python 3.?
        except select.error:
            pass

    def spin_once(self):

        def baroPressureToHeight(value):
            c1 = 44330.0
            c2 = 9.869232667160128300024673081668e-6
            c3 = 0.1901975534856
            intermediate = 1 - math.pow(c2 * value, c3)
            height = c1 * intermediate
            return height

        # get data
        data = self.mt.read_measurement()
        # common header
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = self.frame_id

        # common UTC time msg
        utc_time = UtcTime()
        pub_utc = False

        # get data (None if not present)
        # temp = data.get('Temp')  float
        orient_data = data.get('Orientation Data')
        velocity_data = data.get('Velocity')
        position_data = data.get('Latlon')
        altitude_data = data.get('Altitude')
        acc_data = data.get('Acceleration')
        gyr_data = data.get('Angular Velocity')
        mag_data = data.get('Magnetic')
        pressure_data = data.get('Pressure')
        time_data = data.get('Timestamp')
        gnss_data = data.get('GNSS')
        status = data.get('Status')  # int
        ''' NEW '''
        # Check status word for trigger indication
        if status is not None and 'StatusWord' in status:
            #print(status['StatusWord'])
            if status['StatusWord'] & self.Status_Trigger_Active == self.Status_Trigger_Active:
                self.output_trigger = True
                utc_time.triggerActive = True
        else:
            self.output_trigger = False
            utc_time.triggerActive = False

        # create messages and default values
        "Imu message supported with Modes 1 & 2"
        imu_msg = ImuSensor()
        pub_imu = False
        "SensorSample message supported with Mode 2"
        ss_msg = sensorSample()
        pub_ss = False
        "Mag message supported with Modes 1 & 2"
        mag_msg = Vector3Stamped()
        pub_mag = False
        "Baro in meters"
        baro_msg = baroSample()
        pub_baro = False
        "GNSS message supported only with MTi-G-7xx devices"
        "Valid only for modes 1 and 2"
        gnssPvt_msg = gnssSample()
        pub_gnssPvt = False
        # gnssSatinfo_msg = GPSStatus()
        # pub_gnssSatinfo = False
        # All filter related outputs
        "Supported in mode 3"
        ori_msg = orientationEstimate()
        pub_ori = False
        "Supported in mode 3 for MTi-G-7xx devices"
        vel_msg = velocityEstimate()
        pub_vel = False
        "Supported in mode 3 for MTi-G-7xx devices"
        pos_msg = positionEstimate()
        pub_pos = False

        # ORIGINAL TIME KEEPING##############################
        secs = 0
        nsecs = 0
        
        # Use sample time from imu (GNSS synchronized UTC)
        if time_data:
        # first getting the sampleTimeFine
            if 'Year' in time_data: # Found UTC time
                pub_utc = True
                utc_time.year = time_data['Year']
                utc_time.month = time_data['Month']
                utc_time.day = time_data['Day']
                utc_time.hour = time_data['Hour']
                utc_time.minute = time_data['Minute']
                utc_time.second = time_data['Second']
                utc_time.ns = time_data['ns']

            time = time_data['SampleTimeFine']
            secs = 100e-6*time
            nsecs = 1e5*time - 1e9*math.floor(secs)
            if self.prev_secs != int(math.floor(secs)):
                # UTC second rollover, publish NMEA
                self.pub_nmea = True
                self.prev_secs = int(math.floor(secs))
                #rospy.loginfo('secs is %d' % secs)
                #rospy.loginfo('prev_secs is %d' % self.prev_secs)


        # EDITED TIME KEEPING##################
        # ~ secs = 0
        # ~ nsecs = 0

        # ~ if time_data:
        # ~     time = rospy.Time.now()
        # ~     secs = time.secs
        # ~     nsecs = time.nsecs

        if acc_data:
            if 'Delta v.x' in acc_data:  # found delta-v's

                pub_imu = True
                imu_msg.dv.x = acc_data['Delta v.x']
                imu_msg.dv.y = acc_data['Delta v.y']
                imu_msg.dv.z = acc_data['Delta v.z']

                # pub_ss = True
                # ss_msg.internal.imu.dv.x = acc_data['Delta v.x']
                # ss_msg.internal.imu.dv.y = acc_data['Delta v.y']
                # ss_msg.internal.imu.dv.z = acc_data['Delta v.z']
            elif 'accX' in acc_data:  # found acceleration
                pub_imu = True
                imu_msg.linear_acceleration.x = acc_data['accX']
                imu_msg.linear_acceleration.y = acc_data['accY']
                imu_msg.linear_acceleration.z = acc_data['accZ']
            else:
                raise MTException("Unsupported message in XDI_AccelerationGroup.")

        if gyr_data:
            if 'Delta q0' in gyr_data:  # found delta-q's
                # pub_ss = True
                # ss_msg.internal.imu.dq.w = gyr_data['Delta q0']
                # ss_msg.internal.imu.dq.x = gyr_data['Delta q1']
                # ss_msg.internal.imu.dq.y = gyr_data['Delta q2']
                # ss_msg.internal.imu.dq.z = gyr_data['Delta q3']

                pub_imu = True
                imu_msg.dq.w = gyr_data['Delta q0']
                imu_msg.dq.x = gyr_data['Delta q1']
                imu_msg.dq.y = gyr_data['Delta q2']
                imu_msg.dq.z = gyr_data['Delta q3']
            elif 'gyrX' in gyr_data:  # found rate of turn
                pub_imu = True
                imu_msg.angular_velocity.x = gyr_data['gyrX']
                imu_msg.angular_velocity.y = gyr_data['gyrY']
                imu_msg.angular_velocity.z = gyr_data['gyrZ']
            else:
                raise MTException("Unsupported message in XDI_AngularVelocityGroup.")

        if mag_data:
            # magfield
            ss_msg.internal.mag.x = mag_msg.vector.x = mag_data['magX']
            ss_msg.internal.mag.y = mag_msg.vector.y = mag_data['magY']
            ss_msg.internal.mag.z = mag_msg.vector.z = mag_data['magZ']
            pub_mag = True

        if pressure_data:
            pub_baro = True
            height = baroPressureToHeight(pressure_data['Pressure'])
            baro_msg.height = ss_msg.internal.baro.height = height

        if gnss_data:
            pub_gnssPvt = True
            gnssPvt_msg.itow = gnss_data['itow']
            gnssPvt_msg.fix = gnss_data['fixtype']
            gnssPvt_msg.latitude = gnss_data['lat']
            gnssPvt_msg.longitude = gnss_data['lon']
            gnssPvt_msg.hEll = gnss_data['height']
            gnssPvt_msg.hMsl = gnss_data['hMSL']
            gnssPvt_msg.vel.x = gnss_data['velE']
            gnssPvt_msg.vel.y = gnss_data['velN']
            gnssPvt_msg.vel.z = -gnss_data['velD']
            gnssPvt_msg.hAcc = gnss_data['hAcc']
            gnssPvt_msg.vAcc = gnss_data['vAcc']
            gnssPvt_msg.sAcc = gnss_data['sAcc']
            gnssPvt_msg.pDop = gnss_data['pdop']
            gnssPvt_msg.hDop = gnss_data['hdop']
            gnssPvt_msg.vDop = gnss_data['vdop']
            gnssPvt_msg.numSat = gnss_data['numSV']
            gnssPvt_msg.heading = gnss_data['headMot']
            gnssPvt_msg.headingAcc = gnss_data['headAcc']

        if orient_data:
            if 'Q0' in orient_data:
                pub_imu = True
                imu_msg.dq.w = orient_data['Q0']
                imu_msg.dq.x = orient_data['Q1']
                imu_msg.dq.y = orient_data['Q2']
                imu_msg.dq.z = orient_data['Q3']
            elif 'Roll' in orient_data:
                # ~ pub_ori = True
                pub_imu = True
                # ~ ori_msg.roll = orient_data['Roll']
                # ~ ori_msg.pitch = orient_data['Pitch']
                # ~ ori_msg.yaw = orient_data['Yaw']
                imu_msg.orientation.roll = orient_data['Roll']
                imu_msg.orientation.pitch = orient_data['Pitch']
                imu_msg.orientation.yaw = orient_data['Yaw']
            else:
                raise MTException('Unsupported message in XDI_OrientationGroup')

        if velocity_data:
            pub_vel = True
            vel_msg.velE = velocity_data['velX']
            vel_msg.velN = velocity_data['velY']
            vel_msg.velU = velocity_data['velZ']

        if position_data:
            pub_pos = True
            pos_msg.latitude = position_data['lat']
            pos_msg.longitude = position_data['lon']

        if altitude_data:
            pub_pos = True
            tempData = altitude_data['ellipsoid']
            pos_msg.hEll = tempData[0]

        # if status is not None:
        #   if status & 0b0001:
        #       self.stest_stat.level = DiagnosticStatus.OK
        #       self.stest_stat.message = "Ok"
        #   else:
        #       self.stest_stat.level = DiagnosticStatus.ERROR
        #       self.stest_stat.message = "Failed"
        #   if status & 0b0010:
        #       self.xkf_stat.level = DiagnosticStatus.OK
        #       self.xkf_stat.message = "Valid"
        #   else:
        #       self.xkf_stat.level = DiagnosticStatus.WARN
        #       self.xkf_stat.message = "Invalid"
        #   if status & 0b0100:
        #       self.gps_stat.level = DiagnosticStatus.OK
        #       self.gps_stat.message = "Ok"
        #   else:
        #       self.gps_stat.level = DiagnosticStatus.WARN
        #       self.gps_stat.message = "No fix"
        #   self.diag_msg.header = h
        #   self.diag_pub.publish(self.diag_msg)

        # Publish velocity and position data all the time
        pub_vel = True
        pub_pos = True

        # publish available information
        if pub_imu:
            imu_msg.header = h
            # all time assignments (overwriting ROS time)
            # Comment the two lines below if you need ROS time
            imu_msg.header.stamp.secs = secs
            imu_msg.header.stamp.nsecs = nsecs
            imu_msg.utc = utc_time
            # NEW check if output trigger flag is set
            if self.output_trigger:
                imu_msg.trigger_indication = 1
            else:
                imu_msg.trigger_indication = 0
            self.imu_pub.publish(imu_msg)
        # if pub_gps:
        #   xgps_msg.header = gps_msg.header = h
        #   self.gps_pub.publish(gps_msg)
        #   self.xgps_pub.publish(xgps_msg)
        if pub_mag:
            mag_msg.header = h
            #self.mag_pub.publish(mag_msg)
        # if pub_temp:
        #   self.temp_pub.publish(temp_msg)
        if pub_ss:
            ss_msg.header = h
            # all time assignments (overwriting ROS time)
            # Comment the two lines below if you need ROS time
            ss_msg.header.stamp.secs = secs
            ss_msg.header.stamp.nsecs = nsecs
            self.ss_pub.publish(ss_msg)
        if pub_baro:
            baro_msg.header = h
            # all time assignments (overwriting ROS time)
            # Comment the two lines below if you need ROS time
            baro_msg.header.stamp.secs = secs
            baro_msg.header.stamp.nsecs = nsecs
            #self.baro_pub.publish(baro_msg)
        if pub_gnssPvt:
            gnssPvt_msg.header = h
            # all time assignments (overwriting ROS time)
            # Comment the two lines below if you need ROS time
            baro_msg.header.stamp.secs = secs
            baro_msg.header.stamp.nsecs = nsecs
            self.gnssPvt_pub.publish(gnssPvt_msg)
        if pub_ori:
            ori_msg.header = h
            # all time assignments (overwriting ROS time)
            # Comment the two lines below if you need ROS time
            ori_msg.header.stamp.secs = secs
            ori_msg.header.stamp.nsecs = nsecs
            self.ori_pub.publish(ori_msg)
            # ~ #################    EDITED #################################################################
            # ~
            # ~ self.yaw_pub.publish(ori_msg.yaw)
            # ~
            # ~ ############################################################################################
        if pub_vel:
            vel_msg.header = h
            # all time assignments (overwriting ROS time)
            # Comment the two lines below if you need ROS time
            vel_msg.header.stamp.secs = secs
            vel_msg.header.stamp.nsecs = nsecs
            self.vel_pub.publish(vel_msg)
        if pub_pos:
            pos_msg.header = h
            # all time assignments (overwriting ROS time)
            # Comment the two lines below if you need ROS time
            pos_msg.header.stamp.secs = secs
            pos_msg.header.stamp.nsecs = nsecs
            self.pos_pub.publish(pos_msg)
        # TODO build nmea string from available data, send over serial line
        if self.pub_nmea and gnss_data:
            gprmc_string = nmea_string(gnss_data)
            #rospy.loginfo(gprmc_string)
            self.pub_nmea = False

        if pub_utc:
            self.utc_pub.publish(utc_time)


def main():
    '''Create a ROS node and instantiate the class.'''
    rospy.init_node('xsens_driver')
    driver = XSensDriver()
    driver.spin()


if __name__ == '__main__':
    main()
