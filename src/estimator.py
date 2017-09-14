#!/usr/bin/env python

# This node estimates the robot position, relative to a map of markers

from __future__ import division

import time
import json
import math
import numpy
import rospy
import tf
from tf import transformations as t
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from raspcv.msg import Marker, MarkerArray
from util import orientation_from_quaternion, orientation_from_euler


rospy.init_node('marker_position_estimator')
rospy.loginfo('Inited marker position estimator')


# TODO: broadcast initial local_origin -> marker_map


SQRT_2 = math.sqrt(2)

FOV = math.radians(rospy.get_param('~camera_fov'))
MAP_OFFSET = json.loads(rospy.get_param('~map_offset'))
CAMERA_OFFSET = json.loads(rospy.get_param('~camera_offset'))
POSITION_OFFSET = rospy.get_param('~position_offset')
PUBLISH_MAVLINK = rospy.get_param('~publish_mavlink')


FOV1 = math.radians(239)
FOV2 = math.radians(140)

counter = 0

# Grid marker map generation
#
# MARKER_SIZE = 0.223
# MARKER_X_DIST = 0.8
# MARKER_Y_DIST = 0.8
# MARKER_COUNT = 120
# 
# MARKERS_MAP = {str(i): {"x": round(MARKER_X_DIST * (i / 10), 6),
#                         "y": round(MARKER_Y_DIST * (i % 10), 6),
#                         "yaw": math.pi / 2,
#                         "z": 0,
#                         "size": MARKER_SIZE} for i in xrange(MARKER_COUNT)}

MARKERS_MAP = {'0':   {'size': 0.223, 'x': 0.0, 'y': 0.0, 'yaw': math.pi / 2, 'z': 0},
               '1':   {'size': 0.223, 'x': 0.0, 'y': 0.8, 'yaw': math.pi / 2, 'z': 0},
               '10':  {'size': 0.223, 'x': 0.8, 'y': 0.0, 'yaw': math.pi / 2, 'z': 0},
               '100': {'size': 0.223, 'x': 8.0, 'y': 0.0, 'yaw': math.pi / 2, 'z': 0},
               '101': {'size': 0.223, 'x': 8.0, 'y': 0.8, 'yaw': math.pi / 2, 'z': 0},
               '102': {'size': 0.223, 'x': 8.0, 'y': 1.6, 'yaw': math.pi / 2, 'z': 0},
               '103': {'size': 0.223, 'x': 8.0, 'y': 2.4, 'yaw': math.pi / 2, 'z': 0},
               '104': {'size': 0.223, 'x': 8.0, 'y': 3.2, 'yaw': math.pi / 2, 'z': 0},
               '105': {'size': 0.223, 'x': 8.0, 'y': 4.0, 'yaw': math.pi / 2, 'z': 0},
               '106': {'size': 0.223, 'x': 8.0, 'y': 4.8, 'yaw': math.pi / 2, 'z': 0},
               '107': {'size': 0.223, 'x': 8.0, 'y': 5.6, 'yaw': math.pi / 2, 'z': 0},
               '108': {'size': 0.223, 'x': 8.0, 'y': 6.4, 'yaw': math.pi / 2, 'z': 0},
               '109': {'size': 0.223, 'x': 8.0, 'y': 7.2, 'yaw': math.pi / 2, 'z': 0},
               '11':  {'size': 0.223, 'x': 0.8, 'y': 0.8, 'yaw': math.pi / 2, 'z': 0},
               '110': {'size': 0.223, 'x': 8.8, 'y': 0.0, 'yaw': math.pi / 2, 'z': 0},
               '111': {'size': 0.223, 'x': 8.8, 'y': 0.8, 'yaw': math.pi / 2, 'z': 0},
               '112': {'size': 0.223, 'x': 8.8, 'y': 1.6, 'yaw': math.pi / 2, 'z': 0},
               '113': {'size': 0.223, 'x': 8.8, 'y': 2.4, 'yaw': math.pi / 2, 'z': 0},
               '114': {'size': 0.223, 'x': 8.8, 'y': 3.2, 'yaw': math.pi / 2, 'z': 0},
               '115': {'size': 0.223, 'x': 8.8, 'y': 4.0, 'yaw': math.pi / 2, 'z': 0},
               '116': {'size': 0.223, 'x': 8.8, 'y': 4.8, 'yaw': math.pi / 2, 'z': 0},
               '117': {'size': 0.223, 'x': 8.8, 'y': 5.6, 'yaw': math.pi / 2, 'z': 0},
               '118': {'size': 0.223, 'x': 8.8, 'y': 6.4, 'yaw': math.pi / 2, 'z': 0},
               '119': {'size': 0.223, 'x': 8.8, 'y': 7.2, 'yaw': math.pi / 2, 'z': 0},
               '12':  {'size': 0.223, 'x': 0.8, 'y': 1.6, 'yaw': math.pi / 2, 'z': 0},
               '13':  {'size': 0.223, 'x': 0.8, 'y': 2.4, 'yaw': math.pi / 2, 'z': 0},
               '14':  {'size': 0.223, 'x': 0.8, 'y': 3.2, 'yaw': math.pi / 2, 'z': 0},
               '15':  {'size': 0.223, 'x': 0.8, 'y': 4.0, 'yaw': math.pi / 2, 'z': 0},
               '16':  {'size': 0.223, 'x': 0.8, 'y': 4.8, 'yaw': math.pi / 2, 'z': 0},
               '17':  {'size': 0.223, 'x': 0.8, 'y': 5.6, 'yaw': math.pi / 2, 'z': 0},
               '18':  {'size': 0.223, 'x': 0.8, 'y': 6.4, 'yaw': math.pi / 2, 'z': 0},
               '19':  {'size': 0.223, 'x': 0.8, 'y': 7.2, 'yaw': math.pi / 2, 'z': 0},
               '2':   {'size': 0.223, 'x': 0.0, 'y': 1.6, 'yaw': math.pi / 2, 'z': 0},
               '20':  {'size': 0.223, 'x': 1.6, 'y': 0.0, 'yaw': math.pi / 2, 'z': 0},
               '21':  {'size': 0.223, 'x': 1.6, 'y': 0.8, 'yaw': math.pi / 2, 'z': 0},
               '22':  {'size': 0.223, 'x': 1.6, 'y': 1.6, 'yaw': math.pi / 2, 'z': 0},
               '23':  {'size': 0.223, 'x': 1.6, 'y': 2.4, 'yaw': math.pi / 2, 'z': 0},
               '24':  {'size': 0.223, 'x': 1.6, 'y': 3.2, 'yaw': math.pi / 2, 'z': 0},
               '25':  {'size': 0.223, 'x': 1.6, 'y': 4.0, 'yaw': math.pi / 2, 'z': 0},
               '26':  {'size': 0.223, 'x': 1.6, 'y': 4.8, 'yaw': math.pi / 2, 'z': 0},
               '27':  {'size': 0.223, 'x': 1.6, 'y': 5.6, 'yaw': math.pi / 2, 'z': 0},
               '28':  {'size': 0.223, 'x': 1.6, 'y': 6.4, 'yaw': math.pi / 2, 'z': 0},
               '29':  {'size': 0.223, 'x': 1.6, 'y': 7.2, 'yaw': math.pi / 2, 'z': 0},
               '3':   {'size': 0.223, 'x': 0.0, 'y': 2.4, 'yaw': math.pi / 2, 'z': 0},
               '30':  {'size': 0.223, 'x': 2.4, 'y': 0.0, 'yaw': math.pi / 2, 'z': 0},
               '31':  {'size': 0.223, 'x': 2.4, 'y': 0.8, 'yaw': math.pi / 2, 'z': 0},
               '32':  {'size': 0.223, 'x': 2.4, 'y': 1.6, 'yaw': math.pi / 2, 'z': 0},
               '33':  {'size': 0.223, 'x': 2.4, 'y': 2.4, 'yaw': math.pi / 2, 'z': 0},
               '34':  {'size': 0.223, 'x': 2.4, 'y': 3.2, 'yaw': math.pi / 2, 'z': 0},
               '35':  {'size': 0.223, 'x': 2.4, 'y': 4.0, 'yaw': math.pi / 2, 'z': 0},
               '36':  {'size': 0.223, 'x': 2.4, 'y': 4.8, 'yaw': math.pi / 2, 'z': 0},
               '37':  {'size': 0.223, 'x': 2.4, 'y': 5.6, 'yaw': math.pi / 2, 'z': 0},
               '38':  {'size': 0.223, 'x': 2.4, 'y': 6.4, 'yaw': math.pi / 2, 'z': 0},
               '39':  {'size': 0.223, 'x': 2.4, 'y': 7.2, 'yaw': math.pi / 2, 'z': 0},
               '4':   {'size': 0.223, 'x': 0.0, 'y': 3.2, 'yaw': math.pi / 2, 'z': 0},
               '40':  {'size': 0.223, 'x': 3.2, 'y': 0.0, 'yaw': math.pi / 2, 'z': 0},
               '41':  {'size': 0.223, 'x': 3.2, 'y': 0.8, 'yaw': math.pi / 2, 'z': 0},
               '42':  {'size': 0.223, 'x': 3.2, 'y': 1.6, 'yaw': math.pi / 2, 'z': 0},
               '43':  {'size': 0.223, 'x': 3.2, 'y': 2.4, 'yaw': math.pi / 2, 'z': 0},
               '44':  {'size': 0.223, 'x': 3.2, 'y': 3.2, 'yaw': math.pi / 2, 'z': 0},
               '45':  {'size': 0.223, 'x': 3.2, 'y': 4.0, 'yaw': math.pi / 2, 'z': 0},
               '46':  {'size': 0.223, 'x': 3.2, 'y': 4.8, 'yaw': math.pi / 2, 'z': 0},
               '47':  {'size': 0.223, 'x': 3.2, 'y': 5.6, 'yaw': math.pi / 2, 'z': 0},
               '48':  {'size': 0.223, 'x': 3.2, 'y': 6.4, 'yaw': math.pi / 2, 'z': 0},
               '49':  {'size': 0.223, 'x': 3.2, 'y': 7.2, 'yaw': math.pi / 2, 'z': 0},
               '5':   {'size': 0.223, 'x': 0.0, 'y': 4.0, 'yaw': math.pi / 2, 'z': 0},
               '50':  {'size': 0.223, 'x': 4.0, 'y': 0.0, 'yaw': math.pi / 2, 'z': 0},
               '51':  {'size': 0.223, 'x': 4.0, 'y': 0.8, 'yaw': math.pi / 2, 'z': 0},
               '52':  {'size': 0.223, 'x': 4.0, 'y': 1.6, 'yaw': math.pi / 2, 'z': 0},
               '53':  {'size': 0.223, 'x': 4.0, 'y': 2.4, 'yaw': math.pi / 2, 'z': 0},
               '54':  {'size': 0.223, 'x': 4.0, 'y': 3.2, 'yaw': math.pi / 2, 'z': 0},
               '55':  {'size': 0.223, 'x': 4.0, 'y': 4.0, 'yaw': math.pi / 2, 'z': 0},
               '56':  {'size': 0.223, 'x': 4.0, 'y': 4.8, 'yaw': math.pi / 2, 'z': 0},
               '57':  {'size': 0.223, 'x': 4.0, 'y': 5.6, 'yaw': math.pi / 2, 'z': 0},
               '58':  {'size': 0.223, 'x': 4.0, 'y': 6.4, 'yaw': math.pi / 2, 'z': 0},
               '59':  {'size': 0.223, 'x': 4.0, 'y': 7.2, 'yaw': math.pi / 2, 'z': 0},
               '6':   {'size': 0.223, 'x': 0.0, 'y': 4.8, 'yaw': math.pi / 2, 'z': 0},
               '60':  {'size': 0.223, 'x': 4.8, 'y': 0.0, 'yaw': math.pi / 2, 'z': 0},
               '61':  {'size': 0.223, 'x': 4.8, 'y': 0.8, 'yaw': math.pi / 2, 'z': 0},
               '62':  {'size': 0.223, 'x': 4.8, 'y': 1.6, 'yaw': math.pi / 2, 'z': 0},
               '63':  {'size': 0.223, 'x': 4.8, 'y': 2.4, 'yaw': math.pi / 2, 'z': 0},
               '64':  {'size': 0.223, 'x': 4.8, 'y': 3.2, 'yaw': math.pi / 2, 'z': 0},
               '65':  {'size': 0.223, 'x': 4.8, 'y': 4.0, 'yaw': math.pi / 2, 'z': 0},
               '66':  {'size': 0.223, 'x': 4.8, 'y': 4.8, 'yaw': math.pi / 2, 'z': 0},
               '67':  {'size': 0.223, 'x': 4.8, 'y': 5.6, 'yaw': math.pi / 2, 'z': 0},
               '68':  {'size': 0.223, 'x': 4.8, 'y': 6.4, 'yaw': math.pi / 2, 'z': 0},
               '69':  {'size': 0.223, 'x': 4.8, 'y': 7.2, 'yaw': math.pi / 2, 'z': 0},
               '7':   {'size': 0.223, 'x': 0.0, 'y': 5.6, 'yaw': math.pi / 2, 'z': 0},
               '70':  {'size': 0.223, 'x': 5.6, 'y': 0.0, 'yaw': math.pi / 2, 'z': 0},
               '71':  {'size': 0.223, 'x': 5.6, 'y': 0.8, 'yaw': math.pi / 2, 'z': 0},
               '72':  {'size': 0.223, 'x': 5.6, 'y': 1.6, 'yaw': math.pi / 2, 'z': 0},
               '73':  {'size': 0.223, 'x': 5.6, 'y': 2.4, 'yaw': math.pi / 2, 'z': 0},
               '74':  {'size': 0.223, 'x': 5.6, 'y': 3.2, 'yaw': math.pi / 2, 'z': 0},
               '75':  {'size': 0.223, 'x': 5.6, 'y': 4.0, 'yaw': math.pi / 2, 'z': 0},
               '76':  {'size': 0.223, 'x': 5.6, 'y': 4.8, 'yaw': math.pi / 2, 'z': 0},
               '77':  {'size': 0.223, 'x': 5.6, 'y': 5.6, 'yaw': math.pi / 2, 'z': 0},
               '78':  {'size': 0.223, 'x': 5.6, 'y': 6.4, 'yaw': math.pi / 2, 'z': 0},
               '79':  {'size': 0.223, 'x': 5.6, 'y': 7.2, 'yaw': math.pi / 2, 'z': 0},
               '8':   {'size': 0.223, 'x': 0.0, 'y': 6.4, 'yaw': math.pi / 2, 'z': 0},
               '80':  {'size': 0.223, 'x': 6.4, 'y': 0.0, 'yaw': math.pi / 2, 'z': 0},
               '81':  {'size': 0.223, 'x': 6.4, 'y': 0.8, 'yaw': math.pi / 2, 'z': 0},
               '82':  {'size': 0.223, 'x': 6.4, 'y': 1.6, 'yaw': math.pi / 2, 'z': 0},
               '83':  {'size': 0.223, 'x': 6.4, 'y': 2.4, 'yaw': math.pi / 2, 'z': 0},
               '84':  {'size': 0.223, 'x': 6.4, 'y': 3.2, 'yaw': math.pi / 2, 'z': 0},
               '85':  {'size': 0.223, 'x': 6.4, 'y': 4.0, 'yaw': math.pi / 2, 'z': 0},
               '86':  {'size': 0.223, 'x': 6.4, 'y': 4.8, 'yaw': math.pi / 2, 'z': 0},
               '87':  {'size': 0.223, 'x': 6.4, 'y': 5.6, 'yaw': math.pi / 2, 'z': 0},
               '88':  {'size': 0.223, 'x': 6.4, 'y': 6.4, 'yaw': math.pi / 2, 'z': 0},
               '89':  {'size': 0.223, 'x': 6.4, 'y': 7.2, 'yaw': math.pi / 2, 'z': 0},
               '9':   {'size': 0.223, 'x': 0.0, 'y': 7.2, 'yaw': math.pi / 2, 'z': 0},
               '90':  {'size': 0.223, 'x': 7.2, 'y': 0.0, 'yaw': math.pi / 2, 'z': 0},
               '91':  {'size': 0.223, 'x': 7.2, 'y': 0.8, 'yaw': math.pi / 2, 'z': 0},
               '92':  {'size': 0.223, 'x': 7.2, 'y': 1.6, 'yaw': math.pi / 2, 'z': 0},
               '93':  {'size': 0.223, 'x': 7.2, 'y': 2.4, 'yaw': math.pi / 2, 'z': 0},
               '94':  {'size': 0.223, 'x': 7.2, 'y': 3.2, 'yaw': math.pi / 2, 'z': 0},
               '95':  {'size': 0.223, 'x': 7.2, 'y': 4.0, 'yaw': math.pi / 2, 'z': 0},
               '96':  {'size': 0.223, 'x': 7.2, 'y': 4.8, 'yaw': math.pi / 2, 'z': 0},
               '97':  {'size': 0.223, 'x': 7.2, 'y': 5.6, 'yaw': math.pi / 2, 'z': 0},
               '98':  {'size': 0.223, 'x': 7.2, 'y': 6.4, 'yaw': math.pi / 2, 'z': 0},
               '99':  {'size': 0.223, 'x': 7.2, 'y': 7.2, 'yaw': math.pi / 2, 'z': 0}}


transform_broadcaster = tf.TransformBroadcaster()
transform_listener = tf.TransformListener()


class MarkerPositionEstimator(object):
    pose = None
    last_published = None

    fcu_marker_translation = None
    fcu_marker_rotation = None

    recalc_time = 0

    def __init__(self):
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback, queue_size=1)
        rospy.Subscriber('/marker_data', MarkerArray, self.marker_callback, queue_size=1)

        self.vision_position_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
        self.offset_pub = rospy.Publisher('~offset', PointStamped, queue_size=1)
        self.position_fcu_pub = rospy.Publisher('~position/fcu', PoseStamped, queue_size=1)
        self.pose_pub = rospy.Publisher('~position/marker_map', PoseStamped, queue_size=1)

        self.vision_position_message = PoseStamped()

        self.position_fcu_message = PoseStamped()
        self.position_fcu_message.header.frame_id = 'vision_fcu'

        self.pose_message = PoseStamped()
        self.pose_message.header.frame_id = 'marker_map'

        self.marker_position_offset = PointStamped()

    def pose_callback(self, pose):
        self.pose = pose

    def calc_angle(self, a, frame_width):
        return math.atan(a * math.tan(FOV2 / 2) / (frame_width / 2))

    def marker_callback(self, data):
        if self.pose is None:
            return

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            [self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z,
             self.pose.pose.orientation.w])

        px2rad = FOV1 / data.frame_width

        x_fl = []
        y_fl = []
        z_fl = []
        yaw_fl = []

        biggest_marker_size = 0
        biggest_marker_yaw = None

        # print len(data.markers)

        # Process markers
        for marker in data.markers:
            marker_id = str(marker.id)
            if marker_id not in MARKERS_MAP.keys():
                continue

            marker_info = MARKERS_MAP[marker_id]

            marker_yaw = marker.yaw - marker_info['yaw'] - math.pi / 2  # ?
            marker_yaw = -marker_yaw

            corners = marker.points
            x = numpy.mean([corner.x for corner in corners]) - (data.frame_width / 2)
            y = numpy.mean([corner.y for corner in corners]) - (data.frame_height / 2)

            diag1 = math.hypot(corners[2].x - corners[0].x, corners[2].y - corners[0].y)
            diag2 = math.hypot(corners[3].x - corners[1].x, corners[3].y - corners[1].y)
            size = max(diag1, diag2)
            ang_size = size * px2rad

            # xa = -y * px2rad + pitch
            # ya = x * px2rad + roll

            xa = -self.calc_angle(y, data.frame_width) + pitch - math.radians(20)
            ya = self.calc_angle(x, data.frame_width) + roll

            # print math.degrees(roll), math.degrees(pitch), math.degrees(xa), math.degrees(ya)

            marker_size = marker_info['size'] * SQRT_2
            dist = ((marker_size / 2) / math.tan(ang_size / 2))  # * math.cos(xa) * math.cos(ya)  # comment cos!
            tx = -dist * math.tan(xa) + math.sin(pitch) * CAMERA_OFFSET['z'] + CAMERA_OFFSET['x'] + \
                 (marker_info['x'] + MAP_OFFSET['x']) * math.cos(marker_yaw) + \
                 (marker_info['y'] + MAP_OFFSET['y']) * math.sin(marker_yaw)
            ty = dist * math.tan(ya) - math.sin(roll) * CAMERA_OFFSET['z'] + CAMERA_OFFSET['y'] + \
                 (marker_info['x'] + MAP_OFFSET['x']) * -math.sin(marker_yaw) + \
                 (marker_info['y'] + MAP_OFFSET['y']) * math.cos(marker_yaw)

            # GEOMETRY BEGINS

            FOV2 = math.radians(130)
            FOV_X_2_TAN = math.tan(FOV2 / 2)
            FOV_Y_2_TAN = math.tan(FOV2 / 2) * data.frame_height / data.frame_width

            # PRINCIPAL_POINT_X = 831.2 / 1640 * data.frame_width
            # PRINCIPAL_POINT_Y = 563.8 / 1232 * data.frame_height
            PRINCIPAL_POINT_X = 330 * data.frame_width / 640
            PRINCIPAL_POINT_Y = 200 * data.frame_height / 480

            marker_yaw_ak = marker.yaw - marker_info['yaw'] + math.pi / 2

            height_pixels = (data.frame_width / 2) / FOV_X_2_TAN
            height_meters = height_pixels * marker_size / size

            marker_dist_to_center = math.sqrt((numpy.mean([corner.x for corner in corners]) - PRINCIPAL_POINT_X) ** 2 \
                                  + (numpy.mean([corner.y for corner in corners]) - PRINCIPAL_POINT_Y) ** 2)
            angle_to_marker = math.atan(marker_dist_to_center * FOV_X_2_TAN / data.frame_width)
            altitude = height_meters * math.cos(angle_to_marker + roll) / math.cos(angle_to_marker)
            altitude_comp = altitude / math.cos(roll)

            height_meters = altitude

            # print "\t".join([str(height_meters),
            #                  str(marker_dist_to_center),
            #                  str(angle_to_marker),
            #                  str(altitude),
            #                  str(altitude_comp)])

            # print "\t".join([str(numpy.mean([corner.x for corner in corners])),
            #                  str(numpy.mean([corner.y for corner in corners])),
            #                  str(pitch),
            #                  str(roll)])

            x_uncomp = (numpy.mean([corner.x for corner in corners]) - PRINCIPAL_POINT_X) / (data.frame_width / 2)
            y_uncomp = (numpy.mean([corner.y for corner in corners]) - PRINCIPAL_POINT_Y) / (data.frame_width / 2)

            x_comp = math.tan(math.atan(x_uncomp * FOV_X_2_TAN) + roll) / FOV_X_2_TAN
            y_comp = math.tan(math.atan(y_uncomp * FOV_X_2_TAN) - pitch) / FOV_X_2_TAN

            body_x_meters = x_comp * height_meters * FOV_X_2_TAN
            body_y_meters = y_comp * height_meters * FOV_X_2_TAN

            marker_field_x = body_y_meters \
                           - (marker_info['x'] + MAP_OFFSET['x']) * math.cos(marker_yaw_ak) \
                           + (marker_info['y'] + MAP_OFFSET['y']) * math.sin(marker_yaw_ak)

            marker_field_y = body_x_meters \
                           - (marker_info['x'] + MAP_OFFSET['x']) * math.sin(marker_yaw_ak) \
                           - (marker_info['y'] + MAP_OFFSET['y']) * math.cos(marker_yaw_ak)

            # print "\t".join(["height_m", "x_uncomp", "y_uncomp", "x_compensated", "y_compensated", "pitch"])
            # print "\t".join([str(height_meters), str(x_uncomp), str(y_uncomp), str(x_comp), str(y_comp), str(math.degrees(pitch))])

            # print "\t".join(["marker_id",
            #                  "height_m",
            #                  "body_x_yaw",
            #                  "body_y_yaw",
            #                  "marker_yaw"])

            # print "\t".join([marker_id,
            #                  str(height_meters),
            #                  str(marker_field_x),
            #                  str(marker_field_y),
            #                  str(marker_yaw_ak),
            #                  str(marker_dist_to_center)])

            # GEOMETRY ENDS

            # print tx, ty, marker_id

            dist += marker_info['z']

            if size > biggest_marker_size:
                biggest_marker_size = marker_size
                biggest_marker_yaw = marker_yaw_ak

            # x_fl.append(tx)
            # y_fl.append(ty)
            # z_fl.append(dist)

            x_fl.append(marker_field_x)
            y_fl.append(marker_field_y)
            z_fl.append(height_meters)
            yaw_fl.append(marker_yaw_ak % (2 * math.pi))

        global counter
        if not x_fl:
            counter = 0
            return

        tx = numpy.median(x_fl)
        ty = numpy.median(y_fl)
        dist = numpy.median(z_fl)
        biggest_marker_yaw = numpy.median(yaw_fl) #+ math.pi

        # print "\n---------------------------------------------------------------------------------------\n"

        # print tx, ty, dist, [marker.id for marker in data.markers]

        # quaternion = tf.transformations.quaternion_from_euler(0, 0, -biggest_marker_yaw + math.pi)
        quaternion = t.quaternion_from_euler(0, 0, biggest_marker_yaw)

        # print -biggest_marker_yaw + math.pi

        self.position_fcu_message.pose.position.x = tx
        self.position_fcu_message.pose.position.y = ty
        self.position_fcu_message.pose.position.z = -dist
        # self.position_fcu_message.pose.position.x = tx * math.cos(biggest_marker_yaw) + ty * math.sin(biggest_marker_yaw)
        # self.position_fcu_message.pose.position.y = tx * math.sin(biggest_marker_yaw) + ty * -math.cos(biggest_marker_yaw)

        self.position_fcu_message.pose.orientation.x = quaternion[0]
        self.position_fcu_message.pose.orientation.y = quaternion[1]
        self.position_fcu_message.pose.orientation.z = quaternion[2]
        self.position_fcu_message.pose.orientation.w = quaternion[3]
        self.position_fcu_message.header.stamp = data.header.stamp

        # Publish marker's map position in body frame
        self.position_fcu_pub.publish(self.position_fcu_message)

        # if dist < 4.0:
        #     filter_size = 3
        # else:
        #     filter_size = 6

        # ENU_x = tx * math.cos(yaw) + ty * math.sin(yaw)
        # ENU_y = tx * math.sin(yaw) + ty * -math.cos(yaw)
        #
        # quaternion = tf.transformations.quaternion_from_euler(0, 0, -biggest_marker_yaw)
        # self.pose_message.pose.position.x = ENU_x
        # self.pose_message.pose.position.y = ENU_y
        # self.pose_message.pose.position.z = dist
        # self.pose_message.pose.orientation.x = quaternion[0]
        # self.pose_message.pose.orientation.y = quaternion[1]
        # self.pose_message.pose.orientation.z = quaternion[2]
        # self.pose_message.pose.orientation.w = quaternion[3]
        # self.pose_message.header.stamp = data.header.stamp
        # self.pose_message.header.frame_id = 'marker_map'
        #
        # # Pose copter's position relative to the marker's map
        # self.pose_pub.publish(self.pose_message)

        if self.last_published is None or \
                                rospy.get_rostime() - self.last_published > rospy.Duration.from_sec(1.5) or \
                                time.time() - self.recalc_time > 5:

            counter += 1
            if counter > 3:
                rospy.loginfo('Recalculate marker_map frame')

                self.fcu_marker_translation = (tx, ty, -dist)
                self.fcu_marker_rotation = quaternion

                self.fcu_local_translation = self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z
                self.fcu_local_rotation = self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z, self.pose.pose.orientation.w

                self.recalc_time = time.time()
            else:
                return

        transform_broadcaster.sendTransform(
            (tx, ty, -dist),
            quaternion,
            data.header.stamp,
            'marker_map_dynamic',
            'tmp_fcu'
        )

        transform_broadcaster.sendTransform(
            self.fcu_marker_translation,
            self.fcu_marker_rotation,
            data.header.stamp,
            'marker_map',
            'tmp_fcu'
        )

        transform_broadcaster.sendTransform(
            self.fcu_local_translation,
            self.fcu_local_rotation,
            data.header.stamp,
            'tmp_fcu',
            'local_origin'
        )

        transform = t.concatenate_matrices(t.translation_matrix((self.position_fcu_message.pose.position.x,
                                                                self.position_fcu_message.pose.position.y,
                                                                self.position_fcu_message.pose.position.z)),
                                           t.quaternion_matrix((self.position_fcu_message.pose.orientation.x,
                                                                self.position_fcu_message.pose.orientation.y,
                                                                self.position_fcu_message.pose.orientation.z,
                                                                self.position_fcu_message.pose.orientation.w
                                                               )))
        inversed_transform = t.inverse_matrix(transform)

        transform_broadcaster.sendTransform(
            t.translation_from_matrix(inversed_transform),
            t.quaternion_from_matrix(inversed_transform),
            data.header.stamp,
            'vision_fcu',
            'marker_map'
        )

        try:
            # self.position_fcu_message.header.stamp = rospy.Time(0)
            # self.pose_message = transform_listener.transformPose('marker_map', self.position_fcu_message)
            self.pose_message.header.stamp = data.header.stamp
            self.pose_message.pose.position.x = t.translation_from_matrix(inversed_transform)[0]
            self.pose_message.pose.position.y = t.translation_from_matrix(inversed_transform)[1]
            self.pose_message.pose.position.z = t.translation_from_matrix(inversed_transform)[2]
            self.pose_message.pose.orientation.x = t.quaternion_from_matrix(inversed_transform)[0]
            self.pose_message.pose.orientation.y = t.quaternion_from_matrix(inversed_transform)[1]
            self.pose_message.pose.orientation.z = t.quaternion_from_matrix(inversed_transform)[2]
            self.pose_message.pose.orientation.w = t.quaternion_from_matrix(inversed_transform)[3]

            self.pose_pub.publish(self.pose_message)
        except:
            pass

        if PUBLISH_MAVLINK:
            # p = PoseStamped()
            # p.pose.position = Point(
            #     self.pose_message.pose.position.x + self.fcu_marker_translation[0] + self.fcu_local_translation[0],
            #     self.pose_message.pose.position.y + self.fcu_marker_translation[1] + self.fcu_local_translation[1],
            #     self.pose_message.pose.position.z + self.fcu_marker_translation[2] + self.fcu_local_translation[2]
            # )
            # p.pose.orientation = orientation_from_euler(0, 0, -biggest_marker_yaw)
            # p.header.stamp = data.header.stamp
            # self.vision_position_pub.publish(p)


            try:
                p = PoseStamped()
                p.pose.position.x = 0
                p.pose.position.y = 0
                p.pose.position.z = 0
                q = t.quaternion_from_euler(0, 0, 0)
                p.pose.orientation.x = q[0]
                p.pose.orientation.y = q[1]
                p.pose.orientation.z = q[2]
                p.pose.orientation.w = q[3]
                p.header.frame_id = 'vision_fcu'
                p.header.stamp = rospy.Time(0)

                vision_position = transform_listener.transformPose('local_origin', p)
                vision_position.header.stamp = data.header.stamp
                self.vision_position_pub.publish(vision_position)
                # print vision_position
                # print 'vision yaw', t.euler_from_quaternion((vision_position.pose.orientation.x,vision_position.pose.orientation.y,vision_position.pose.orientation.z, vision_position.pose.orientation.w))[2]
            except Exception as e:
                print e

        # try:
            # print 'wait transform'
            # transform_listener.waitForTransform('local_origin', 'marker_map', rospy.get_rostime(), rospy.Duration(1))
            # print transform_listener.transformPose('local_origin', self.pose_message)
            # print 'published'
        # except:
            # print 'not published'

        # if PUBLISH_MAVLINK:
        #
        #     self.vision_position_message.pose.position.x = self.pose_message.pose.position.x
        #     self.vision_position_message.pose.position.y = self.pose_message.pose.position.y
        #     self.vision_position_message.pose.position.z = self.pose_message.pose.position.z
        #     self.vision_position_message.pose.orientation.x = quaternion[0]
        #     self.vision_position_message.pose.orientation.y = quaternion[1]
        #     self.vision_position_message.pose.orientation.z = quaternion[2]
        #     self.vision_position_message.pose.orientation.w = quaternion[3]
        #     self.vision_position_message.header.stamp = data.header.stamp
        #
        #     if POSITION_OFFSET:
        #         if self.last_published is None or \
        #                                 rospy.get_rostime() - self.last_published > rospy.Duration.from_sec(1.5):
        #             # Calculate offset between local position and marker position
        #             self.marker_position_offset.point.x = self.pose.pose.position.x - ENU_x
        #             self.marker_position_offset.point.y = self.pose.pose.position.y - ENU_y
        #             self.marker_position_offset.point.z = self.pose.pose.position.z - dist
        #             self.marker_position_offset.header.stamp = data.header.stamp
        #
        #             # Publish the offset
        #             self.offset_pub.publish(self.marker_position_offset)
        #
        #         # Apply the offset
        #         self.vision_position_message.pose.position.x += self.marker_position_offset.point.x
        #         self.vision_position_message.pose.position.y += self.marker_position_offset.point.y
        #         self.vision_position_message.pose.position.z += self.marker_position_offset.point.z
        #
        #     self.vision_position_pub.publish(self.vision_position_message)

        self.last_published = rospy.get_rostime()


MarkerPositionEstimator()

rospy.spin()
