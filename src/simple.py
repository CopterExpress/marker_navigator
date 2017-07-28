from __future__ import division

import rospy
import time
import math
from threading import Lock
from geometry_msgs.msg import PoseStamped, PointStamped, Point, Vector3, Vector3Stamped
from mavros_msgs.msg import PositionTarget, AttitudeTarget
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from marker_navigator.srv import SetPosition, SetPositionRequest, SetPositionResponse, \
    SetPositionYawRate, SetPositionYawRateRequest, SetPositionYawRateResponse, \
    SetVelocity, SetVelocityRequest, SetVelocityResponse, \
    SetVelocityYawRate, SetVelocityYawRateRequest, SetVelocityYawRateResponse, \
    SetAttitude, SetAttitudeRequest, SetAttitudeResponse, \
    SetAttitudeYawRate, SetAttitudeYawRateRequest, SetAttitudeYawRateResponse, \
    SetRatesYaw, SetRatesYawRequest, SetRatesYawResponse, \
    SetRates, SetRatesRequest, SetRatesResponse
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import tf
import tf.transformations as t

from util import orientation_from_quaternion, orientation_from_euler, eurler_from_orientation, \
    vector3_from_point


rospy.init_node('simple_control', disable_signals=True)


task = None
msg = None
pub = None
response_class = None

task_change_lock = Lock()

mode = None
armed = None


transform_listener = tf.TransformListener()
transform_broadcaster = tf.TransformBroadcaster()


def update_pose(data):
    pose = data.pose
    translation = pose.position.x, pose.position.y, pose.position.z
    yaw = eurler_from_orientation(pose.orientation)[2]
    rotation = t.quaternion_from_euler(0, 0, yaw)

    transform_broadcaster.sendTransform(
        translation,
        rotation,
        data.header.stamp,
        'fcu_horiz',
        'local_origin'
    )


rospy.Subscriber('/mavros/local_position/pose', PoseStamped, update_pose)

position_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
attitude_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)


def state_update(data):
    global mode, armed
    mode = data.mode
    armed = data.armed


rospy.Subscriber('/mavros/state', State, state_update)


def offboard_and_arm():
    if mode != 'OFFBOARD':
        time.sleep(.3)
        print 'Offboarding'
        ret = set_mode(base_mode=0, custom_mode='OFFBOARD')
        if not ret:
            return False
        start = time.time()
        while True:
            if mode == 'OFFBOARD':
                break
            if time.time() - start > 3:
                return False
    if not armed:
        print 'Arming'
        ret = arming(True)
        if not ret:
            return False
        start = time.time()
        while True:
            if armed:
                break
            if time.time() - start > 5:
                return False
    return True


def get_response_class(req):
    return {
        SetPositionRequest: SetPositionResponse,
        SetPositionYawRateRequest: SetPositionYawRateResponse,
        SetVelocityRequest: SetVelocityResponse,
        SetVelocityYawRateRequest: SetVelocityYawRateResponse,
        SetAttitudeRequest: SetAttitudeResponse,
        SetAttitudeYawRateRequest: SetAttitudeYawRateResponse,
        SetRatesYawRequest: SetRatesYawResponse,
        SetRatesRequest: SetRatesResponse,
    }[type(req)]


def get_message_publisher(task):
    if isinstance(task, SetPositionRequest):
        pose = PoseStamped()
        pose.header.frame_id = task.frame_id or 'local_origin'
        pose.header.stamp = rospy.Time(0)
        pose.pose.position = Point(task.x, task.y, task.z)
        pose.pose.orientation = orientation_from_euler(0, 0, task.yaw)
        pose_local = transform_listener.transformPose('local_origin', pose)
        msg = PositionTarget(
            coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
            type_mask=PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +
                      PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                      PositionTarget.IGNORE_YAW_RATE,
            position=pose_local.pose.position,
            yaw=eurler_from_orientation(pose_local.pose.orientation)[2] - math.pi / 2
        )
        return position_pub, msg

    elif isinstance(task, SetPositionYawRateRequest):
        pose = PoseStamped()
        pose.header.frame_id = task.frame_id or 'local_origin'
        pose.header.stamp = rospy.Time(0)
        pose.pose.position = Point(task.x, task.y, task.z)
        pose_local = transform_listener.transformPose('local_origin', pose)
        msg = PositionTarget(
            coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
            type_mask=PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +
                      PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                      PositionTarget.IGNORE_YAW,
            position=pose_local.pose.position,
            yaw_rate=task.yaw_rate
        )
        return position_pub, msg

    elif isinstance(task, SetVelocityRequest):
        vector = Vector3Stamped()
        vector.vector = Vector3(task.vx, task.vy, task.vz)
        vector.header.frame_id = task.frame_id or 'local_origin'
        pose = PoseStamped()
        pose.header.frame_id = task.frame_id or 'local_origin'
        pose.header.stamp = rospy.Time(0)
        pose.pose.orientation = orientation_from_euler(0, 0, task.yaw)
        pose_local = transform_listener.transformPose('local_origin', pose)
        vector_local = transform_listener.transformVector3('local_origin', vector)
        msg = PositionTarget(
            coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
            type_mask=PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ +
                      PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                      PositionTarget.IGNORE_YAW_RATE,
            yaw=eurler_from_orientation(pose_local.pose.orientation)[2] - math.pi / 2
        )
        msg.velocity = vector_local.vector
        return position_pub, msg

    elif isinstance(task, SetVelocityYawRateRequest):
        vector = Vector3Stamped()
        vector.vector = Vector3(task.vx, task.vy, task.vz)
        vector.header.frame_id = task.frame_id or 'local_origin'
        vector_local = transform_listener.transformVector3('local_origin', vector)
        msg = PositionTarget(
            coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
            type_mask=PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ +
                      PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                      PositionTarget.IGNORE_YAW,
            yaw_rate=task.yaw_rate
        )
        msg.velocity = vector_local.vector
        return position_pub, msg

    elif isinstance(task, SetAttitudeRequest):
        pose = PoseStamped()
        pose.header.frame_id = task.frame_id or 'local_origin'
        pose.header.stamp = rospy.Time(0)
        pose.pose.orientation = orientation_from_euler(task.roll, task.pitch, task.yaw)
        pose_local = transform_listener.transformPose('local_origin', pose)
        msg = AttitudeTarget(
            orientation=pose_local.pose.orientation,
            thrust=task.thrust,
            type_mask=AttitudeTarget.IGNORE_YAW_RATE + AttitudeTarget.IGNORE_PITCH_RATE + AttitudeTarget.IGNORE_ROLL_RATE
        )
        return attitude_pub, msg

    elif isinstance(task, SetAttitudeYawRateRequest):
        msg = AttitudeTarget(
            orientation=orientation_from_euler(task.roll, task.pitch, 0),
            thrust=task.thrust,
            type_mask=AttitudeTarget.IGNORE_PITCH_RATE + AttitudeTarget.IGNORE_ROLL_RATE
        )
        msg.body_rate.z = task.yaw_rate
        return attitude_pub, msg

    elif isinstance(task, SetRatesYawRequest):
        pose = PoseStamped()
        pose.header.frame_id = task.frame_id or 'local_origin'
        pose.header.stamp = rospy.Time(0)
        pose.pose.orientation = orientation_from_euler(0, 0, task.yaw)
        pose_local = transform_listener.transformPose('local_origin', pose)
        msg = AttitudeTarget(
            orientation=pose_local.pose.orientation,
            thrust=task.thrust,
            type_mask=AttitudeTarget.IGNORE_YAW_RATE
        )
        msg.body_rate = Vector3(task.roll_rate, task.pitch_rate, 0)
        return attitude_pub, msg

    elif isinstance(task, SetRatesRequest):
        msg = AttitudeTarget(
            thrust=task.thrust,
            type_mask=AttitudeTarget.IGNORE_ATTITUDE
        )
        msg.body_rate = Vector3(task.roll_rate, task.pitch_rate, task.yaw_rate)
        return attitude_pub, msg


def handle(req):
    global msg, pub, task
    print 'get resp class'
    response_class = get_response_class(req)
    print 'resp', response_class
    try:
        task = req
        with task_change_lock:
            pub, msg = get_message_publisher(task)
        print pub, msg
        msg.header.stamp = rospy.get_rostime()
        pub.publish(msg)
        if not offboard_and_arm():
            return response_class(success=False, message='Cannot arm or offboard the vehicle')
        return response_class(success=True)
    except Exception as e:
        return response_class(success=False, message=e.message)


def release(req):
    global msg
    with task_change_lock:
        msg = None
    return TriggerResponse(success=True)


rospy.Service('/set_position', SetPosition, handle)
rospy.Service('/set_position/yaw_rate', SetPositionYawRate, handle)
rospy.Service('/set_velocity', SetVelocity, handle)
rospy.Service('/set_velocity/yaw_rate', SetVelocityYawRate, handle)
rospy.Service('/set_attitude', SetAttitude, handle)
rospy.Service('/set_attitude/yaw_rate', SetAttitudeYawRate, handle)
rospy.Service('/set_rates', SetRates, handle)
rospy.Service('/set_rates/yaw', SetRatesYaw, handle)
rospy.Service('/release', Trigger, release)


r = rospy.Rate(10)

while True:
    if msg is not None:
        try:
            with task_change_lock:
                if hasattr(task, 'update_frame') and task.update_frame:
                    pub, msg = get_message_publisher(task)
                    print msg
                msg.header.stamp = rospy.get_rostime()
                pub.publish(msg)

        except Exception as e:
            print e

    r.sleep()
