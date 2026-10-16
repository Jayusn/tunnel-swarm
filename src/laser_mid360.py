import rospy
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
import sys
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from tf.transformations import quaternion_matrix, quaternion_from_matrix, quaternion_multiply
import numpy as np
import fcntl
import os
import struct
from mavros_msgs.msg import Mavlink
from pymavlink.dialects.v20 import common as mavlink2


class FastLioOdometrySender:
    """通过 MAVROS 原始通道发送带局部坐标系和重置计数的外部位姿。"""

    def __init__(self, publisher, reset_file, system_id=1, component_id=191):
        self.publisher = publisher
        self.reset_file = reset_file
        self.encoder = mavlink2.MAVLink(None, srcSystem=system_id, srcComponent=component_id)
        self.reset_counter = None
        self.last_sample = None

    def next_reset_counter(self):
        # ROS 参数随 roscore 退出会丢失，计数写在 ROS_HOME，跨脚本/ROS 重启保持。
        os.makedirs(os.path.dirname(self.reset_file), exist_ok=True)
        with open(self.reset_file, 'a+', encoding='ascii') as state:
            fcntl.flock(state, fcntl.LOCK_EX)
            state.seek(0)
            previous = state.read().strip()
            counter = (int(previous or '0') + 1) % 256
            state.seek(0)
            state.truncate()
            state.write(str(counter))
            state.flush()
            os.fsync(state.fileno())
        return counter

    def publish(self, odom):
        stamp = odom.header.stamp.to_nsec()
        sample = (stamp, odom.header.seq, odom.header.frame_id)
        if self.last_sample is not None:
            if sample == self.last_sample:
                return False
            # 只认源序号/时间回退或坐标系变更，不用位置、角度突变猜测重置。
            restarted = (stamp < self.last_sample[0] or
                         (odom.header.seq < self.last_sample[1] and
                          self.last_sample[1] != 0xffffffff) or
                         odom.header.frame_id != self.last_sample[2])
        else:
            restarted = True
        if restarted:
            self.reset_counter = self.next_reset_counter()

        pose = odom.pose.pose
        q = pose.orientation
        # 保持原 MAVROS vision_pose 的 ENU->NED、FLU->FRD 数值转换。
        # LOCAL_FRD 仅声明该局部世界轴不以磁北为基准，不额外旋转规划坐标系。
        world_rotation = [np.sqrt(0.5), np.sqrt(0.5), 0.0, 0.0]
        body_rotation = [1.0, 0.0, 0.0, 0.0]
        rotation = quaternion_multiply(
            quaternion_multiply(world_rotation, [q.x, q.y, q.z, q.w]), body_rotation)
        position = pose.position
        unknown = float('nan')
        msg = self.encoder.odometry_encode(
            stamp // 1000, mavlink2.MAV_FRAME_LOCAL_FRD, mavlink2.MAV_FRAME_BODY_FRD,
            position.y, position.x, -position.z,
            [rotation[3], rotation[0], rotation[1], rotation[2]],
            unknown, unknown, unknown, unknown, unknown, unknown,
            # 原 PoseStamped 接口给 PX4 的位姿协方差为零；速度仍不提供。
            [0.0] * 21, [unknown] * 21, self.reset_counter,
            # PX4 1.16 的外部视觉入口接受 VISION，不接受 LIDAR 枚举。
            mavlink2.MAV_ESTIMATOR_TYPE_VISION, 0)
        msg.pack(self.encoder)
        self.encoder.seq = (self.encoder.seq + 1) % 256
        payload = msg.get_payload()
        padded = payload + b'\0' * (-len(payload) % 8)
        header = msg.get_header()
        wire = Mavlink()
        wire.header.stamp = odom.header.stamp
        wire.framing_status = Mavlink.FRAMING_OK
        wire.magic = Mavlink.MAVLINK_V20
        wire.len = len(payload)
        wire.seq = msg.get_seq()
        wire.sysid = msg.get_srcSystem()
        wire.compid = msg.get_srcComponent()
        wire.msgid = msg.get_msgId()
        wire.incompat_flags = header.incompat_flags
        wire.compat_flags = header.compat_flags
        wire.checksum = msg.get_crc()
        wire.payload64 = struct.unpack('<%dQ' % (len(padded) // 8), padded)
        self.publisher.publish(wire)
        self.last_sample = sample
        return True

print("sys.argv 内容：", sys.argv)


# 从命令行参数获取车辆类型、ID和激光SLAM类型
vehicle_type = sys.argv[1]  # 第一个参数：车辆类型
vehicle_id = sys.argv[2]  # 第二个参数：车辆ID
laser_slam_type = sys.argv[3]  # 第三个参数：激光SLAM类型
truth_odom_switch = sys.argv[4] if len(sys.argv) > 4 else 'off'  # 第四个参数：是否输出真值里程计，默认关闭
truth_odom_mode = truth_odom_switch.lower()
enable_truth_odom = truth_odom_mode in ('on', 'true', '1', 'gt', 'truth', 'aligned', 'raw')
model_name = vehicle_type + '_' + vehicle_id
# 2026-07-27: Gazebo/PX4 model 仍按 iris_0/iris_1 查找，仅将 ROS 感知与 MAVROS 前缀统一为 UAV0/UAV1。
ros_vehicle_name = 'UAV' + vehicle_id


# 2026-07-27: 节点名跟随新的 UAV 前缀，命令行第一个 iris 参数仍只表示 Gazebo 机型。
rospy.init_node(ros_vehicle_name+'_'+laser_slam_type+'_laser_transfer')

# 2026-07-27: 感知桥接的里程计、真值、MAVROS 和 TF 全部切换到 UAV0/UAV1。
vehicle_ns = '/' + ros_vehicle_name
odom_topic = rospy.get_param('~odom_topic', vehicle_ns + '/fast_lio/Odometry')
truth_odom_topic = rospy.get_param('~truth_odom_topic', vehicle_ns + '/ground_truth_odom')
vision_pose_topic = rospy.get_param('~vision_pose_topic', vehicle_ns + '/mavros/vision_pose/pose')
fastlio_world_frame = rospy.get_param('~fastlio_world_frame', ros_vehicle_name + '/camera_init')
fastlio_body_frame = rospy.get_param('~fastlio_body_frame', ros_vehicle_name + '/body')

# 实机 FAST-LIO 只发送 ODOMETRY，避免旧 vision_pose 同时参与融合。
use_fastlio_odometry = laser_slam_type == 'fastlio' and not enable_truth_odom
pose_pub = None
fastlio_sender = None
if use_fastlio_odometry:
    mavlink_topic = vehicle_ns + '/mavlink/to'
    reset_file = os.path.join(os.environ.get('ROS_HOME', os.path.expanduser('~/.ros')),
                              ros_vehicle_name + '_fastlio_odom_reset_counter')
    fastlio_sender = FastLioOdometrySender(
        rospy.Publisher(mavlink_topic, Mavlink, queue_size=1), reset_file,
        int(rospy.get_param(vehicle_ns + '/mavros/system_id', 1)),
        int(rospy.get_param(vehicle_ns + '/mavros/component_id', 191)))
    rospy.loginfo('[laser_mid360] FAST-LIO -> %s: ODOMETRY LOCAL_FRD, pose only; '
                  'reset counter persists in %s.', mavlink_topic, reset_file)
else:
    pose_pub = rospy.Publisher(vision_pose_topic, PoseStamped, queue_size=1)
truth_odom_pub = None
truth_model_index = None
latest_fastlio_odom = None
truth_alignment = None
truth_output_frame = 'map'
truth_child_frame = 'base_link'

if enable_truth_odom:
    truth_odom_pub = rospy.Publisher(truth_odom_topic, Odometry, queue_size=10)

# 设置TF监听器，用于获取坐标变换信息
tfBuffer = Buffer()
tflistener = TransformListener(tfBuffer)

# 初始化局部姿态
local_pose = PoseStamped()
local_pose.header.frame_id = 'map'  # 设置坐标系为'map'

# 初始化Hector SLAM姿态和高度
hector = PoseStamped()
height = 0



def hector_callback(data):
    """
    Hector SLAM回调函数，更新全局hector姿态
    :param data: 接收到的PoseStamped消息
    """
    global hector
    hector = data

def height_distance_callback(msg):
    """
    高度距离回调函数，更新全局高度值
    :param msg: 接收到的LaserScan消息
    """
    global height
    height = msg.ranges[0]  # 获取第一个激光测距值作为高度
    if(height==float("inf")):  # 如果高度值为无穷大，设为0
        height = 0


def gazebo_truth_callback(msg):
    """
    Gazebo真值回调函数
    从 /gazebo/model_states 中提取当前无人机的真值，并直接发布到独立真值话题。
    """
    global truth_model_index

    if truth_odom_pub is None:
        return

    if truth_model_index is None:
        try:
            truth_model_index = msg.name.index(model_name)
            rospy.loginfo('[laser_mid360] Truth odom enabled. Model %s found in /gazebo/model_states.', model_name)
        except ValueError:
            rospy.logwarn_throttle(2.0, '[laser_mid360] Model %s not found in /gazebo/model_states.', model_name)
            return

    if truth_model_index >= len(msg.pose) or truth_model_index >= len(msg.twist):
        rospy.logwarn_throttle(2.0, '[laser_mid360] Truth odom index out of range for %s.', model_name)
        return

    truth_odom = build_truth_odom(msg.pose[truth_model_index], msg.twist[truth_model_index])
    if truth_odom is None:
        return

    truth_odom.pose.covariance = [0.0] * 36
    truth_odom.twist.covariance = [0.0] * 36
    truth_odom_pub.publish(truth_odom)
    # 2026-07-28: 仿真参数 fastlio on 不再只旁路记录真值；同步把对齐真值送入 PX4 外部视觉，
    # 避免 FAST-LIO 在窄通道退化时把 MAVROS local 估计拉到数十米外。实机/off 模式仍使用 FAST-LIO TF。
    truth_pose = PoseStamped()
    truth_pose.header = truth_odom.header
    truth_pose.pose = truth_odom.pose.pose
    pose_pub.publish(truth_pose)


def odom_callback(msg):
    """
    保存当前 FastLIO /Odometry，用于把 Gazebo 真值对齐到当前 odom 坐标系。
    """
    global latest_fastlio_odom
    latest_fastlio_odom = msg


def pose_to_matrix(pose):
    transform = quaternion_matrix([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ])
    transform[0, 3] = pose.position.x
    transform[1, 3] = pose.position.y
    transform[2, 3] = pose.position.z
    return transform


def matrix_to_pose(transform, pose_msg):
    quat = quaternion_from_matrix(transform)
    pose_msg.position.x = float(transform[0, 3])
    pose_msg.position.y = float(transform[1, 3])
    pose_msg.position.z = float(transform[2, 3])
    pose_msg.orientation.x = float(quat[0])
    pose_msg.orientation.y = float(quat[1])
    pose_msg.orientation.z = float(quat[2])
    pose_msg.orientation.w = float(quat[3])


def vector_to_numpy(vec):
    return np.array([vec.x, vec.y, vec.z], dtype=float)


def ensure_truth_alignment(truth_pose):
    """
    在 aligned/on 模式下，用首次同步到的 /Odometry 和 Gazebo 真值建立固定坐标替换。
    """
    global truth_alignment, truth_output_frame, truth_child_frame

    if truth_odom_mode == 'raw':
        truth_output_frame = 'map'
        truth_child_frame = 'base_link'
        return True

    if truth_alignment is not None:
        return True

    if latest_fastlio_odom is None:
        rospy.logwarn_throttle(2.0, '[laser_mid360] Waiting for /Odometry before aligning truth odom.')
        return False

    odom_transform = pose_to_matrix(latest_fastlio_odom.pose.pose)
    truth_transform = pose_to_matrix(truth_pose)
    truth_alignment = np.dot(odom_transform, np.linalg.inv(truth_transform))
    truth_output_frame = latest_fastlio_odom.header.frame_id or 'camera_init'
    truth_child_frame = latest_fastlio_odom.child_frame_id or 'body'
    rospy.loginfo(
        '[laser_mid360] Truth odom aligned to /Odometry. frame=%s child=%s mode=%s',
        truth_output_frame, truth_child_frame, truth_odom_mode)
    return True


def build_truth_odom(truth_pose, truth_twist):
    truth_odom = Odometry()
    truth_odom.header.stamp = rospy.Time.now()

    if not ensure_truth_alignment(truth_pose):
        return None

    if truth_odom_mode == 'raw':
        truth_odom.header.frame_id = 'map'
        truth_odom.child_frame_id = 'base_link'
        truth_odom.pose.pose = truth_pose
        truth_odom.twist.twist = truth_twist
        return truth_odom

    aligned_transform = np.dot(truth_alignment, pose_to_matrix(truth_pose))
    truth_odom.header.frame_id = truth_output_frame
    truth_odom.child_frame_id = truth_child_frame
    matrix_to_pose(aligned_transform, truth_odom.pose.pose)

    rotation = truth_alignment[:3, :3]
    linear = rotation.dot(vector_to_numpy(truth_twist.linear))
    angular = rotation.dot(vector_to_numpy(truth_twist.angular))
    truth_odom.twist.twist.linear.x = float(linear[0])
    truth_odom.twist.twist.linear.y = float(linear[1])
    truth_odom.twist.twist.linear.z = float(linear[2])
    truth_odom.twist.twist.angular.x = float(angular[0])
    truth_odom.twist.twist.angular.y = float(angular[1])
    truth_odom.twist.twist.angular.z = float(angular[2])
    return truth_odom
    
def hector_slam():
    """
    Hector SLAM主函数
    订阅姿态信息，并以100Hz的频率发布更新后的姿态
    """
    global local_pose, height
    pose2d_sub = rospy.Subscriber(vehicle_type+'_'+ vehicle_id+"/pose", PoseStamped, hector_callback,queue_size=1)
    rate = rospy.Rate(100)  # 设置循环频率为100Hz
    while True:
        local_pose = hector  # 更新局部姿态
        local_pose.pose.position.z = height  # 更新高度信息
        pose_pub.publish(local_pose)  # 发布更新后的姿态
        rate.sleep()  # 等待下一个循环
        
def cartographer():
    """
    Cartographer SLAM主函数
    以30Hz的频率获取并发布更新后的姿态
    """
    global local_pose, height
    rate = rospy.Rate(30)  # 设置循环频率为30Hz
    while not rospy.is_shutdown():
        try:
            # 尝试获取从'map'到'base_link'的坐标变换
            tfstamped = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0))
        except:
            continue  # 如果获取失败，继续下一次循环
        local_pose.header.stamp = rospy.Time().now()  # 更新时间戳
        local_pose.pose.position = tfstamped.transform.translation  # 更新位置
        local_pose.pose.position.z = height  # 更新高度
        local_pose.pose.orientation = tfstamped.transform.rotation  # 更新朝向
        pose_pub.publish(local_pose)  # 发布更新后的姿态
        rate.sleep()  # 等待下一个循环
        
def aloam():
    """
    A-LOAM SLAM主函数
    以30Hz的频率获取并发布更新后的姿态
    """
    global local_pose
    rate = rospy.Rate(30)  # 设置循环频率为30Hz
    while not rospy.is_shutdown():
        try:
            # 尝试获取从'camera_init'到'aft_mapped'的坐标变换
            tfstamped = tfBuffer.lookup_transform('camera_init', 'aft_mapped', rospy.Time(0))
        except:
            continue  # 如果获取失败，继续下一次循环
        local_pose.header.stamp = rospy.Time().now()  # 更新时间戳
        local_pose.pose.position = tfstamped.transform.translation  # 更新位置
        local_pose.pose.orientation = tfstamped.transform.rotation  # 更新朝向
        pose_pub.publish(local_pose)  # 发布更新后的姿态
        rate.sleep()  # 等待下一个循环

def fastlio():
    """实机按最多50Hz转发高频里程计，保留测量时刻；仿真由真值回调发布。"""
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        odom = latest_fastlio_odom
        if not enable_truth_odom and odom is not None:
            fastlio_sender.publish(odom)
        rate.sleep()

if __name__ == '__main__':
    if enable_truth_odom or laser_slam_type == 'fastlio':
        rospy.Subscriber(odom_topic, Odometry, odom_callback, queue_size=1)
    if enable_truth_odom:
        rospy.Subscriber('/gazebo/model_states', ModelStates, gazebo_truth_callback, queue_size=1)
        rospy.loginfo(
            '[laser_mid360] Truth odom output is ON. mode=%s, odom=%s truth=%s frames=%s->%s.',
            truth_odom_mode, odom_topic, truth_odom_topic, fastlio_world_frame, fastlio_body_frame)
    else:
        rospy.loginfo('[laser_mid360] Truth odom output is OFF.')

    # 根据指定的SLAM类型执行相应的函数
    if laser_slam_type == 'hector':
        # 订阅距离信息，用于更新高度
        height_distance_sub = rospy.Subscriber(vehicle_type+'_'+ vehicle_id+"/distance", LaserScan, height_distance_callback,queue_size=1)
        rospy.loginfo(f"{laser_slam_type} :: start translating ! ! ! ")
        hector_slam()  # 执行Hector SLAM
        
    elif laser_slam_type == 'cartographer':
        # 订阅距离信息，用于更新高度
        height_distance_sub = rospy.Subscriber(vehicle_type+'_'+ vehicle_id+"/distance", LaserScan, height_distance_callback,queue_size=1)
        rospy.loginfo(f"{laser_slam_type} :: start translating ! ! ! ")
        cartographer()  # 执行Cartographer SLAM

    elif laser_slam_type == 'aloam':
        rospy.loginfo(f"{laser_slam_type} :: start translating ! ! ! ")
        aloam()  # 执行A-LOAM SLAM

    elif laser_slam_type == 'fastlio':
        rospy.loginfo(f"{laser_slam_type} :: start translating ! ! ! ")
        fastlio()  # 执行fastlio SLAM

    else:
        print('input error')  # 如果输入的SLAM类型不正确，打印错误信息
        
