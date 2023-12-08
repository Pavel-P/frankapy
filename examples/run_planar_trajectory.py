import pickle as pkl
import numpy as np
import argparse

from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import ForcePositionSensorMessage, ForcePositionControllerSensorMessage
from franka_interface_msgs.msg import SensorDataGroup
from frankapy.utils import transform_to_list, convert_array_to_rigid_transform


from tqdm import trange

import rospy

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--trajectory_pickle', '-t', type=str, required=True,
                        help='Path to trajectory (in pickle format) to replay.')
    args = parser.parse_args()

    fa = FrankaArm()
    fa.reset_joints()

    rospy.loginfo('Loading Trajectory')

    with open(args.trajectory_pickle, 'rb') as pkl_f:
        skill_data = pickle.load(pkl_f)
    
    assert skill_data[0]['skill_description'] == 'PlanarGuideMode', \
        "Trajectory not collected in planar guide mode"
    skill_state_dict = skill_data[0]['skill_state_dict']

    pose_traj = skill_state_dict['O_T_EE']
    pose_traj = [convert_array_to_rigid_transform(p) for p in pose_traj]
    pose_traj = [transform_to_list(p) for p in pose_traj]

    T = skill_data[0]['time']
    force = skill_data[0]['force']
    target_force = [0, 0, -force, 0, 0, 0]
    hz = skill_data[0]['rate']

    S = [1, 1, 0, 1, 1, 1]
    tz = FC.DEFAULT_TRANSLATIONAL_STIFFNESSES
    rz = FC.DEFAULT_ROTATIONAL_STIFFNESSES
    position_kps_cart = tz + rz
    force_kps_cart = [0.1] * 6

    rospy.loginfo('Initializing Sensor Publisher')
    pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=10)
    rate = rospy.Rate(hz)

    rospy.loginfo('Publishing HFPC trajectory w/ cartesian gains...')
    fa.run_dynamic_force_position(duration=T, buffer_time=3, S=S,
                                use_cartesian_gains=True,
                                position_kps_cart=position_kps_cart,
                                force_kps_cart=force_kps_cart)
    init_time = rospy.Time.now().to_time()
    N = T * hz
    for i in trange(N):
        t = i % N
        timestamp = rospy.Time.now().to_time() - init_time
        traj_gen_proto_msg = ForcePositionSensorMessage(
            id=i, timestamp=timestamp, seg_run_time=1./hz,
            pose=transform_to_list(pose_traj[i]),
            force=target_force
        )
        fb_ctrlr_proto = ForcePositionControllerSensorMessage(
            id=i, timestamp=timestamp,
            position_kps_cart=position_kps_cart,
            force_kps_cart=force_kps_cart,
            selection=S
        )
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                traj_gen_proto_msg, SensorDataMessageType.FORCE_POSITION),
            feedback_controller_sensor_msg=sensor_proto2ros_msg(
                fb_ctrlr_proto, SensorDataMessageType.FORCE_POSITION_GAINS)
            )
        pub.publish(ros_msg)
        rate.sleep()

    fa.stop_skill()

    rospy.loginfo('Done')
