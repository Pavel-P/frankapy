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

    rospy.loginfo('Loading trajectory data')

    with open(args.trajectory_pickle, 'rb') as pkl_f:
        skill_data = pkl.load(pkl_f)
    
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

    rospy.loginfo('Moving to standby')
    standby_joints = np.array([ 0.04426385,  0.05210685, -0.03669728, -2.47104876,
                                0.00424517,  2.524196  ,  0.77732302])
    fa.goto_joints(standby_joints)
    
    rospy.loginfo('Moving to surface')
    surface_joints = np.array([ 0.03829632,  0.33102359, -0.02919324, -2.46936744,
                                0.00428743,  2.78393814,  0.78432293])
    fa.goto_joints(surface_joints, force_thresholds=[1e6,1e6,force,1e6,1e6,1e6])


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
            pose=pose_traj[i],
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

    rospy.sleep(3.)
    rospy.loginfo('Returning to standby')
    standby_joints = np.array([ 0.04426385,  0.05210685, -0.03669728, -2.47104876,
                                0.00424517,  2.524196  ,  0.77732302])
    fa.goto_joints(standby_joints)

    rospy.loginfo('Done')
