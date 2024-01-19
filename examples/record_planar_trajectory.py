import pickle as pkl
import numpy as np
import argparse

from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import ForcePositionSensorMessage, ForcePositionControllerSensorMessage
from franka_interface_msgs.msg import SensorDataGroup
from frankapy.utils import transform_to_list, convert_rigid_transform_to_array

from tqdm import trange

import rospy

def create_formated_skill_dict(joints, end_effector_positions, time_since_skill_started,
                               time, rate, target_force):
    skill_dict = dict(skill_description='PlanarGuideMode', skill_state_dict=dict())
    skill_dict['skill_state_dict']['q'] = np.array(joints)
    skill_dict['skill_state_dict']['O_T_EE'] = np.array(end_effector_positions)
    skill_dict['skill_state_dict']['time_since_skill_started'] = np.array(time_since_skill_started)
    skill_dict['time'] = time
    skill_dict['rate'] = rate
    skill_dict['force'] = target_force

    # The key (0 here) usually represents the absolute time when the skill was started but
    formatted_dict = {0: skill_dict}
    return formatted_dict


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--time', '-t', type=float, default=20)
    parser.add_argument('--rate', type=float, default=60)
    parser.add_argument('--force', type=float, default=8)
    parser.add_argument('--file', '-f', default='franka_traj.pkl')
    args = parser.parse_args()

    fa = FrankaArm()

    rospy.loginfo('Moving to standby')
    standby_joints = np.array([ 0.04426385,  0.05210685, -0.03669728, -2.47104876,
                                0.00424517,  2.524196  ,  0.77732302])
    fa.goto_joints(standby_joints)
    
    rospy.loginfo('Moving to surface')
    surface_joints = np.array([ 0.03829632,  0.33102359, -0.02919324, -2.46936744,
                                0.00428743,  2.78393814,  0.78432293])
    fa.goto_joints(surface_joints, force_thresholds=[1e6,1e6,args.force,1e6,1e6,1e6])

    rospy.loginfo('Moving to standby pose')
    rospy.loginfo('Moving to contact pose')

    T = args.time
    target_force = [0, 0, -args.force, 0, 0, 0]
    S = [1, 1, 0, 1, 1, 1]

    tz = FC.DEFAULT_TRANSLATIONAL_STIFFNESSES
    rz = FC.DEFAULT_ROTATIONAL_STIFFNESSES
    tz = [0, 0, tz[2]] 
    position_kps_cart = tz + rz
    force_kps_cart = [0.1] * 6

    rospy.loginfo('Initializing Sensor Publisher')
    pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1)
    rate = rospy.Rate(args.rate)
    p0 = transform_to_list(fa.get_pose())

    end_effector_position = []
    joints = []
    time_since_skill_started = []

    rospy.loginfo('Recording trajectory...')
    fa.run_dynamic_force_position(duration=T, buffer_time=3, S=S,
                                use_cartesian_gains=True,
                                position_kps_cart=position_kps_cart,
                                force_kps_cart=force_kps_cart)
    init_time = rospy.Time.now().to_time()
    N = T * args.rate
    for i in trange(N):
        t = i % N
        timestamp = rospy.Time.now().to_time() - init_time
        traj_gen_proto_msg = ForcePositionSensorMessage(
            id=i, timestamp=timestamp, seg_run_time=1./args.rate,
            pose=p0,
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
        pose_array = convert_rigid_transform_to_array(fa.get_pose())
        end_effector_position.append(pose_array)
        joints.append(fa.get_joints())
        time_since_skill_started.append(timestamp)

        pub.publish(ros_msg)
        rate.sleep()

    fa.stop_skill()

    skill_dict = create_formated_skill_dict(joints, end_effector_position, time_since_skill_started,
                                            args.time, args.rate, args.force)
    with open(args.file, 'wb') as pkl_f:
        pkl.dump(skill_dict, pkl_f)
        print("Did save skill dict: {}".format(args.file))

    rospy.loginfo('Done recording.')
    rospy.sleep(3.)
    rospy.loginfo('Returning to standby.')
    standby_joints = np.array([ 0.04426385,  0.05210685, -0.03669728, -2.47104876,
                                0.00424517,  2.524196  ,  0.77732302])
    fa.goto_joints(standby_joints)

    rospy.loginfo('Done')
