import urdfpy
import numpy as np
import pathlib
import stretch_body.hello_utils as hu

# NOTE: copied from https://github.com/hello-robot/stretch_tutorials/blob/master/stretch_body/jupyter/inverse_kinematics.ipynb

def make_ik_urdf(save_path: str):
    """
    Creates a simplified URDF file for use with the IK solver.

    Args:
        save_path (str): Path to save the modified URDF file.
    """
    
    # load urdf
    print("Loading original URDF... ", end=None)
    urdf_path = str((pathlib.Path(hu.get_fleet_directory()) / 'exported_urdf' / 'stretch.urdf').absolute())
    original_urdf = urdfpy.URDF.load(urdf_path)
    print("done!")

    print(f"name: {original_urdf.name}")
    print(f"num links: {len(original_urdf.links)}")
    print(f"num joints: {len(original_urdf.joints)}")

    # remove the links and joints we don't need
    modified_urdf = original_urdf
    names_of_links_to_remove = ['link_right_wheel', 'link_left_wheel', 'caster_link', 'link_gripper_finger_left', 'link_gripper_fingertip_left', 'link_gripper_finger_right', 'link_gripper_fingertip_right', 'link_head', 'link_head_pan', 'link_head_tilt', 'link_aruco_right_base', 'link_aruco_left_base', 'link_aruco_shoulder', 'link_aruco_top_wrist', 'link_aruco_inner_wrist', 'camera_bottom_screw_frame', 'camera_link', 'camera_depth_frame', 'camera_depth_optical_frame', 'camera_infra1_frame', 'camera_infra1_optical_frame', 'camera_infra2_frame', 'camera_infra2_optical_frame', 'camera_color_frame', 'camera_color_optical_frame', 'camera_accel_frame', 'camera_accel_optical_frame', 'camera_gyro_frame', 'camera_gyro_optical_frame', 'laser', 'respeaker_base']
    links_to_remove = [l for l in modified_urdf._links if l.name in names_of_links_to_remove]
    for lr in links_to_remove:
        modified_urdf._links.remove(lr)
    names_of_joints_to_remove = ['joint_right_wheel', 'joint_left_wheel', 'caster_joint', 'joint_gripper_finger_left', 'joint_gripper_fingertip_left', 'joint_gripper_finger_right', 'joint_gripper_fingertip_right', 'joint_head', 'joint_head_pan', 'joint_head_tilt', 'joint_aruco_right_base', 'joint_aruco_left_base', 'joint_aruco_shoulder', 'joint_aruco_top_wrist', 'joint_aruco_inner_wrist', 'camera_joint', 'camera_link_joint', 'camera_depth_joint', 'camera_depth_optical_joint', 'camera_infra1_joint', 'camera_infra1_optical_joint', 'camera_infra2_joint', 'camera_infra2_optical_joint', 'camera_color_joint', 'camera_color_optical_joint', 'camera_accel_joint', 'camera_accel_optical_joint', 'camera_gyro_joint', 'camera_gyro_optical_joint', 'joint_laser', 'joint_respeaker']
    joints_to_remove = [l for l in modified_urdf._joints if l.name in names_of_joints_to_remove]
    for jr in joints_to_remove:
        modified_urdf._joints.remove(jr)
    
    # print(f"name: {modified_urdf.name}")
    print("\nDecimated URDF info: ")
    print(f"num links: {len(modified_urdf.links)}")
    print(f"num joints: {len(modified_urdf.joints)}")

    # add the base translation joint and link
    joint_base_translation = urdfpy.Joint(name='joint_base_translation',
                                        parent='base_link',
                                        child='link_base_translation',
                                        joint_type='prismatic',
                                        axis=np.array([1.0, 0.0, 0.0]),
                                        origin=np.eye(4, dtype=np.float64),
                                        limit=urdfpy.JointLimit(effort=100.0, velocity=1.0, lower=-1.0, upper=1.0))
    modified_urdf._joints.append(joint_base_translation)
    link_base_translation = urdfpy.Link(name='link_base_translation',
                                        inertial=None,
                                        visuals=None,
                                        collisions=None)
    modified_urdf._links.append(link_base_translation)

    # amend the chain
    for j in modified_urdf._joints:
        if j.name == 'joint_mast':
            j.parent = 'link_base_translation'
    # print(f"name: {modified_urdf.name}")
    print("\nFinal URDF info:")
    print(f"num links: {len(modified_urdf.links)}")
    print(f"num joints: {len(modified_urdf.joints)}")

    # save
    if not save_path.endswith('.urdf'):
        save_path += '.urdf'

    print("\nSaving to {}".format(save_path))
    modified_urdf.save(save_path)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    # parser.add_argument('--urdf_path', type=str, help="Path to the original URDF file.")
    parser.add_argument('--save_path', type=str, help="Path to save the modified URDF file.")
    args = parser.parse_args()

    try:
        make_ik_urdf(args.save_path)
    except Exception as e:
        print(e)
