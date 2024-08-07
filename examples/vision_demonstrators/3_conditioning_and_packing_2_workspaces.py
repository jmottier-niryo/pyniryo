"""
This script shows an example of how to use Ned's vision with 2
workspaces : 1 for picking & 1 for packing

The robot will firstly detect one pack place, then it will take one object
from the picking area. This object will be pack and then, the pack will be
place on the side of the working area
"""

from pyniryo import NiryoRobot, PoseObject, ObjectShape, ObjectColor

# -- MUST Change these variables
robot_ip_address = '<robot_ip_address>'  # IP address of the robot
workspace_pick_name = "workspace_1"  # Robot's picking Workspace Name
workspace_place_name = "workspace_place"  # Robot's placing Workspace Name

# -- Should Change these variables
# The pose from where the image processing for picking happens
observation_pose_wks_pick = PoseObject(0.2, -0.0, 0.3, 3.14, -0.01, -0.02)
# The pose from where the image processing for placing happens
observation_pose_wks_place = PoseObject(0.0, -0.2, 0.35, 3.14, -0.01, -1.55)
# First conditioning pose in the cubes row
pos_conditioning1 = PoseObject(0.1, -0.29, 0.12, 3.14, 0.0, -1.57)
# First conditioning pose in the circles row
pos_conditioning2 = PoseObject(-0.14, -0.28, 0.12, 3.14, -0.01, -1.58)

# Joints where the robot goes at the end of its process
sleep_joints = [0.0, 0.55, -1.2, 0.0, 0.0, 0.0]


def process(niryo_robot: NiryoRobot):
    catch_count_category_1 = 0
    catch_count_category_2 = 0
    try_without_success = 0
    # Loop until too many failures or 6 objects have been already picked
    while try_without_success < 3 and catch_count_category_1 + catch_count_category_2 < 6:
        obj_pose = None
        # Moving to observation pose over the packing workspace
        niryo_robot.move(observation_pose_wks_place)
        # Trying to get place pose from python ROS wrapper
        ret = niryo_robot.get_target_pose_from_cam(workspace_place_name,
                                                   height_offset=0.0,
                                                   shape=ObjectShape.ANY,
                                                   color=ObjectColor.ANY)
        # Unpacking return result
        obj_found, place_pose, _, _ = ret
        if not obj_found:  # Aborting iteration if issue
            try_without_success += 1
            continue

        # Moving to observation pose over the picking workspace
        niryo_robot.move(observation_pose_wks_pick)

        try_without_success_within = 0
        obj_found_within = False
        should_leave = False
        shape_obj = None
        while not obj_found_within:  # Looping over picking area
            ret = niryo_robot.get_target_pose_from_cam(workspace_pick_name,
                                                       height_offset=0.0,
                                                       shape=ObjectShape.ANY,
                                                       color=ObjectColor.ANY)
            obj_found_within, obj_pose, shape_obj, _ = ret
            if not obj_found_within:
                try_without_success_within += 1
                if try_without_success_within > 3:
                    should_leave = True
                    break

        if should_leave:  # If nothing has been found, nothing more to pack, leave
            break
        # Everything is good, so we pick the object
        niryo_robot.pick(obj_pose)

        # Packing
        place_pose_high = place_pose.copy_with_offsets(z_offset=0.05)
        niryo_robot.move(place_pose_high)
        niryo_robot.move(place_pose.copy_with_offsets(z_offset=0.01))
        # Opening gripper to pack
        niryo_robot.open_gripper()
        # Going down to pick package
        niryo_robot.move(place_pose.copy_with_offsets(z_offset=-0.011))
        niryo_robot.close_gripper()
        niryo_robot.move(place_pose.copy_with_offsets(z_offset=0.05))

        # Getting place position
        if shape_obj == ObjectShape.SQUARE:
            target_pose = pos_conditioning1.copy_with_offsets(y_offset=0.05 * catch_count_category_1 % 3)
            catch_count_category_1 += 1
        else:
            target_pose = pos_conditioning2.copy_with_offsets(y_offset=0.05 * catch_count_category_2 % 3)
            catch_count_category_2 += 1

        niryo_robot.place(target_pose)

    niryo_robot.move(observation_pose_wks_pick)


if __name__ == '__main__':
    # Connect to robot
    robot = NiryoRobot(robot_ip_address)
    # Changing tool
    robot.update_tool()
    # Calibrate robot if robot needs calibration
    robot.calibrate_auto()
    # Launching main process
    process(robot)
    # Ending
    robot.go_to_sleep()
    # Releasing connection
    robot.close_connection()
