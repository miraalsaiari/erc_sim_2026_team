import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import copy
from sensor_msgs.msg import JointState
from moveit_msgs.msg import JointConstraint
from sensor_msgs.msg import JointState

class ManipulationNode(Node):

    def __init__(self):
        super().__init__('manipulation_node')
        self.motion_in_progress = False
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_right_controller_raw/follow_joint_trajectory'
        )
        self.current_joint_positions = {}

        self.holding_book = False

        self.bin_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/target_bin_pose',
            self.bin_pose_callback,
            10
        )

        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
)

        self.target_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/target_book_pose',
            self.target_pose_callback,
            10
        )

        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        if self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('Connected to gripper controller!')
        else:
            self.get_logger().warn('Gripper controller not available')

        self.get_logger().info('Manipulation node started')
        self.get_logger().info('Waiting for target book pose...')
        self.get_logger().info('Waiting for MoveIt...')

        if self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('Connected to MoveIt!')
        else:
            self.get_logger().warn('MoveIt action server is not available')
        self.pending_bin_pose = None
    def target_pose_callback(self, msg):

        if self.motion_in_progress:
            self.get_logger().warn(
                'Manipulation already in progress. Ignoring new target.'
            )
            return

        self.motion_in_progress = True
        self.target_book_pose = msg

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        self.get_logger().info(
            f'Book received: x={x:.3f}, y={y:.3f}, z={z:.3f}'
        )

        self.start_grasp_sequence()
    def send_plan_request(self, target_pose):
        goal_msg = MoveGroup.Goal()

        goal_msg.request.group_name = 'right_arm'
        if self.current_motion in [
    'approach',
    'lift',
    'place_down',
    'place_retract'
]:          
            goal_msg.request.path_constraints = self.build_path_constraints()
            self.get_logger().info('Arm posture constraints enabled')
        goal_msg.request.start_state.is_diff = True

        goal_msg.request.max_velocity_scaling_factor = 0.2
        goal_msg.request.max_acceleration_scaling_factor = 0.2

        # PLAN ONLY — do not execute yet
        goal_msg.planning_options.plan_only = False

        constraints = Constraints()

        # -----------------------------
        # Position constraint
        # -----------------------------
        position_constraint = PositionConstraint()

        position_constraint.header.frame_id = target_pose.header.frame_id
        position_constraint.link_name = 'arm_right_tool_link'

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]

        position_constraint.constraint_region.primitives.append(sphere)
        position_constraint.constraint_region.primitive_poses.append(
            target_pose.pose
        )

        position_constraint.weight = 1.0

        # -----------------------------
        # Orientation constraint
        # -----------------------------
        orientation_constraint = OrientationConstraint()

        orientation_constraint.header.frame_id = target_pose.header.frame_id
        orientation_constraint.link_name = 'arm_right_tool_link'
        orientation_constraint.orientation = target_pose.pose.orientation

        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1

        orientation_constraint.weight = 1.0

        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        goal_msg.request.goal_constraints.append(constraints)

        self.get_logger().info('Sending plan request to MoveIt...')

        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('MoveIt rejected the goal')
            self.motion_in_progress = False
            return

        self.get_logger().info('MoveIt accepted the goal')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        error_code = result.error_code.val

        if error_code != 1:
            self.get_logger().error(
                f'Motion failed. MoveIt error code: {error_code}'
            )
            self.motion_in_progress = False
            return

        # Pre-grasp finished
        if self.current_motion == 'pregrasp':
            self.get_logger().info('Pre-grasp reached')

            self.get_logger().info('Step 3: Approaching book')

            self.current_motion = 'approach'
            self.send_plan_request(self.target_book_pose)

        # Book pose reached
        elif self.current_motion == 'approach':
            self.get_logger().info('Book pose reached')

            self.get_logger().info('Step 4: Closing gripper')
            self.close_gripper()

        elif self.current_motion == 'place_above':
            self.get_logger().info('Reached above bin')
            self.get_logger().info('Lowering book into bin')

            self.current_motion = 'place_down'
            self.send_plan_request(self.target_bin_pose)


        elif self.current_motion == 'place_down':
            self.get_logger().info('Book positioned in bin')
            self.get_logger().info('Opening gripper to release book')

            self.open_gripper_for_place()


        elif self.current_motion == 'place_retract':
            self.get_logger().info('Arm retracted from bin')
            self.get_logger().info('PLACE SEQUENCE COMPLETE')

            self.holding_book = False
            self.motion_in_progress = False

        elif self.current_motion == 'lift':
            self.get_logger().info('Book lifted!')
            self.get_logger().info('GRASP SEQUENCE COMPLETE')

            self.holding_book = True
            self.motion_in_progress = False

            # If the bin pose arrived early, start placement now
            if self.pending_bin_pose is not None:
                self.get_logger().info(
                    'Saved bin pose found. Starting place sequence.'
                )

                bin_pose = self.pending_bin_pose
                self.pending_bin_pose = None

                self.bin_pose_callback(bin_pose)

    def open_gripper(self):
        goal = FollowJointTrajectory.Goal()

        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = [
            'gripper_right_finger_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.04]   # 4 cm = open
        point.time_from_start.sec = 2

        goal.trajectory.points = [point]

        self.get_logger().info('Opening gripper...')

        self.gripper_client.send_goal_async(goal)


    def close_gripper(self):
        goal = FollowJointTrajectory.Goal()

        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = ['gripper_right_finger_joint']

        point = JointTrajectoryPoint()
        point.positions = [0.00]
        point.time_from_start.sec = 2

        goal.trajectory.points = [point]

        self.get_logger().info('Closing gripper...')

        future = self.gripper_client.send_goal_async(goal)
        future.add_done_callback(self.close_goal_callback)

    def start_grasp_sequence(self):

        # Make a copy of the book pose
        self.pregrasp_pose = copy.deepcopy(self.target_book_pose)

        # Stay 10 cm in front of the book
        self.pregrasp_pose.pose.position.x -= 0.05

        # Make the lift pose
        self.lift_pose = copy.deepcopy(self.target_book_pose)

        # Lift the book 5 cm upward
        self.lift_pose.pose.position.z += 0.03

        self.get_logger().info('Starting grasp sequence')

        self.get_logger().info('Step 1: Open gripper')
        self.open_gripper()

    def open_gripper(self):
        goal = FollowJointTrajectory.Goal()

        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = ['gripper_right_finger_joint']

        point = JointTrajectoryPoint()
        point.positions = [0.04]
        point.time_from_start.sec = 2

        goal.trajectory.points = [point]

        self.get_logger().info('Opening gripper...')

        future = self.gripper_client.send_goal_async(goal)
        future.add_done_callback(self.open_goal_callback)

    def open_goal_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Open gripper command rejected')
            self.motion_in_progress = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.open_finished_callback)


    def open_finished_callback(self, future):
        self.get_logger().info('Gripper opened')
        self.get_logger().info('Skipping pre-grasp for test')

        self.current_motion = 'approach'
        self.send_plan_request(self.target_book_pose)


    def close_goal_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Close gripper command rejected')
            self.motion_in_progress = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.close_finished_callback)


    def close_finished_callback(self, future):
        self.get_logger().info('Gripper closed')

        self.get_logger().info('Step 5: Lifting book')

        self.current_motion = 'lift'
        self.send_plan_request(self.lift_pose)


    def build_path_constraints(self):
        constraints = Constraints()

        # Joint tolerances in radians.
        # Smaller number = joint is allowed to move less.
        joint_tolerances = {

            'arm_right_7_joint': 0.15,
        }
        for joint_name, tolerance in joint_tolerances.items():

            if joint_name not in self.current_joint_positions:
                continue

            joint_constraint = JointConstraint()

            joint_constraint.joint_name = joint_name
            joint_constraint.position = self.current_joint_positions[joint_name]

            joint_constraint.tolerance_above = tolerance
            joint_constraint.tolerance_below = tolerance

            joint_constraint.weight = 1.0

            constraints.joint_constraints.append(joint_constraint)

        return constraints

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name.startswith('arm_right_'):
                self.current_joint_positions[name] = position


    def bin_pose_callback(self, msg):

        if not self.holding_book:
            self.pending_bin_pose = copy.deepcopy(msg)

            self.get_logger().info(
                'Bin pose saved. Waiting for grasp to complete.'
            )
            return

        if self.motion_in_progress:
            self.get_logger().warn(
                'Manipulation already in progress.'
            )
            return

        self.motion_in_progress = True
        self.target_bin_pose = msg

        # Pose above the bin
        self.above_bin_pose = copy.deepcopy(msg)
        self.above_bin_pose.pose.position.z += 0.05

        # Pose used after releasing the book
        self.retract_pose = copy.deepcopy(self.above_bin_pose)

        self.get_logger().info('Bin pose received')
        self.get_logger().info('Starting place sequence')

        self.current_motion = 'place_above'
        self.send_plan_request(self.above_bin_pose)


    def open_gripper_for_place(self):
        goal = FollowJointTrajectory.Goal()

        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = [
            'gripper_right_finger_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.04]
        point.time_from_start.sec = 2

        goal.trajectory.points = [point]

        future = self.gripper_client.send_goal_async(goal)
        future.add_done_callback(self.place_open_goal_callback)

    def place_open_goal_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                'Gripper release command rejected'
            )
            self.motion_in_progress = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.place_open_finished_callback
        )

    def place_open_finished_callback(self, future):
        self.get_logger().info('Book released')

        self.get_logger().info('Retracting arm from bin')

        self.current_motion = 'place_retract'
        self.send_plan_request(self.retract_pose)

def main(args=None):
    rclpy.init(args=args)

    node = ManipulationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()