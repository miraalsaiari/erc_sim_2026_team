"""Start the perception node with typed target and camera parameters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Expose the standalone perception interface."""
    shelf_column_number = LaunchConfiguration(
        'shelf_column_number'
    )
    book_colour = LaunchConfiguration('book_colour')
    output_directory = LaunchConfiguration(
        'output_directory'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'shelf_column_number',
            default_value='1',
            description='Requested shelf column number (1 to 5)',
        ),
        DeclareLaunchArgument(
            'book_colour',
            default_value='red',
            description=(
                'Requested book colour: '
                'red, green, yellow, or blue'
            ),
        ),
        DeclareLaunchArgument(
            'template_directory',
            default_value='',
            description='Digit templates; default uses erc_description',
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value=(
                '/head_front_camera/head_front_camera/color/image_raw'),
            description='RGB camera topic',
        ),
        DeclareLaunchArgument(
            'confirmation_frames', default_value='5',
            description='Consecutive matching frames before publishing',
        ),
        DeclareLaunchArgument(
            'processing_rate', default_value='5.0',
            description='Maximum image processing rate in Hz',
        ),
        DeclareLaunchArgument(
            'output_directory',
            default_value='/opt/erc_ws/erc_images',
            description='Directory for annotated result images',
        ),
        Node(
            package='erc_perception',
            executable='camera_subscriber',
            name='camera_subscriber',
            output='screen',
            parameters=[{
                'shelf_column_number': ParameterValue(
                    shelf_column_number, value_type=int),
                'book_colour': ParameterValue(book_colour, value_type=str),
                'output_directory': ParameterValue(
                    output_directory, value_type=str),
                'template_directory': ParameterValue(
                    LaunchConfiguration('template_directory'), value_type=str),
                'camera_topic': ParameterValue(
                    LaunchConfiguration('camera_topic'), value_type=str),
                'confirmation_frames': ParameterValue(
                    LaunchConfiguration('confirmation_frames'),
                    value_type=int),
                'processing_rate': ParameterValue(
                    LaunchConfiguration('processing_rate'), value_type=float),
            }],
        ),
    ])
