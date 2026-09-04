from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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
                'shelf_column_number': shelf_column_number,
                'book_colour': book_colour,
                'output_directory': output_directory,
            }],
        ),
    ])
