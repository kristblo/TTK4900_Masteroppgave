from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    #moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()
    moveit_config = MoveItConfigsBuilder("orca").to_dict() #seems to append "_moveit_config" and load that package on its own
    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

    run_move_group_node = Node(
      package="moveit_ros_move_group",
      executable="move_group",
      output="screen",
      parameters=[
        moveit_config,
        move_group_capabilities
      ],
    )
    
    launch_argument = DeclareLaunchArgument(
        "objName",
        description="Which object to use the script on",
        default_value="bottle0"
    )
    
    
    # Bartender tasks node
    bartender_tasks = Node(
        package="bartender_tasks",
        executable="bartender_tasks",
        output="screen",
        parameters=[
            moveit_config,
            {"objName": "bottle1"}
    #        move_group_capabilities,
        ]
    )

    return LaunchDescription(
      [launch_argument] +
      [
      bartender_tasks,
      run_move_group_node,
      ])