from launch import LaunchDescription
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
    
    # MTC Demo node
    pick_place_demo = Node(
        package="mtc_tutorial",
        executable="mtc_tutorial",
        output="screen",
        parameters=[
            moveit_config,
    #        move_group_capabilities,
        ],
    )

    return LaunchDescription([
      pick_place_demo,
      run_move_group_node
      ])