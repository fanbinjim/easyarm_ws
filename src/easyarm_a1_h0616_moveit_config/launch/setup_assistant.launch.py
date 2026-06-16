from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("easyarm_a1", package_name="easyarm_a1_h0616_moveit_config").to_moveit_configs()
    return generate_setup_assistant_launch(moveit_config)
