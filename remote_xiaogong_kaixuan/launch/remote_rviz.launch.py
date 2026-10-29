"""Local RViz only. All feedback, planning, and execution come from the robot."""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

ROOT = Path(__file__).resolve().parents[1]


def read_yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


def client_parameters():
    # Use portable file URIs, so the bundle needs no colcon build or local ROS package.
    robot = ET.parse(ROOT / "model/robot.urdf").getroot()
    for mesh in robot.findall(".//mesh"):
        filename = mesh.get("filename", "")
        prefix = "package://xiaogong_kaixuan/meshes/"
        if not filename.startswith(prefix):
            raise ValueError(f"Unexpected mesh resource: {filename}")
        asset = ROOT / "model/meshes" / filename.removeprefix(prefix)
        if not asset.is_file():
            raise FileNotFoundError(asset)
        mesh.set("filename", asset.resolve().as_uri())
    defaults = Path(get_package_share_directory("moveit_configs_utils")) / "default_configs"
    ompl = read_yaml(defaults / "ompl_planning.yaml")
    ompl.update(read_yaml(defaults / "ompl_defaults.yaml"))
    return {
        "robot_description": ParameterValue(ET.tostring(robot, encoding="unicode"), value_type=str),
        "robot_description_semantic": ParameterValue((ROOT / "config/kaixuan1.srdf").read_text(), value_type=str),
        "robot_description_kinematics": read_yaml(ROOT / "config/kinematics.yaml"),
        "robot_description_planning": read_yaml(ROOT / "config/joint_limits.yaml"),
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl,
        "use_sim_time": False,
    }


def launch_rviz(context):
    arm = LaunchConfiguration("arm").perform(context)
    return [Node(
        package="rviz2", executable="rviz2", name="kaixuan_remote_rviz", output="screen",
        arguments=["-d", str(ROOT / f"config/remote_{arm}.rviz")],
        parameters=[client_parameters()],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arm", default_value="left", choices=["left", "right", "both"],
                              description="RViz 默认规划组，应与机器人启用的机械臂对应"),
        OpaqueFunction(function=launch_rviz),
    ])
