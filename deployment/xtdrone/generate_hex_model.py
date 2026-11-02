"""构造 Hex-X Gazebo 模型；保留 PX4 上游 IMU/气压/GNSS 传感器定义。"""

import argparse
import copy
import math
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scenario_profile import load_profile  # noqa: E402


def add(parent, tag, value=None, **attrs):
    node = ET.SubElement(parent, tag, attrs)
    if value is not None:
        node.text = str(value)
    return node


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--px4", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scenario", type=Path)
    parser.add_argument("--truth-rate-hz", type=int, default=0)
    args = parser.parse_args()
    if not 0 <= args.truth_rate_hz <= 1000:
        parser.error("--truth-rate-hz must be between 0 and 1000")
    scenario = load_profile(args.scenario) if args.scenario else None
    target = args.output / "hex_x12"
    target.mkdir(parents=True, exist_ok=True)
    license_file = args.px4 / "Tools/simulation/gz/LICENSE"
    shutil.copyfile(license_file, target / "LICENSE_PX4_MODELS.txt")
    shutil.copyfile(license_file, args.output / "LICENSE_PX4_MODELS.txt")
    upstream = ET.parse(args.px4 / "Tools/simulation/gz/models/x500_base/model.sdf")
    document = ET.Element("sdf", version="1.9")
    model = add(document, "model", name="hex_x12")
    if args.truth_rate_hz:
        publisher = add(model, "plugin", filename="gz-sim-pose-publisher-system",
                        name="gz::sim::systems::PosePublisher")
        add(publisher, "publish_model_pose", "true")
        add(publisher, "publish_link_pose", "false")
        add(publisher, "publish_visual_pose", "false")
        add(publisher, "publish_collision_pose", "false")
        add(publisher, "publish_sensor_pose", "false")
        add(publisher, "publish_nested_model_pose", "false")
        add(publisher, "use_pose_vector_msg", "true")
        add(publisher, "update_frequency", args.truth_rate_hz)
        add(publisher, "topic", "/uav_motion/truth_pose")
    add(model, "pose", "0 0 0 0 0 0")
    base = add(model, "link", name="base_link")
    if scenario and any(scenario["gazebo"]["wind_velocity_enu_mps"]):
        add(base, "enable_wind", "true")
    inertial = add(base, "inertial")
    rotor_mass = 0.02
    rotor_i_xy = rotor_mass * (3 * 0.12**2 + 0.005**2) / 12
    rotor_i_z = rotor_mass * 0.12**2 / 2
    add(inertial, "mass", 12.0 - 6 * rotor_mass)
    inertia = add(inertial, "inertia")
    for axis, value, subtract, rotor_i in (
        ("ixx", 0.55, 3 * 0.55**2, rotor_i_xy),
        ("iyy", 0.60, 3 * 0.55**2, rotor_i_xy),
        ("izz", 1.0, 6 * 0.55**2, rotor_i_z),
    ):
        add(inertia, axis, value - rotor_mass * subtract - 6 * rotor_i)
    for axis in ("ixy", "ixz", "iyz"):
        add(inertia, axis, 0)
    for kind in ("visual",):
        shape = add(base, kind, name="body_" + kind)
        add(add(add(shape, "geometry"), "box"), "size", "0.3 0.3 0.18")
    for collision in upstream.findall("./model/link[@name='base_link']/collision"):
        base.append(copy.deepcopy(collision))
    for sensor in upstream.findall("./model/link[@name='base_link']/sensor"):
        sensor = copy.deepcopy(sensor)
        if scenario and sensor.get("name") == "imu_sensor":
            for field, scale in (("angular_velocity", scenario["gazebo"]["imu_gyro_noise_scale"]),
                                 ("linear_acceleration", scenario["gazebo"]["imu_accel_noise_scale"])):
                for noise in sensor.findall(f"./imu/{field}/*/noise"):
                    for tag in ("stddev", "dynamic_bias_stddev"):
                        element = noise.find(tag)
                        if element is not None:
                            element.text = format(float(element.text) * scale, ".12g")
        base.append(sensor)
    maximum = math.sqrt(42 / 8e-5)
    idle_speed = 150.0
    # ESC 将归一化速度 u 映射为 w=idle+(maximum-idle)*u，推力为 kf*w²。
    # 去除怠速推力后，归一化推力可写成 a*u²+(1-a)*u。
    model_factor = (maximum - idle_speed) / (maximum + idle_speed)
    idle_thrust = 8e-5 * idle_speed**2
    hover_thrust = (12 * 9.81 / 6 - idle_thrust) / (42 - idle_thrust)
    airframe = [
        "#!/bin/sh",
        ". ${R}etc/init.d/rc.mc_defaults",
        "PX4_SIMULATOR=gz",
        "PX4_SIM_MODEL=hex_x12",
        "PX4_GZ_WORLD=default",
        "param set-default SIM_GZ_EN 1",
        "param set-default SENS_EN_GPSSIM 1",
        "param set-default SENS_EN_BAROSIM 0",
        "param set-default SENS_EN_MAGSIM 1",
        "param set-default CA_AIRFRAME 0",
        "param set-default CA_ROTOR_COUNT 6",
        f"param set-default MPC_THR_HOVER {hover_thrust:.4f}",
        f"param set-default THR_MDL_FAC {model_factor:.3f}",
    ]
    for i in range(6):
        angle = math.radians(30 + 60 * i)
        x, y = 0.55 * math.cos(angle), 0.55 * math.sin(angle)
        link = add(model, "link", name=f"rotor_{i}")
        add(link, "pose", f"{x} {-y} 0 0 0 0")
        ri = add(link, "inertial")
        add(ri, "mass", rotor_mass)
        ii = add(ri, "inertia")
        for axis, value in (("ixx", rotor_i_xy), ("iyy", rotor_i_xy), ("izz", rotor_i_z)):
            add(ii, axis, value)
        visual = add(link, "visual", name="propeller")
        cylinder = add(add(visual, "geometry"), "cylinder")
        add(cylinder, "radius", 0.12)
        add(cylinder, "length", 0.005)
        joint = add(model, "joint", name=f"rotor_{i}_joint", type="revolute")
        add(joint, "parent", "base_link")
        add(joint, "child", f"rotor_{i}")
        add(add(joint, "axis"), "xyz", "0 0 1")
        plugin = add(
            model,
            "plugin",
            filename="gz-sim-multicopter-motor-model-system",
            name="gz::sim::systems::MulticopterMotorModel",
        )
        for tag, value in {
            "jointName": f"rotor_{i}_joint",
            "linkName": f"rotor_{i}",
            "turningDirection": "ccw" if i % 2 == 0 else "cw",
            "timeConstantUp": 0.1,
            "timeConstantDown": 0.1,
            "maxRotVelocity": maximum,
            "motorConstant": 8e-5,
            "momentConstant": 0.03,
            "commandSubTopic": "command/motor_speed",
            "motorNumber": i,
            "rotorDragCoefficient": 0,
            "rollingMomentCoefficient": 0,
            "rotorVelocitySlowdownSim": 10,
            "motorType": "velocity",
        }.items():
            add(plugin, tag, value)
        airframe += [
            f"param set-default CA_ROTOR{i}_PX {x}",
            f"param set-default CA_ROTOR{i}_PY {y}",
            f"param set-default CA_ROTOR{i}_KM {0.03 if i % 2 == 0 else -0.03}",
            f"param set-default SIM_GZ_EC_FUNC{i + 1} {101 + i}",
            f"param set-default SIM_GZ_EC_MIN{i + 1} {idle_speed:g}",
            f"param set-default SIM_GZ_EC_MAX{i + 1} {maximum}",
        ]
    ET.indent(document)
    ET.ElementTree(document).write(target / "model.sdf", encoding="utf-8", xml_declaration=True)
    (target / "model.config").write_text(
        '<model><name>hex_x12</name><version>1.0</version><sdf version="1.9">model.sdf</sdf></model>',
        encoding="utf-8",
    )
    (args.output / "6001_gz_hex_x12").write_text("\n".join(airframe) + "\n", encoding="utf-8")
    world = ET.parse(args.px4 / "Tools/simulation/gz/worlds/default.sdf")
    world.find("./world/physics/max_step_size").text = "0.001"
    world.find("./world/physics/real_time_update_rate").text = "1000"
    world.find("./world/gravity").text = "0 0 -9.81"
    if scenario and any(scenario["gazebo"]["wind_velocity_enu_mps"]):
        wind = scenario["gazebo"]
        world_node = world.find("./world")
        add(add(world_node, "wind"), "linear_velocity",
            " ".join(str(component) for component in wind["wind_velocity_enu_mps"]))
        wind_plugin = add(world_node, "plugin", filename="gz-sim-wind-effects-system",
                          name="gz::sim::systems::WindEffects")
        add(wind_plugin, "force_approximation_scaling_factor", wind["wind_force_scale"])
        magnitude = add(add(wind_plugin, "horizontal"), "magnitude")
        add(magnitude, "time_for_rise", 5)
        gust = add(magnitude, "sin")
        add(gust, "amplitude_percent", wind["wind_gust_fraction"])
        add(gust, "period", wind["wind_gust_period_s"])
    world.write(args.output / "hex_world.sdf", encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    main()
