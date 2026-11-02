import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from numpy.testing import assert_allclose


def test_gazebo_hex_mass_inertia_and_thrust_match_baseline(tmp_path):
    source = tmp_path / "px4/Tools/simulation/gz/models/x500_base"
    source.mkdir(parents=True)
    (tmp_path / "px4/Tools/simulation/gz/LICENSE").write_text("Test fixture license")
    (source / "model.sdf").write_text(
        '<sdf><model><link name="base_link"><sensor name="imu_sensor" type="imu"/></link></model></sdf>'
    )
    worlds = tmp_path / "px4/Tools/simulation/gz/worlds"
    worlds.mkdir(parents=True)
    (worlds / "default.sdf").write_text(
        "<sdf><world><physics><max_step_size>0.004</max_step_size>"
        "<real_time_update_rate>250</real_time_update_rate></physics><gravity>0 0 -9.8</gravity></world></sdf>"
    )
    script = Path(__file__).resolve().parents[1] / "deployment/xtdrone/generate_hex_model.py"
    output = tmp_path / "models"
    subprocess.run(
        [sys.executable, str(script), "--px4", str(tmp_path / "px4"), "--output", str(output)], check=True
    )
    model = ET.parse(output / "hex_x12/model.sdf").getroot().find("model")
    total_mass = 0.0
    inertia = np.zeros(3)
    for link in model.findall("link"):
        mass = float(link.findtext("inertial/mass"))
        position = np.array([float(v) for v in link.findtext("pose", "0 0 0 0 0 0").split()[:3]])
        total_mass += mass
        inertia += [float(link.findtext("inertial/inertia/" + name)) for name in ("ixx", "iyy", "izz")]
        inertia += mass * (position @ position - position**2)
    assert_allclose(total_mass, 12.0, atol=1e-12)
    assert_allclose(inertia, [0.55, 0.60, 1.0], atol=1e-12)
    plugins = model.findall("plugin")
    assert len(plugins) == 6
    for plugin in plugins:
        coefficient = float(plugin.findtext("motorConstant"))
        maximum = float(plugin.findtext("maxRotVelocity"))
        assert_allclose(coefficient * maximum**2, 42.0, atol=1e-10)
        assert float(plugin.findtext("timeConstantUp")) == 0.1


def test_scenario_build_adds_physical_wind_and_imu_noise(tmp_path):
    source = tmp_path / "px4/Tools/simulation/gz/models/x500_base"
    source.mkdir(parents=True)
    (tmp_path / "px4/Tools/simulation/gz/LICENSE").write_text("Test fixture license")
    (source / "model.sdf").write_text(
        '<sdf><model><link name="base_link"><sensor name="imu_sensor" type="imu"><imu>'
        '<angular_velocity><x><noise><stddev>0.01</stddev></noise></x></angular_velocity>'
        '<linear_acceleration><x><noise><stddev>0.02</stddev></noise></x></linear_acceleration>'
        '</imu></sensor></link></model></sdf>'
    )
    worlds = tmp_path / "px4/Tools/simulation/gz/worlds"
    worlds.mkdir(parents=True)
    (worlds / "default.sdf").write_text(
        "<sdf><world><physics><max_step_size>0.004</max_step_size>"
        "<real_time_update_rate>250</real_time_update_rate></physics><gravity>0 0 -9.8</gravity></world></sdf>"
    )
    script = Path(__file__).resolve().parents[1] / "deployment/xtdrone/generate_hex_model.py"
    profile = Path(__file__).resolve().parents[1] / "deployment/scenarios/crosswind_sensor.json"
    output = tmp_path / "models"
    subprocess.run([sys.executable, str(script), "--px4", str(tmp_path / "px4"),
                    "--output", str(output), "--scenario", str(profile)], check=True)
    model = ET.parse(output / "hex_x12/model.sdf").getroot()
    assert model.findtext("./model/link[@name='base_link']/enable_wind") == "true"
    assert float(model.findtext("./model/link[@name='base_link']/sensor/imu/angular_velocity/x/noise/stddev")) == 0.02
    assert float(model.findtext("./model/link[@name='base_link']/sensor/imu/linear_acceleration/x/noise/stddev")) == 0.04
    world = ET.parse(output / "hex_world.sdf").getroot()
    assert world.findtext("./world/wind/linear_velocity") == "-2.0 0.0 0.0"
    assert float(world.findtext("./world/plugin[@name='gz::sim::systems::WindEffects']/force_approximation_scaling_factor")) == 0.1
