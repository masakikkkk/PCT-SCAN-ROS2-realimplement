"""Public launch argument checks with fake launch transport, not a ROS launch."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest


@pytest.fixture
def launcher(monkeypatch):
    def module(name, **symbols):
        result = ModuleType(name)
        result.__dict__.update(symbols)
        monkeypatch.setitem(sys.modules, name, result)

    module('ament_index_python.packages', get_package_share_directory=lambda name: '/tmp/' + name)
    module('launch', LaunchDescription=list)
    module('launch.actions', DeclareLaunchArgument=object, OpaqueFunction=object)
    module('launch.substitutions', LaunchConfiguration=lambda name: NS(perform=lambda c: c[name]))
    module('launch_ros.actions', Node=lambda **kw: NS(**kw))
    source = Path(__file__).parents[1] / 'launch/navigation.launch.py'
    spec = importlib.util.spec_from_file_location('_go2_online_launch_test', source)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    context = dict(config=str(Path(__file__).parents[1] / 'config/go2.yaml'),
                   map_mode='online', tomogram_path='', body_height='0.4')
    return value, context


def test_online_launch_needs_no_pickle_and_uses_configured_map(launcher):
    launch, context = launcher
    nodes = launch.setup(context)
    planner = [node for node in nodes if node.package == 'go2_pct_adapter'
               and node.name == 'pct_scan_bridge'][0]
    assert planner.executable == 'online_pct_bridge'
    assert planner.parameters[0]['map_topic'] == '/uslam/cloud_map'
    assert planner.parameters[0]['map_source'] == 'maintained'
    mapper = [node for node in nodes if node.package == 'go2_map_maintenance'][0]
    assert mapper.parameters[1]['planning_frame'] == 'odom'
    assert mapper.parameters[1]['min_confirmed_points'] == 500
    assert planner.parameters[1]['defer_map_loading'] is True
    sport = [node for node in nodes if node.name == 'go2_sport_bridge'][0]
    assert sport.parameters[1]['require_route_valid'] is True


def test_offline_launch_still_requires_a_real_map_file(launcher):
    launch, context = launcher
    context['map_mode'] = 'offline'
    with pytest.raises(RuntimeError, match='tomogram_path'):
        launch.setup(context)


def test_bad_map_mode_is_rejected(launcher):
    launch, context = launcher
    context['map_mode'] = 'typo'
    with pytest.raises(RuntimeError, match='map_mode'):
        launch.setup(context)


def test_external_map_mode_does_not_launch_accumulator(launcher, tmp_path):
    import yaml
    launch, context = launcher
    settings = yaml.safe_load(Path(context['config']).read_text())
    settings['go2_online_planner']['ros__parameters']['map_source'] = 'external'
    config = tmp_path / 'external.yaml'
    config.write_text(yaml.safe_dump(settings))
    context['config'] = str(config)
    assert not any(node.package == 'go2_map_maintenance' for node in launch.setup(context))


def test_accumulation_refuses_sensor_frame_clouds(launcher, tmp_path):
    import yaml
    launch, context = launcher
    settings = yaml.safe_load(Path(context['config']).read_text())
    settings['go2_state_adapter']['ros__parameters']['cloud_is_world'] = False
    config = tmp_path / 'local.yaml'
    config.write_text(yaml.safe_dump(settings))
    context['config'] = str(config)
    with pytest.raises(RuntimeError, match='world-frame'):
        launch.setup(context)


def test_accumulation_requires_heartbeat_timeout_longer_than_publication_period(launcher, tmp_path):
    import yaml
    launch, context = launcher
    settings = yaml.safe_load(Path(context['config']).read_text())
    settings['go2_map_maintenance']['ros__parameters']['publish_rate_hz'] = 0.1
    config = tmp_path / 'too-slow.yaml'
    config.write_text(yaml.safe_dump(settings))
    context['config'] = str(config)
    with pytest.raises(RuntimeError, match='heartbeat'):
        launch.setup(context)
