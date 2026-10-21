from glob import glob
from setuptools import find_packages, setup

setup(
    name="go2_pct_adapter",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/go2_pct_adapter"]),
        ("share/go2_pct_adapter", ["package.xml", "LICENSE"]),
        ("share/go2_pct_adapter/config", glob("config/*.yaml")),
        ("share/go2_pct_adapter/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Go2 navigation team",
    maintainer_email="maintainer@example.com",
    description="Foxy interfaces for Go2 PCT/SCAN navigation",
    url="https://github.com/yagami-light7/PCT-SCAN-ROS2",
    license="GPL-2.0-or-later",
    entry_points={"console_scripts": [
        "state_adapter = go2_pct_adapter.state_adapter:main",
        "sport_bridge = go2_pct_adapter.sport_bridge:main",
        "live_pct_bridge = go2_pct_adapter.live_pct_bridge:main",
        "online_pct_bridge = go2_pct_adapter.online_pct_bridge:main",
        "preflight = go2_pct_adapter.preflight:main",
    ]},
)
