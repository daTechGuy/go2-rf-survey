from glob import glob

from setuptools import setup

package_name = "go2_survey_wifi"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="daTechGuy",
    maintainer_email="brian.a.dye@gmail.com",
    description="WiFi scan sweeps joined to SLAM poses for RF site surveying.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "survey_node = go2_survey_wifi.survey_node:main",
            # Standalone adapter check -- confirms a radio works before ROS does.
            "scan_once = go2_survey_wifi.scanner:_main",
        ],
    },
)
