from glob import glob

from setuptools import setup

package_name = "go2_survey_bridge"

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
    description="/cmd_vel to Go2 sport-mode bridge with watchdog and e-stop.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge_node = go2_survey_bridge.bridge_node:main",
        ],
    },
)
