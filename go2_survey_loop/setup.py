from setuptools import setup

package_name = "go2_survey_loop"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="daTechGuy",
    maintainer_email="brian.a.dye@gmail.com",
    description="Autonomous rectangle loop with obstacle stop; publishes /cmd_vel only.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "loop_node = go2_survey_loop.loop_node:main",
        ],
    },
)
