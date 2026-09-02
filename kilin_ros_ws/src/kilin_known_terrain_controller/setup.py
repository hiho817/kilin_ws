from glob import glob
from setuptools import find_packages, setup


package_name = "kilin_known_terrain_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    test_suite="test",
    zip_safe=True,
    maintainer="biorola",
    maintainer_email="kuojack0114@gmail.com",
    description="Version 2 online known-terrain controller for Kilin",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "known_terrain_controller = "
            "kilin_known_terrain_controller.controller_node:main",
            "run_real_terrain_trial = "
            "kilin_known_terrain_controller.trial_runner:main",
        ],
    },
)
