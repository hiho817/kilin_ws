from setuptools import find_packages, setup


package_name = "kilin_motion_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="biorola",
    maintainer_email="kuojack0114@gmail.com",
    description="Runtime motion-planning library used by Kilin ROS controllers.",
    license="Apache-2.0",
)
