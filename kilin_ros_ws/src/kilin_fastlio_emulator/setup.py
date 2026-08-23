from glob import glob
from setuptools import setup
setup(name="kilin_fastlio_emulator", version="0.1.0", packages=["kilin_fastlio_emulator"], data_files=[("share/ament_index/resource_index/packages", ["resource/kilin_fastlio_emulator"]), ("share/kilin_fastlio_emulator", ["package.xml"]), ("share/kilin_fastlio_emulator/launch", glob("launch/*.launch.py"))], install_requires=["setuptools"], zip_safe=True, entry_points={"console_scripts": ["fastlio_odometry_emulator = kilin_fastlio_emulator.odometry_emulator:main"]})
