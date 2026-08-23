from setuptools import setup
setup(name="kilin_local_terrain_mapping",version="0.1.0",packages=["kilin_local_terrain_mapping"],data_files=[("share/ament_index/resource_index/packages",["resource/kilin_local_terrain_mapping"]),("share/kilin_local_terrain_mapping",["package.xml"])],entry_points={"console_scripts":["local_terrain_window=kilin_local_terrain_mapping.node:main"]})
