from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'robot_navigation'

def generate_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    
    # Collect all launch, config, rviz, bridge files and keep their folder structure
    for root, dirs, files in os.walk('src/robot_navigation'):
        for file in files:
            if file.endswith(('.py', '.yaml', '.rviz', '.sdf', '.xml')) and not file == '__init__.py' and not root == 'src/robot_navigation':
                # Determine destination path in share
                rel_path = os.path.relpath(root, 'src/robot_navigation')
                dest_path = os.path.join('share', package_name, rel_path)
                src_path = os.path.join(root, file)
                
                # Check if dest_path already in data_files
                found = False
                for df in data_files:
                    if df[0] == dest_path:
                        df[1].append(src_path)
                        found = True
                        break
                if not found:
                    data_files.append((dest_path, [src_path]))
                    
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=generate_data_files(),
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='aarush-sivaraman',
    maintainer_email='aarush@example.com',
    description='Robot navigation package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'world_analyzer = robot_navigation.world_analyzer.__main__:main',
            'frontier_explorer = robot_navigation.exploration.frontier_explorer:main',
        ],
    },
)
