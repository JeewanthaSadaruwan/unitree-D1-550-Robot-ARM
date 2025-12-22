from setuptools import setup, find_packages

package_name = 'gp7_boxes'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GP7 Team',
    maintainer_email='gp7@example.com',
    description='RViz visualization markers for GP7 robot box grab and place simulation',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'boxes = gp7_boxes.scripts.boxes:main',
        ],
    },
)
