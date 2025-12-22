from setuptools import setup, find_packages

package_name = 'gp7_boxes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='savindu',
    maintainer_email='savindu@example.com',
    description='RViz markers for GP7 boxes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'boxes = gp7_boxes.scripts.boxes:main',
        ],
    },
)
