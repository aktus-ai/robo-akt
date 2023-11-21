from setuptools import find_packages, setup

package_name = 'petridish'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maryam',
    maintainer_email='bandari.m@gmail.com',
    description='A petridish demo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'petri_node = petridish.petri_node:main'
        ],
    },
)
