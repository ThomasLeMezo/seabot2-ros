from setuptools import setup

package_name = 'seabot2_recorder'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Thomas Le Mézo',
    maintainer_email='thomas.le_mezo@ensta-bretagne.org',
    description='Package to launch de ros2 bag record and restart at each new mission',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'seabot2_recorder = seabot2_recorder.seabot2_recorder:main'
        ],
    },
)
