from setuptools import setup

package_name = 'polaris_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Inclui os arquivos de lançamento na instalação
        ('share/' + package_name + '/launch', ['launch/demo.launch.xml']),
        ('share/' + package_name + '/config', ['config/demo_rviz.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thales',
    maintainer_email='thalesasoares02@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Isso cria o executável 'robot_simulator' a partir do seu script
            'robot_simulator = polaris_control.scripts.robot_simulator:main',
        ],
    },
)