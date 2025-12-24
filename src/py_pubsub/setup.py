from setuptools import find_packages, setup

package_name = 'py_pubsub'

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
    maintainer='rover',
    maintainer_email='rover@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'esp32rst = py_pubsub.esp32_rst:main' , 
            'esp32rst1 = py_pubsub.esp32_rst1:main' , 
            'campub = py_pubsub.camfeed:main',
            'camsub = py_pubsub.camsub:main',
            'camfinal = py_pubsub.camfeed_final:main',

        ],
    },
)
