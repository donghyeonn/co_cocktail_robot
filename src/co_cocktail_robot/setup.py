from setuptools import find_packages, setup
import glob

package_name = 'co_cocktail_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        ('share/' + package_name + '/resource', glob.glob('resource/*')),
        ('share/' + package_name + '/resource', ['resource/.env']), # .env 파일 명시적으로 추가
        ('share/' + package_name, ['co_cocktail_robot/locations/pose.yaml']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='ljhwan1997@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = co_cocktail_robot.main:main',
            'task_planning = co_cocktail_robot.task_planning.get_keyword:main',
            'detection = co_cocktail_robot.object_detection.detection:main',
        ],
    },
)
