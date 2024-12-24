# RoboPod
RoboPod is a containerized ROS2-based project for the iCreate3 robot, built with C++ and managed using Podman. This project provides a lightweight and portable setup to easily develop, test, and deploy robotics applications in a containerized environment.

## Getting Started

It basically ties down to the wonderful documentation form iRobot

1. [iCreate3 Docs](https://iroboteducation.github.io/create3_docs/)
2. [Python iRobot Playground](https://python.irobot.com/)
3. [iRobot Github](https://github.com/iRobotEducation)


## Connecting to WiFi

To work with ROS2 you will want to connect to your own wifi network, so please
first do that as in phase 3 instructions said here: [Connect to Wifi](https://edu.irobot.com/create3-setup)

However, for fun you can also setup your robot's own namespace which is important
if going to use multiple robots: [Get a namespace](https://iroboteducation.github.io/create3_docs/setup/provision/)

You can still access the webserver of your robot, but now it will have the IP
address assigned to it from your router, so look for that.

## Changing the Middleware for ROS2 Humble

Now you must get the middleware configure via the webserver. The complete 
instructions are here: [Middleware install](https://iroboteducation.github.io/create3_docs/setup/xml-config/)

Then it will tell you to restart the application with the button on that webpage,
so follow those instructions.


## Docker Setup with ROS2

In the Github there are already docker images to run Create3 applications,
but there always room to expand such as adding deep learning libraries and
frameworks.

To run and create the Docker image you need to run the following commands:

```sh
docker build -t icreate3-dev -f .devcontainer/Dockerfile .
```

```sh
docker run -it --rm \
    --network=host \
    --env="DISPLAY=$DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd):/root/create3_ws/src/my_project" \
    icreate3-dev
```

## Interacting with ROS2 (Pubs, Subs, Topics)

I had some experience working with Pubs, Subs, and topics before from my prior
internship with Schilling orbotics, so it was nice getting to interact with it 
like before but this time also getting like the real ROS2 experience with my
own robot.

To see the topics you can work with just use the following command:

```sh
ros2 topic list -t
```

## Interacting with Gazebo

In your docker container please install Gazebo Harmonic with as instructed here:

[Gazebo Haramonic Install](https://gazebosim.org/docs/harmonic/install_ubuntu/)

## Examples Directory

If you go to `src/examples` you will see four examples of getting with ROS2
with the iCreate3, so the scripts entail:

1. `sub_ir.py`: Lets us create a subscriber for infared topic
2. `pub_lightring.py`: Lets us create a publish for lightring topic
3. `actionclient_rotate.py`: Lets us us create an action client for rotating
4. `audio_bump.py`: Lets us see how to create a ROS2 package


However here is an example of how to create packages in ROS2:

Assuming you installed ROS2 Humble and have your docker container running:

Nex you will need to run to create the package:

```sh
source install/setup.bash
cd src
ros2 pkg create --build-type ament_python package_example
```

This is want created the package and it basically is configured for python, but
next we must build again with the new package using:

```sh
cd ..
colcon build --packages-select package_example
. install/local_setup.bash
```

In all packages you create you can get access to a `package.xml` file that you
can edit and incoprate basically details abotu the project like description,
name, license, and author. You should repeat your changes for the `setup.py`
script that was installed in that same location.

In that same `setup.py` we can add a discovery script like here:

```python
entry_points=[
    'console_scripts': ['pub_sub = package_example.audio_bump:main'],
]
```

For any edits in your package you should always run:

```sh
colcon built --packages-select package_example
. install/local_setup.bash
```

So you if you copy the `audio_bump.py` file you will need to rerun them. 
Speaking of copying it try it then run those commands:

```sh
cd ~/create3_ws
cp src/my_project/src/examples/audio_bump.py src/package_example/package_example
colcon built --packages-select package_example
. install/local_setup.bash
```

To run the scripts you edited in entry point just build as shown in the commands
above this line. You can then run this package by doing:

```sh
ros2 run create3_package pub_sub
```
