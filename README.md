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

