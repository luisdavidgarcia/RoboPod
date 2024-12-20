# RoboPod
RoboPod is a containerized ROS2-based project for the iCreate3 robot, built with C++ and managed using Podman. This project provides a lightweight and portable setup to easily develop, test, and deploy robotics applications in a containerized environment.

## Getting Started

It basically ties down to the wonderful documentation form iRobot

1. [iCreate3 Docs](https://iroboteducation.github.io/create3_docs/)
2. [Python iRobot Playground](https://python.irobot.com/)
3. [iRobot Github](https://github.com/iRobotEducation)

In the Github there are already docker images to run Create3 applications,
but there always room to expand such as adding deep learning libraries and
frameworks.

To run and create the Docker image you need to run the following commands:

```sh
podman build -t icreate3-dev -f .devcontainer/Dockerfile .
```

```sh
podman run -it --rm \
    --network=host \
    --env="DISPLAY=$DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume=".:/root/create3_ws/src/my_project" \
    icreate3-dev
```