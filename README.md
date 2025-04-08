# Diff_Drive_control
This project investigates three distinct control methodologies for differential drive robots using ROS (Robot Operating System). It contains various components such as robot descriptions, launch files, and simulation configurations to simulate and control the robot in a ROS environment.


## Project Structure

Below is an overview of the key files and directories in this repository:

### `catkin_ws/`
This is the main workspace directory for your ROS packages. It contains the source code, build files, and development setup for the project.

- `src/`: Contains the source code for the project.

To build:

```bash
cd catkin_ws
catkin_make
source devel/setup.bash
```

### `catkin_ws/src/diff_drive/`
Contains the main source code related to the differential drive robot.

- `CMakeLists.txt`: Specifies dependencies: rospy, roscpp, std_msgs, urdf
- `diff_drive_description/`: This folder contains the .xacro file describing the robot, including the gazebo plugin for the  simulation.
- `launch/`: Contains launch files to start the robot simulation and bring up the robot’s components.
    - `gazebo.launch`:  launches Gazebo and spawns the robot
    - `controller.launch`: launches Gazebo, the robot, controllers, and loads controller parameters
	 ⚠️ Edit this file to switch between controllers
- `src/`: This folders contains three different python scripts for: I/O Linearization, Non Linear Control, Polar Posture Regulation
- `immagini_report/`: Plots of simulation results using default controller parameters

### `docker_ws/`
This folder contains the dockerfile to build the docker image.

## Running the Project with Docker

To make it easy to run the project in any environment, we provide Docker support. Follow these steps to get started:

### 1. Prerequisites
Make sure Docker is installed on your machine. You can download and install Docker from the official website:  
[https://www.docker.com/get-started](https://www.docker.com/get-started).

### 2. Clone the repository
Clone this repository to your local machine:

```bash
git clone https://github.com/miktm-99/Diff_Drive_control.git
cd Diff_Drive_control
```


### 3. Build the docker image

From the `docker_ws` directory, build the Docker image:

```bash
cd docker_ws
docker build -t ros:studenti .
```

### 4. Run the container
Run the container with GUI and X11 forwarding support:

```bash
cd ..
xhost +local:root
docker run --rm -it \
  --privileged \
  --net host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  --ipc=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/.Xauthority:/root/.Xauthority:rw \
  -v $(pwd):/root/ws/ \
  -w /root/ws \
  ros:studenti \
  bash
```

### 5. Launch the project
Once inside the container, navigate to the ROS workspace and build it:

```bash
cd catkin_ws
catkin_make
source devel/setup.bash
roslaunch diff_drive gazebo.launch
```

To run the python script of the controller, write in another terminal:
```bash
docker ps
docker exec -it container_name bash

rosrun diff_drive non_linear_control.py
```

### 6. Exit docker container 
Once you are done with the simulation, you can exit the Docker container by typing:

```bash
exit
```


