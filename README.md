# caric_docker

Make sure you have docker installed: [instruction](https://docs.docker.com/engine/install/ubuntu/).

Clone this package and build docker:
```bash
git clone https://github.com/caomuqing/caric_docker
cd caric_docker
docker compose build
```
If the build succeeds, run the following command to run the docker container and build the ros workspace:

```bash
docker compose up
```
After building the ros package, do not close the terminal; open a new terminal. Run the following command to enter the container:

```bash
xhost + #allowing allow docker access to screen
docker exec -it ros_caric_container_1 bash # open a container terminal
```
To run the baseline method:
```bash
source devel/setup.bash
roslaunch caric_baseline run.launch scenario:=crane
```
