# Docker and ROS Melodic Install

Updated: May 11, 2020

Note: Python2 used in docker ROS image

## Docker Install

For Ubuntu the general steps to install are outlined here: https://docs.docker.com/engine/install/ubuntu/, but a quick rundown of steps are provided below.

Ensure Clean Docker Install by removing any old docker installs

```
$ sudo apt-get remove docker docker-engine docker.io containerd runc
```

Install a few prerequisites

```
$ sudo apt-get update

$ sudo apt-get install \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg-agent \
    software-properties-common
```

Add official GPG key

```
$ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
```

Get the fingerprint

```
$ sudo apt-key fingerprint 0EBFCD88
```

Ensure that it is the same as `9DC8 5822 9FC7 DD38 854A  E2D8 8D81 803C 0EBF CD88`.


Add the repository to download Docker (Stable)

```
$ sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"
```

Install docker from the repo by using apt

```
 $ sudo apt-get update
 $ sudo apt-get install docker-ce docker-ce-cli containerd.io
```

Check if the install is working

```
$ sudo docker run hello-world
```

### ROS Install

We are using ROS melodic:

```
$ sudo docker pull ros:melodic
```

If you want the "desktop-full" install 

```
$ sudo docker pull osrf/ros:melodic-desktop-full
```

To run the docker ROS container

```
$ sudo docker run -it --rm --name ros_main osrf/ros:melodic-desktop-full roscore
```

To get into the container and set it up

``` 
$ sudo docker exec -it ros_main bash
```

Once you are inside the container ensure you are root 

```
$ whoami
``` 

To finish setup run

```
# source /opt/ros/melodic/setup.bash
```

Now you should be able to work with ROS. 


Try checking the rostopics

```
# rostopics list
```

## Docker Details

#### Stopping containers

```
$ sudo docker stop <container_name>
```

#### Starting containers

```
$ sudo docker run -it <container_name>
```

Docker run options can make the container run in the background, connect to a network, link to ports and more. To learn more about the optios: https://docs.docker.com/engine/reference/run/


#### Checking containers running

```
$ sudo docker ps
```





