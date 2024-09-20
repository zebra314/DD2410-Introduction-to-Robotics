# Assignment Folders
ASSIGNMENT_1 := assignment_1_ros_basic
ASSIGNMENT_2 := assignment_2_inverse_kinematic
ASSIGNMENT_3 := assignment_3_planning
ASSIGNMENT_4 := assignment_4_mapping

# Docker image names
ROS_IMAGE := ros-noetic-zsh:latest
PYTHON_IMAGE := python-noetic-zsh:latest

# Assignment rules
assignment1:
	$(MAKE) ASSIGNMENT_FOLDER=$(ASSIGNMENT_1) ros

assignment2:
	$(MAKE) ASSIGNMENT_FOLDER=$(ASSIGNMENT_2) ros

assignment3:
	$(MAKE) ASSIGNMENT_FOLDER=$(ASSIGNMENT_3) python

assignment4:
	$(MAKE) ASSIGNMENT_FOLDER=$(ASSIGNMENT_4) ros

ros: build-ros run-ros clean-ros

python: build-python run-python clean-python

build-ros:
	docker build -f Dockerfile.ros -t $(ROS_IMAGE) .

build-python:
	docker build -f Dockerfile.python -t $(PYTHON_IMAGE) .

run-ros:
	xhost +local:root
	-docker run -it \
		--privileged \
		--env="DISPLAY" \
		-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
		-e XDG_RUNTIME_DIR=/tmp \
		-e QT_X11_NO_MITSHM=1 \
		--net=host \
		--name ros-noetic-zsh \
		--ulimit nofile=1024:524288 \
		--mount type=bind,source=$(CURDIR)/$(ASSIGNMENT_FOLDER)/,target=/root/irob_ws/src/ \
		$(ROS_IMAGE)
	xhost -local:root

run-python:
	xhost +local:root
	-docker run -it \
		--privileged \
		--env="DISPLAY" \
		-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
		-e XDG_RUNTIME_DIR=/tmp \
		-e QT_X11_NO_MITSHM=1 \
		--net=host \
		--name python-noetic-zsh \
		--ulimit nofile=1024:524288 \
		--mount type=bind,source=$(CURDIR)/$(ASSIGNMENT_FOLDER),target=/root/$(ASSIGNMENT_FOLDER) \
		--workdir /root/$(ASSIGNMENT_FOLDER) \
		$(PYTHON_IMAGE)
	xhost -local:root

clean-ros:
	-docker container rm ros-noetic-zsh

clean-python:
	-docker container rm python-noetic-zsh

attach-ros:
	-docker exec -it ros-noetic-zsh /bin/zsh

attach-python:
	-docker exec -it python-noetic-zsh /bin/zsh