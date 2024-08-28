all: build run clean

build:
	docker build -t ros-melodic-zsh:latest .

run:
	xhost +local:root
	-docker run -it \
		--privileged \
		--env="DISPLAY" \
		-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
		-e XDG_RUNTIME_DIR=/tmp \
		-e QT_X11_NO_MITSHM=1 \
		--net=host \
		--name ros-melodic-zsh \
		--ulimit nofile=1024:524288 \
		--mount type=bind,source=$(CURDIR)/assignment_1/,target=/root/irob_ws/src/ \
		ros-melodic-zsh:latest
	xhost -local:root

clean:
	docker container rm ros-melodic-zsh

attach:
	-docker exec -it ros-melodic-zsh /bin/zsh