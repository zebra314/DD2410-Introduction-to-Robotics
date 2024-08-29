FROM ros:melodic

# Specify terminal color
ENV TERM xterm-256color

# Install nvim
RUN apt-get update && apt-get install -y neovim

# Install zsh
RUN apt-get update && apt-get install -y zsh
RUN chsh -s $(which zsh)

# Install oh-my-zsh
RUN apt-get update && apt-get install -y curl
RUN apt-get update && apt-get install -y git
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

# Install p10k
RUN git clone --depth=1 https://github.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k
RUN sed -i 's/ZSH_THEME="robbyrussell"/ZSH_THEME="powerlevel10k\/powerlevel10k"/g' ~/.zshrc
COPY dotfiles/.p10k.zsh /root/.p10k.zsh
COPY dotfiles/.zshrc /root/.zshrc

# Install tmux
RUN apt-get update && apt-get install -y tmux 

# Install multiple zsh plugins
# 1. zsh-autosuggestions
# 2. zsh-syntax-highlighting
RUN git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
RUN git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting
RUN sed -i 's/plugins=(git)/plugins=(git zsh-autosuggestions zsh-syntax-highlighting)/g' ~/.zshrc

# Install libeigen3-dev
RUN apt-get update && apt-get install -y libeigen3-dev

# Install qt5-default
RUN apt-get update && apt-get install -y qt5-default

# Allow to run GUI
RUN apt-get update && apt-get install -y libpci-dev
RUN apt-get update && apt-get install -y x11-apps
RUN apt-get update && apt-get install -y qtwayland5
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -yqq xserver-xorg
RUN apt-get update && apt-get install -y xwayland

# Install ROS packages
RUN apt install -y build-essential python3-pip python-pip python-catkin-tools
RUN apt install -y ros-melodic-turtlebot3 ros-melodic-turtlebot3-msgs ros-melodic-turtlebot3-simulations 
RUN apt install -y ros-melodic-navigation ros-melodic-nav-msgs
RUN apt install -y libspatialindex-dev libqt4-dev
RUN apt install -y ros-melodic-rqt ros-melodic-rqt-common-plugins ros-melodic-turtlesim
RUN apt install -y ros-melodic-tf-conversions ros-melodic-geometry2
RUN apt install -y ros-melodic-turtle-tf2 ros-melodic-tf2-tools ros-melodic-tf
RUN apt-get install -y ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
RUN apt-get install -y ros-melodic-rviz
RUN apt-get install -y ros-melodic-hector-slam

# Set workdir
WORKDIR /root/irob_ws

# Entry point
COPY scripts/entrypoint.sh /root/scripts/entrypoint.sh
ENTRYPOINT ["/root/scripts/entrypoint.sh"]
CMD ["zsh"]