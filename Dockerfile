# Use ros2 as base image.
FROM osrf/ros:rolling-desktop

# Install necessary packages
RUN apt-get update && \
    apt-get install -y python3 python3-pip python3-dev && \
    apt-get install -y libgl1-mesa-glx && \
    apt-get install -y wget && \
    apt-get install -y vim && \
    apt     install -y openssh-server && \
    apt     install -y tmux  && \
    rm -rf /var/lib/apt/lists/*

# Upgrade pip
RUN pip3 install --upgrade pip

# Some ROS packages
RUN apt-get update \
    && apt-get install -y python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Requirements for ROS2 YASMIN
RUN pip3 install flask waitress expiringdict


# Install JupyterLab
# RUN pip3 install jupyterlab pytorch-lightning wandb matplotlib pandas numpy
# RUN pip3 install ipywidgets

# Expose the Jupyter notebook port
EXPOSE 8885

# Some packages for ROS2:
# RUN sudo apt-get install qt5-default

# Set frontend mode to noninteractive
ENV DEBIAN_FRONTEND=noninteractive
# Install zsh
RUN apt-get update && apt-get install -y zsh
# Set zsh as the default shell
# RUN chsh -s $(which zsh)
# Zsh
# Default powerline10k theme, no plugins installed
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.5/zsh-in-docker.sh)" -t robbyrussell -p fzf 

# ROS2 setup
ENV ROS2_WS=/home/ros2_ws
# build repo
# RUN cd ${ROS2_WS} \
# 	&& colcon build \
#  	&& source ${ROS2_WS}/install/setup.zsh

# Clean up list of apt packages.
RUN rm -rf /var/lib/apt/lists/*


ENTRYPOINT ["/ros_entrypoint.sh"]


# Set the default command to run when starting the container
# CMD ["jupyter", "lab", "--ip=0.0.0.0", "--port=8885", "--allow-root"]
CMD [ "zsh" ]
