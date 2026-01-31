# Base image with ROS 2 Jazzy Desktop
FROM osrf/ros:jazzy-desktop-full

# Environment setup
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Install basic build dependencies and tools
RUN apt-get update && apt-get install -y \
    git \
    wget \
    curl \
    vim \
    nano \
    sudo \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    bash-completion \
    ros-jazzy-rmw-implementation \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user 'rosdev' matching your devcontainer config
ARG USERNAME=rosdev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Check if user with UID 1000 exists, if so modify it, otherwise create new user
RUN if id -u $USER_UID > /dev/null 2>&1; then \
        EXISTING_USER=$(id -un $USER_UID); \
        usermod -l $USERNAME -d /home/$USERNAME -m $EXISTING_USER || true; \
        groupmod -n $USERNAME $(id -gn $USER_UID) 2>/dev/null || true; \
    else \
        if getent group $USER_GID > /dev/null 2>&1; then \
            useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash; \
        else \
            groupadd --gid $USER_GID $USERNAME; \
            useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash; \
        fi \
    fi && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Create CycloneDDS config file
RUN mkdir -p /etc/cyclonedds && \
    echo '<CycloneDDS>\n\
  <Discovery>\n\
    <ParticipantIndex>auto</ParticipantIndex>\n\
    <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>\n\
  </Discovery>\n\
</CycloneDDS>' > /etc/cyclonedds.xml

# Set CYCLONEDDS_URI
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml

# Initialize rosdep
RUN rosdep update

# Set up workspace directory
RUN mkdir -p /workspaces/warehouse_robot && \
    chown -R $USERNAME:$USERNAME /workspaces

# Switch to non-root user
USER $USERNAME

# Set up bash environment for rosdev user
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "if [ -f /workspaces/warehouse_robot/install/setup.bash ]; then" >> /home/$USERNAME/.bashrc && \
    echo "  source /workspaces/warehouse_robot/install/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "fi" >> /home/$USERNAME/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /home/$USERNAME/.bashrc

# Copy entrypoint script from .devcontainer directory
COPY .devcontainer/entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh

# Set working directory
WORKDIR /workspaces/warehouse_robot

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["/bin/bash"]