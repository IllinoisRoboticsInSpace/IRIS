FROM osrf/ros:jazzy-desktop

COPY ./install_docker_deps.sh /tmp/install_env_deps.sh

RUN chmod +x /tmp/install_env_deps.sh

RUN /tmp/install_env_deps.sh > /log/iris.log 2>&1

RUN rm /tmp/install_env_deps.sh

# RUN rmdir /tmp

ENTRYPOINT ["bash"]
