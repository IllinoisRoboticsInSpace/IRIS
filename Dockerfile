FROM ubuntu:20.04

COPY ./install_docker_deps.sh /tmp/install_env_deps.sh

RUN chmod +x /tmp/install_env_deps.sh

RUN /tmp/install_env_deps.sh 

RUN rm /tmp/install_env_deps.sh

RUN rmdir /tmp

ENTRYPOINT bash
