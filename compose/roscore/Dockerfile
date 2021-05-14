ARG PARENT 
FROM ${PARENT}

RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-rviz

COPY ./start.sh /start.sh
RUN chmod +x /start.sh

CMD [ "/start.sh" ]