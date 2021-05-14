ARG PARENT 
FROM ${PARENT}

RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-rviz

ENV F1TENTH_EXEC_DIR=/f1tenth_gym/build/

COPY ./start.sh /start.sh
RUN chmod +x /start.sh

CMD [ "/start.sh" ]