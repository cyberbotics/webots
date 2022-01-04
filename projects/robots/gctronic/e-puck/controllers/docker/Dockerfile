FROM cyberbotics/webots:R2022a-ubuntu20.04
RUN apt-get update && apt-get install -yq subversion
WORKDIR /home/
RUN mkdir controllers
WORKDIR /home/controllers
RUN svn export https://github.com/cyberbotics/webots/trunk/projects/robots/gctronic/e-puck/controllers/e-puck
WORKDIR /home/controllers/e-puck
RUN make
ENV LD_LIBRARY_PATH=/usr/local/webots/lib/controller
ENTRYPOINT ["/home/controllers/e-puck/e-puck"]
# ENTRYPOINT["python", "e-puck.py"]
