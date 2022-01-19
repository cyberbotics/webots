FROM cyberbotics/webots:R2022a-ubuntu20.04
RUN mkdir -p /home/controllers/e-puck
COPY . /home/controllers/e-puck/
WORKDIR /home/controllers/e-puck
RUN rm -rf build
RUN make release
ENV LD_LIBRARY_PATH=/usr/local/webots/lib/controller
ENTRYPOINT ["/home/controllers/e-puck/e-puck"]
