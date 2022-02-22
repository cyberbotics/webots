ARG FROM_IMAGE
FROM $FROM_IMAGE
ARG MAKE
RUN mkdir -p webots-project
COPY . webots-project
RUN if [ $MAKE = "1" ] ; then cd webots-project && make ; fi
