ARG FROM_IMAGE, MAKE
FROM $FROM_IMAGE
RUN mkdir -p webots-project
COPY . webots-project
RUN if [ $MAKE = "1" ] ; then cd webots-project && make ; fi