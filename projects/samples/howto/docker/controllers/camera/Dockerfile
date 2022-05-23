ARG WEBOTS_DEFAULT_IMAGE
FROM $WEBOTS_DEFAULT_IMAGE
ARG MAKE
ENV WEBOTS_HOME /usr/local/webots
ENV LD_LIBRARY_PATH /usr/local/webots/lib/controller
ENV WEBOTS_STDOUT_REDIRECT 1
ENV WEBOTS_STDERR_REDIRECT 1

RUN mkdir -p /webots_project/controllers/camera
COPY camera.c /webots_project/controllers/camera/camera.c
COPY Makefile /webots_project/controllers/camera/Makefile

RUN if [ "$MAKE" = "1" ]; then \
    cd /webots_project/controllers/camera && \
    make ; \
    fi
