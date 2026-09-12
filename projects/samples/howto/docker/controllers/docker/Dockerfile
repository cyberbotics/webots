ARG WEBOTS_DEFAULT_IMAGE
FROM $WEBOTS_DEFAULT_IMAGE
ARG MAKE
ENV WEBOTS_HOME /usr/local/webots
ENV LD_LIBRARY_PATH /usr/local/webots/lib/controller
ENV WEBOTS_STDOUT_REDIRECT 1
ENV WEBOTS_STDERR_REDIRECT 1

RUN mkdir -p /webots_project/controllers/docker
COPY docker.c /webots_project/controllers/docker/docker.c
COPY Makefile /webots_project/controllers/docker/Makefile

RUN if [ "$MAKE" = "1" ]; then \
    cd /webots_project/controllers/docker && \
    make ; \
    fi
