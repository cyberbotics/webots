ARG WEBOTS_DEFAULT_IMAGE
FROM $WEBOTS_DEFAULT_IMAGE
ARG MAKE
RUN mkdir -p /webots_project
COPY controllers /webots_project/controllers
RUN if [ "$MAKE" = "1" ]; then \
    cd /webots_project/controllers && \
    make ; \
    fi
