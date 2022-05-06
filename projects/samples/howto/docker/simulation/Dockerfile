ARG WEBOTS_DEFAULT_IMAGE
FROM $WEBOTS_DEFAULT_IMAGE
ARG MAKE
RUN mkdir -p /webots_project
COPY worlds /webots_project/worlds
RUN if [ "$MAKE" = "1" ]; then \
    cd /webots_project && \
    make ; \
    fi
