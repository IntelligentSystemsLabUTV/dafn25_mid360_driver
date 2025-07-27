ARG VERSION=20.04
FROM ubuntu:$VERSION

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update &&\
    apt-get install -y --no-install-recommends \
    build-essential\
    git &&\ 
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*/apt/lists/*

ENV DEBIAN_FRONTEND=dialog
LABEL maintainer.name="Alessio Vinci"

CMD ["bash"]
