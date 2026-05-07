FROM espressif/idf:release-v5.0

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update -q && \
    apt-get install -yq \
        sudo \
        lsb-release \
        gosu \
        nano \
        usbutils \
        python3-pip && \
    rm -rf /var/lib/apt/lists/*

ARG TZ_ARG=UTC
ENV TZ=$TZ_ARG
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

ARG USER_ID=espidf
RUN useradd --create-home \
            --home-dir /home/$USER_ID \
            --shell /bin/bash \
            --user-group \
            --groups adm,sudo,dialout \
            $USER_ID && \
    echo "$USER_ID:$USER_ID" | chpasswd && \
    echo "$USER_ID ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

ARG USER_ID
USER $USER_ID

WORKDIR /code

# Pre-install Python deps used by idf.py component manager
RUN pip3 --no-cache-dir install idf-component-manager 2>/dev/null || true
