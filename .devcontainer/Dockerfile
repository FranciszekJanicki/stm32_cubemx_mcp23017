FROM archlinux:base-devel

ARG TOOLS_PATH=/opt/stm32-tools

RUN pacman -Sy --noconfirm base-devel curl unzip jre-openjdk libusb usbutils arm-none-eabi-gcc arm-none-eabi-gdb cmake ninja git minicom stlink && \
    pacman -Scc --noconfirm && rm -rf /var/cache/pacman/pkg/*

RUN mkdir -p ${TOOLS_PATH}

COPY STM32CubeMX-latest-linux.zip /tmp/
COPY STM32CubeProgrammer-latest-linux.zip /tmp/
COPY STM32CubeFinder-latest-linux.zip /tmp/
COPY stlink-server_v2-0-1-0_linux_x86_64.zip /tmp/

RUN echo "Unzipping STM32CubeMX..." && \
    unzip /tmp/STM32CubeMX-latest-linux.zip -d ${TOOLS_PATH} && \
    rm /tmp/STM32CubeMX-latest-linux.zip && \
    chmod +x ${TOOLS_PATH}/STM32CubeMX/STM32CubeMX

RUN echo "Unzipping STM32CubeProgrammer..." && \
    unzip /tmp/STM32CubeProgrammer-latest-linux.zip -d ${TOOLS_PATH} && \
    rm /tmp/STM32CubeProgrammer-latest-linux.zip && \
    chmod +x ${TOOLS_PATH}/STM32CubeProgrammer/bin/STM32_Programmer_CLI

RUN echo "Unzipping STM32CubeFinder..." && \
    unzip /tmp/STM32CubeFinder-latest-linux.zip -d ${TOOLS_PATH} && \
    rm /tmp/STM32CubeFinder-latest-linux.zip && \
    chmod +x ${TOOLS_PATH}/STM32CubeFinder/STM32CubeFinder

RUN echo "Unzipping ST-LINK server..." && \
    unzip /tmp/stlink-server_v2-0-1-0_linux_x86_64.zip -d ${TOOLS_PATH} && \
    rm /tmp/stlink-server_v2-0-1-0_linux_x86_64.zip && \
    chmod +x ${TOOLS_PATH}/stlink-server/stlink-server

RUN ln -s ${TOOLS_PATH}/STM32CubeMX/STM32CubeMX /usr/local/bin/STM32CubeMX && \
    ln -s ${TOOLS_PATH}/STM32CubeProgrammer/bin/STM32_Programmer_CLI /usr/local/bin/STM32_Programmer_CLI && \
    ln -s ${TOOLS_PATH}/STM32CubeFinder/STM32CubeFinder /usr/local/bin/STM32CubeFinder && \
    ln -s ${TOOLS_PATH}/stlink-server/stlink-server /usr/local/bin/stlink-server

RUN echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="3748", MODE="0666", GROUP="plugdev"' > /etc/udev/rules.d/99-stlink.rules && \
    udevadm control --reload-rules && udevadm trigger

SHELL ["/bin/bash", "-c"]
