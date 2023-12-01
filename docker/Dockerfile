
FROM leimao/boost:1.80.0

SHELL ["/bin/bash", "-c"]

WORKDIR /rpi

COPY . .

RUN /usr/bin/g++ -fdiagnostics-color=always -g -I/rpi/include -I/rpi/src /rpi/src/main.cpp /rpi/src/Parser.cpp -o /rpi/src/main -pthread

ENTRYPOINT [ "/rpi/src/main" ]~