# Light Control Emulated

## How to run?

To start the containers run inside this folder:

```sh
docker compose up -d
```

To stop the containers run inside this folder:

```sh
docker compose stop
```

### NodeRed Setup

Instrall the following node-red plugins after configuring it:

- node-red-dashboard
- node-red-contrib-postgresql

Then configure the postgresql connection to the timescale db using the params in [docker-compose.yaml](./docker-compose.yaml).

### Run the Simulation

You can find the ESP32 simulated circuit in the folder [esp32-wokwi-emulated](./esp32-wokwi-emulated/) and use the wokwi extension in VS Code or load the code to [wokwi.com](https://wokwi.com).
