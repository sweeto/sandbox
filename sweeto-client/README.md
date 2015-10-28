# Client

## Installation

```sh
pip install -r requirements.pip
```

## Usage

```
--server-address SERVER_ADDRESS
    MQTT Broker address
--server-port SERVER_PORT
    MQTT Broker port
--username USERNAME
    MQTT Broker username
--password PASSWORD
    MQTT Broker password
--update-interval UPDATE_INTERVAL
    Neato status update interval in seconds
--neato-serial-port NEATO_SERIAL_PORT
    Neato Serial Port device (optional). If omitted, the client will start a
    mock Neato device, sending mock status messages.
```
