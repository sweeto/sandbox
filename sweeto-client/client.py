import paho.mqtt.client as mqtt
import neato
import json
import time
import platform
import argparse

payload = """
{"cmd":"SetMotor",
"args":{"LWheelDist":300,
"RWheelDist":300,
"Speed":100,
"Accel": 100,
"RPM": 0
}
}
"""

class SweetoClient:
    def __init__(self, server_address, server_port, username, password, neato_serial_port, update_interval):
        self.update_interval=update_interval
        self.client = mqtt.Client()
        self.client.username_pw_set(username, password)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        if neato_serial_port is None:
            self.neato = neato.NeatoDummy()
        else:
            self.neato = neato.Neato(neato_serial_port)
        self.client.connect(server_address, server_port, 60)

    def run(self):
        
        self.client.loop_start()
        
        while True:
            status = json.dumps(self.neato.GetStatus())
            self.client.publish("neato/status", status)
            time.sleep(self.update_interval)


    def debug(self, txt, dest="neato/status"):
        msg = json.dumps(dict(debug=txt))
        self.client.publish(dest, msg)
            
    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code "+str(rc))
        self.debug("Hello from %s" % platform.uname()[1])
        print("Posted hello msg")
        client.subscribe("neato/commands")


    def on_message(self, client, userdata, msg):
        try:
            obj = json.loads(msg.payload)
        except ValueError:
            print("Could not decode json")
            return
        cmd = obj.get("cmd")
        args = obj.get("args", {})
        func = getattr(self.neato, cmd)
        if func:
            func(**args)
        else:
            print "Could not run %s" % cmd
    


def parseargs():
    parser = argparse.ArgumentParser()
    parser.add_argument("--server-address",
                        help="MQTT Broker address")
    parser.add_argument("--server-port", help="MQTT Broker port")
    parser.add_argument("--username", help="MQTT Broker username")
    parser.add_argument("--password", help="MQTT Broker password")
    parser.add_argument("--update-interval", type=int, default=5, help="Neato status update interval")
    parser.add_argument("--neato-serial-port", help="Neato Serial Port device")
    args = parser.parse_args()
    return args
        
if __name__ == "__main__":
    args = parseargs()
    client = SweetoClient(args.server_address, args.server_port,
                          args.username, args.password, args.neato_serial_port, args.update_interval)
    client.run()
    
