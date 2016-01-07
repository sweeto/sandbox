import paho.mqtt.client as mqtt
import neato
import json
import time
import platform
import argparse
import atexit

payload = """
{"cmd":"SafeMotor",
"args":{"LWheelDist":300,
"RWheelDist":300,
"Speed":100,
"Accel": 100,
"RPM": 0
}
}
"""


class SweetoClient:
    def __init__(self, server_address, server_port, username, password, neato_serial_port, update_interval, dump_file, replay_file):
        self.update_interval=update_interval
        self.client = mqtt.Client()
        self.client.username_pw_set(username, password)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.dump_file = dump_file
        if neato_serial_port is None:
            self.neato = neato.NeatoDummy(replay_file)
        else:
            self.neato = neato.Neato(neato_serial_port, False)
            #self.neato.SetLDSRotation(True)
        self.client.connect(server_address, server_port, 60)

        
    def run(self):
        self.client.loop_start()
        self._running = True
        history = []
        max_hist = 120

        while self._running:
            self.update()
            if self.dump_file and len(history) <= max_hist:
                history.append(status)
                if len(history) == max_hist:
                    print "Dumping status to %s" % self.dump_file
                    with open(self.dump_file, "w") as fd:
                        json.dump(history, fd, indent=2)
                
            time.sleep(self.update_interval)

    def update(self, add_status=None):
        try:
            status = self.neato._UpdateStatus()
        except Exception, e:
            print "Failed to get neato status: %s" % e
            return
        if not status:
            print "No status..."
            return
        self.client.publish("neato/status", json.dumps(status))
        ts, state, err = status.get("updated"), status.get("state"), status.get("error")
        #print "[%s] Posted update: State:'%s' Error: '%s'" % (time.strftime("%H:%M:%S", time.localtime(ts)),
        #                                                      state, err)
        return status

    def stop(self):
        print "Exiting..."
        self._running = False
        self.neato.TestMode(False)
        

    def debug(self, txt, dest="neato/debug"):
        msg = json.dumps(dict(debug=txt))
        self.client.publish(dest, msg)
            
    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code "+str(rc))
        self.debug("Hello from %s" % platform.uname()[1])
        print("Posted hello msg")
        client.subscribe("neato/commands")


    def on_message(self, client, userdata, msg):
        print "Msg payload: [%s]" % msg.payload
        try:
            obj = json.loads(msg.payload)
        except ValueError:
            print("Could not decode json")
            return
        cmd = obj.get("cmd")
        if cmd == "Ping":
            return self.update()
        args = obj.get("args", [])
        kwargs = obj.get("kwargs", {})
        for key, val in kwargs.items():
            if type(val) is unicode:
                kwargs[key] = str(val)
        print "Cmd=%s args=%s kwargs=%s" %(cmd, args, kwargs)
        if hasattr(self.neato, cmd):
            func = getattr(self.neato, cmd)
            try:
                val = func(*args, **kwargs)
                if val:
                    print "Returned: ", val
                    self.update()
            except Exception, e:
                print "Failed to run '%s() args=%s kwargs=%s: %s'" % (cmd, args, kwargs, e)
        else:
            print "Could not run %s" % cmd
    


def parseargs():
    parser = argparse.ArgumentParser()
    parser.add_argument("--server-address",
                        help="MQTT Broker address")
    parser.add_argument("--server-port", help="MQTT Broker port")
    parser.add_argument("--username", help="MQTT Broker username")
    parser.add_argument("--password", help="MQTT Broker password")
    parser.add_argument("--update-interval", type=int, default=2, help="Neato status update interval")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--neato-serial-port", help="Neato Serial Port device")
    group.add_argument("--replay-data", help="Replay data dump file")
    parser.add_argument("--dump-data", help="Data dump file")

    args = parser.parse_args()
    return args
        
if __name__ == "__main__":
    args = parseargs()
    client = SweetoClient(args.server_address, args.server_port,
                          args.username, args.password, args.neato_serial_port, args.update_interval, args.dump_data,
                          args.replay_data)
    atexit.register(client.stop)
    client.run()
    
