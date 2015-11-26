import serial
import sys
import time
import os
import json
import functools
import threading, thread
import math

def wake_neato():
    #dev = "/sys/kernel/debug/gpio_debug/gpio15/"
    dev = "/sys/kernel/debug/gpio_debug/gpio165/"
    if not os.path.isdir(dev):
        print "No gpio here"
        return
    with open(dev + "current_value", "w") as fd:
        fd.write("low")
    with open(dev + "current_direction", "w") as fd:
        fd.write("out")
    time.sleep(1)
    with open(dev + "current_direction", "w") as fd:
        fd.write("in")
    print "Pulled gpio down"
    time.sleep(10)


    
class Neato(object):
    def __init__(self, port, update_loop=True):
        self._debug_uart = False
        self._uart_connected = False
        self.lock = threading.RLock()
        self.status = {}
        self.testmode = False
        for i in range(2):
            try:
                self.port = serial.Serial(port, timeout=0.1)
                self._uart_connected = True
            except:
                print "Could not open serial port %s" % port
                self.port = sys.stderr
                print "Trying to wake neato"
                wake_neato()

        if update_loop:
            thread.start_new_thread(self.update_status_loop, ())
        else:
            # Make sure we initialize the status stuff
            self._UpdateStatus()

    def update_status_loop(self):
        loop_time=1
        print "Starting update loop"
        while True:
            start = time.time()
            self._UpdateStatus()
            wait = 1- (time.time() - start)
            wait = max(0,wait)
            #print "Waiting %s" % wait
            time.sleep(wait)

    def _read(self):
        eof=chr(26)
        txt = ""
        while True:
            bytes_to_read = self.port.inWaiting() or 1
            data = self.port.read(bytes_to_read)
            #print "Read %d bytes - got [%s]" % (bytes_to_read, data)
            if data:
                txt = txt + data
                if data[-1] == eof:
                    #print "Got eof"
                    return txt
                else:
                    pass
                    #print "Get more data"
            #else:
                #print "No data - timeout"
                #return txt

    def do_command(self, cmd, requires_testmode=False):
        with self.lock:
            if requires_testmode and not self.testmode:
                print "Setting testmode"
                self.TestMode(True)

            if self._debug_uart:
                print "UART => %s" % cmd
            self.port.write(cmd +"\n")

            response = self._read()
            response = response.split("\r\n")
            if self._debug_uart:
                for i in response:
                    print "UART <= %s" % i

            if response[0] != cmd:
                print "Response does not match cmd: [%s] != [%s] " %(response[0], cmd)
            if response[1] == 'TestMode must be on to use this command.':
                self.testmode = False
                return self.do_command(cmd, True)
            return response[1:-1]
        
    def GetErr(self):
        return self.do_command("GetErr")

    def GetCharger(self):
        response = self.do_command("GetCharger")
        response = [a.strip().split(",") for a in response[1:]]
        response = [(a, eval(b)) for (a, b) in response]
        return dict(response)

    def GetDigitalSensors(self):
        response = self.do_command("GetDigitalSensors")
        response = [a.strip().split(",") for a in response[1:]]
        response = [(a, int(b)) for (a, b) in response]
        return dict(response)

    def GetAnalogSensors(self):
        response = self.do_command("GetAnalogSensors")
        response = [a.strip().split(",") for a in response[1:]]
        formatted = []
        for line in response:
            try:
                name, unit, value, _ = line
                value = int(value)
                formatted.append((name, value))
            except:
                print "Could not parse %s" % line
        return dict(formatted)

    def Turn(self, deg):
        wheel_dist = 244
        rad = -deg * math.pi / 180
        lw = wheel_dist / 2 * rad
        rw = -lw
        self._SetMotor(lw, rw, 100)

    def GetAccel(self):
        response = self.do_command("GetAccel")
        response = [a.strip().split(",") for a in response[1:]]
        response = [(a, float(b)) for (a, b) in response]
        return dict(response)

    def GetMotors(self):
        response = self.do_command("GetMotors")
        response = [a.strip().split(",") for a in response[1:]]
        response = [(a, int(b)) for (a, b) in response]
        return dict(response)

    
    def SetLDSRotation(self, value):
        cmd = "SetLDSRotation %s" % ("On" if value else "Off")
        return self.do_command(cmd, True)
    
    def GetLDSScan(self):
        lines = self.do_command("GetLDSScan")
        values = []
        expected_line_count=362
        if len(lines) != expected_line_count:
            print "Expected %s lines, got %s" % (expected_line_count, len(lines))
            return []
        
        for line in lines[1:-1]:
            try:
                deg, dist, intensity, errorcode = line.split(",")
                deg = int(deg)
                dist = int(dist)
                intensity = int(intensity)
                errorcode = int(errorcode)
                values.append([deg, dist, intensity, errorcode])
            except:
                print "Could not parse... ", line
        return values


    def TestMode(self, value):
        self.testmode=value
        txt = "TestMode %s" % ("On" if value else "Off")
        self.do_command(txt)


    def Clean(self, arg=None):
        valid_args=[None, "House", "Spot", "Stop"]
        if arg not in valid_args:
            raise Exception("Received unexpected argument: %s, expected one of %s" % (arg, valid_args))
        self.TestMode(False)
        cmd = "Clean %s" % (arg or "")
        return self.do_command(cmd)

    def PlaySound(self, sound):
        self.do_command("PlaySound %d" % sound)

    def SetMotor(self, *args, **kwargs):
        return self.SmartDrive(*args, **kwargs)

    def _SetMotor(self, LWheelDist = 0, RWheelDist = 0, Speed = 0, Accel = 0, RPM = 0):
        txt = "SetMotor LWheelDist %d RWheelDist %d Speed %d Accel %d RPM %d" % (LWheelDist, RWheelDist, Speed, Accel, RPM)
        response = self.do_command(txt, True)
        print txt, response
        return response


    def Stop(self):
        self.TestMode(False)


    def BackToDock(self):
        while self._UpdateStatus()["charger"]["ExtPwrPresent"] == 0:
            self._SetMotor(-5, -5, 20)
            time.sleep(0.5)

    def Drive(self, LWheelDist = 0, RWheelDist = 0, Speed = 0, Accel = 0, RPM = 0):
        _watch_sensors = [("sensors",'LFRONTBIT'),
                          ("sensors",'LLDSBIT'),
                          ("sensors",'LSIDEBIT'),
                          ("sensors",'RFRONTBIT'),
                          ("sensors",'RLDSBIT'),
                          ("sensors",'RSIDEBIT'),
                          ("analog", 'DropSensorLeft'),
                          ("analog", 'DropSensorRight')]
        Speed = min(Speed, 350)

        if LWheelDist != 0 or RWheelDist != 0:
            if Speed == 0:
                Speed = 100
            if Accel == 0:
                Accel = Speed

        exp_time = max(abs(LWheelDist), abs(RWheelDist)) / Speed
        print "Expecting to drive for %f sec" % exp_time
        start = time.time()
        self._SetMotor(LWheelDist, RWheelDist, Speed, Accel, RPM)
        while time.time() < (start + exp_time):
            status = self.GetStatus()
            for group, sensor in _watch_sensors:
                value = status[group][sensor]
                if value > 0:
                    msg = "Stopping! Sensor %s = %s" % (sensor, value)
                    print msg
                    self.Stop()
                    return msg
            time.sleep(0.1)
            
        
    def GetStatus(self):
        return self.status

    def _UpdateStatus(self):
        self.status = dict(error=self.GetErr(),
                           sensors=self.GetDigitalSensors(),
                           analog=self.GetAnalogSensors(),
                           motors=self.GetMotors(),
                           charger=self.GetCharger(),
                      #accel=self.GetAccel(),
                           lds=self.GetLDSScan(),
                           updated=time.time(),
                       )
        state = "Idle"
        if self.status["charger"]["ExtPwrPresent"]:
            state = "Docked"
        if self.status["motors"]["Vacuum_RPM"] and self.status["motors"]["Brush_RPM"]:
            state = "Cleaning"
        self.status["state"] = state
        return self.status

class NeatoDummy(Neato):
    def __init__(self, dump_file):
        print "This is a dummy Neato class"
        with open(dump_file) as fd:
            self._status_history = json.load(fd)

        self._current_item = 0
        self._UpdateStatus()
        thread.start_new_thread(self.update_status_loop, ())

    def do_command(self, txt, *args):
        print("UART Write: [%s]" % txt)

    def _UpdateStatus(self):
        self.status = self._status_history[self._current_item]
        self._current_item = (self._current_item + 1) % len(self._status_history)


        
