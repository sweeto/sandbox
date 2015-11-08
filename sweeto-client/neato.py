import serial
import io
import sys
import time
import os
import functools
import threading, thread

def wake_neato():
    dev = "/sys/kernel/debug/gpio_debug/gpio15/"
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

def synchronized(wrapped):
    lock = threading.RLock()
    @functools.wraps(wrapped)
    def _wrapper(*args, **kwargs):
        with lock:
            return wrapped(*args, **kwargs)
        return _wrapper

    
class Neato(object):
    def __init__(self, port):
        self._uart_connected = False
        self.lock = threading.Lock()
        self.status = {}
        for i in range(2):
            try:
                self.port = serial.Serial(port, timeout=0.1)
                self._uart_connected = True
            except:
                print "Could not open serial port %s" % port
                self.port = sys.stderr
                print "Trying to wake neato"
                wake_neato()

        thread.start_new_thread(self.update_status_loop, ())

    def update_status_loop(self):
        loop_time=1
        print "Starting update loop"
        while True:
            start = time.time()
            self.status = self._GetStatus()
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

    def do_command(self, cmd):
        with self.lock:
            self.port.write(cmd +"\n")
            response = self._read()
            response = response.split("\r\n")
            if response[0] != cmd:
                print "Response does not match cmd: [%s] != [%s] " %(response[0], cmd)
            return response[1:-1]
        
    def GetErr(self):
        return self.do_command("GetErr")

        
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
                formatted.append(("%s [%s]" %(name, unit), value))
            except:
                print "Could not parse %s" % line
        return dict(formatted)

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
        return self.do_command("SetLDSRotation %s" % ("On" if value else "Off"))
    
    def GetLDSScan(self):
        raw = self.do_command("GetLDSScan")
        lines = raw.split("\r\n")
        values = []
        for line in lines:
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
        txt = "TestMode %s" % ("On" if value else "Off")
        self.do_command(txt)


    def Clean(self, arg=None):
        valid_args=[None, "House", "Spot", "Stop"]
        if arg not in valid_args:
            raise Exception("Received unexpected argument: %s, expected one of %s" % (arg, valid_args))
        cmd = "Clean %s" % (arg or "")

    def PlaySound(self, sound):
        self.do_command("PlaySound %d" % sound)

    def SetMotor(self, *args, **kwargs):
        return self.SmartDrive(*args, **kwargs)

    def _SetMotor(self, LWheelDist = 0, RWheelDist = 0, Speed = 0, Accel = 0, RPM = 0):
        self.TestMode(True)
        self.TestMode(True)

        if LWheelDist != 0 or RWheelDist != 0:
            if Speed == 0:
                Speed = 100
            if Accel == 0:
                Accel = Speed
        txt = "SetMotor LWheelDist %d RWheelDist %d Speed %d Accel %d RPM %d" % (LWheelDist, RWheelDist, Speed, Accel, RPM)
        response = self.do_command(txt)
        print txt, response
        return response

    def SmartDrive(self, LWheelDist = 0, RWheelDist = 0, Speed = 0, Accel = 0, RPM = 0):
        exp_time = max(abs(LWheelDist), abs(RWheelDist)) / 100
        print "Expecting to drive for %d sec" % exp_time
        start = time.time()
        self._SetMotor(LWheelDist, RWheelDist, Speed, Accel, RPM)
        while time.time() < (start + exp_time):
            sensors = self.GetStatus()["sensors"]
            if 1 in sensors.values():
                print "Stopping! %s" % sensors
                self.TestMode(False)
                return
            time.sleep(0.1)
            
        
    def GetStatus(self):
        return self.status


    def _GetStatus(self):
        status = dict(error=self.GetErr(),
                      sensors=self.GetDigitalSensors(),
                      analog=self.GetAnalogSensors(),
                      motors=self.GetMotors(),
                      accel=self.GetAccel(),
        )
        
        return status
                            

class NeatoDummy(Neato):
    def __init__(self):
        print "This is a dummy Neato class"
        thread.start_new_thread(self.update_status_loop, ())
        
    def do_command(self, txt):
        print("UART Write: [%s]" % txt)

    def _GetStatus(self):
        s = {'accel': {'PitchInDegrees': 0.050000000000000003,
                    'RollInDegrees': 1.4299999999999999,
                    'SumInG': 0.97299999999999998,
                    'XInG': -0.001,
                    'YInG': 0.024,
                    'ZInG': 0.97299999999999998},
          'analog': {'AccelerometerX [mG]': -6,
                     'AccelerometerY [mG]': 21,
                     'AccelerometerZ [mG]': 977,
                     'BatteryCurrent [mA]': -155,
                     'BatteryTemperature [mC]': 26063,
                     'BatteryVoltage [mV]': 12704,
                     'DropSensorLeft [mm]': 0,
                     'DropSensorRight [mm]': 0,
                     'ExternalVoltage [mV]': 17988,
                     'MagSensorLeft [VAL]': 0,
                     'MagSensorRight [VAL]': 0,
                     'SideBrushCurrent [mA]': 0,
                     'VacuumCurrent [mA]': 0,
                     'WallSensor [mm]': 70},
          'error': ['\r\n'],
          'motors': {'Brush_RPM': 0,
                     'Brush_mA': 0,
                     'LeftWheel_Load%': 0,
                     'LeftWheel_PositionInMM': 0,
                     'LeftWheel_RPM': 0,
                     'LeftWheel_Speed': 0,
                     'RightWheel_Load%': 0,
                     'RightWheel_PositionInMM': 0,
                     'RightWheel_RPM': 0,
                     'RightWheel_Speed': 0,
                     'SideBrush_mA': 0,
                     'Vacuum_RPM': 0,
                     'Vacuum_mA': 0},
          'sensors': {'LFRONTBIT': 0,
                      'LLDSBIT': 0,
                      'LSIDEBIT': 0,
                      'RFRONTBIT': 0,
                      'RLDSBIT': 0,
                      'RSIDEBIT': 0,
                      'SNSR_DC_JACK_IS_IN': 0,
                      'SNSR_DUSTBIN_IS_IN': 0,
                      'SNSR_LEFT_WHEEL_EXTENDED': 0,
                      'SNSR_RIGHT_WHEEL_EXTENDED': 0}}
        return s

        
