import serial
import io
import sys
import time
import os
import threading

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


class Neato(object):
    def __init__(self, port):
        self._uart_connected = False
        self._uart_lock = threading.Lock()
        for i in range(2):
            try:
                self.port = serial.Serial(port, timeout=0.1)
                self._uart_connected = True
            #self.port = io.TextIOWrapper(io.BufferedRWPair(self._port, self._port), line_buffering=True)
            except:
                print "Could not open serial port %s" % port
                self.port = sys.stderr
                print "Trying to wake neato"
                wake_neato()


        self.command_sinks = [self._serial_write]

    def _serial_write(self, cmd):
        with self._uart_lock:
            self.port.write(cmd)
            response = self.port.readlines()
        return response[1:-1]

    def write(self, cmd):
        #print("Cmd: %s" % cmd)
        cmd = cmd + "\n"
        resp = None
        for sink in self.command_sinks:
            r = sink(cmd)
            if r:
                resp = r
        return resp

    def GetErr(self):
        return self.write("GetErr")

        
    def GetDigitalSensors(self):
        response = self.write("GetDigitalSensors")
        response = [a.strip().split(",") for a in response[1:]]
        response = [(a, int(b)) for (a, b) in response]
        return dict(response)

    def GetAnalogSensors(self):
        response = self.write("GetAnalogSensors")
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
        response = self.write("GetAccel")
        response = [a.strip().split(",") for a in response[1:]]
        response = [(a, float(b)) for (a, b) in response]
        return dict(response)

    def GetMotors(self):
        response = self.write("GetMotors")
        response = [a.strip().split(",") for a in response[1:]]
        response = [(a, int(b)) for (a, b) in response]
        return dict(response)

    
    def SetLDSRotation(self, value):
        return self.write("SetLDSRotation %s" % ("On" if value else "Off"))
    
    def GetLDSScan(self):
        raw = self.write("GetLDSScan")
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
        print self.write(txt)


    def Clean(self, arg=None):
        valid_args=[None, "House", "Spot", "Stop"]
        if arg not in valid_args:
            raise Exception("Received unexpected argument: %s, expected one of %s" % (arg, valid_args))
        cmd = "Clean %s" % (arg or "")

    def PlaySound(self, sound):
        self.write("PlaySound %d" % sound)
        
    def SetMotor(self, LWheelDist = 0, RWheelDist = 0, Speed = 0, Accel = 0, RPM = 0):
        self.TestMode(True)
        if LWheelDist != 0 or RWheelDist != 0:
            if Speed == 0:
                Speed = 100
            if Accel == 0:
                Accel = Speed
        txt = "SetMotor LWheelDist %d RWheelDist %d Speed %d Accel %d RPM %d" % (LWheelDist, RWheelDist, Speed, Accel, RPM)
        response = self.write(txt)
        self.TestMode(False)
        return response

    def SmartDrive(self, LWheelDist = 0, RWheelDist = 0, Speed = 0, Accel = 0, RPM = 0):
        exp_time = max(abs(LWheelDist), abs(RWheelDist)) / 100
        print "Expecting to drive for %d sec" % exp_time
        start = time.time()
        self.SetMotor(LWheelDist, RWheelDist, Speed, Accel, RPM)
        while time.time() < (start + exp_time):
            sensors = self.GetDigitalSensors()
            if "1" in sensors.values():
                print "Stopping! %s" % sensors
                self.SetMotor(0,0,0)
                return
            
        
        
    def GetStatus(self):
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
        self.command_sinks = [self.debug]

    def debug(self, txt):
        print("UART Write: [%s]" % txt)

    def GetStatus(self):
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

        
