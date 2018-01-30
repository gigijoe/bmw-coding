#-*-coding:utf-8 -*-
#from hexdump import hexdump
import serial
import time

import struct
from struct import unpack

ZKE = 0x00 # Central Body Electronics / Zentrale Karosserieelektronik
DME = 0x12 # Digital Motor Electronics
CENTRAL_BODY = 0x21
EGS = 0x32 # Electronic Transmission Control - Electronische Getriebe Steuerung
EWS = 0x44 # Electronic Immobiliser / Elektronische Wegfahrsperre
DSC = 0x56 # Dynamic Stability Control
IHKA = 0x5B # Auto Climate Control / Integrierte Heizung Kühlung
IKE = 0x80 # Instrument Cluster
AIRBAG = 0xA4 # Multi Restraint System
LCM = 0xD0 # Light Switching Center / Lichtschaltzentrum
MID = 0xC0 # Multi Information Display
RADIO = 0x68 # Radio
SZM = 0xf5 # Center Console Switching Center

class Application(object):
    def __init__(self):
        self._device = serial.Serial("/dev/ttyS0", 9600, parity=serial.PARITY_EVEN, timeout=0.5)

    def run(self):
        for address in [ DME ]:
            print("Querying " + hex(address))
            #data = self._execute(address, bytes([0x0B, 0x03]))
            #raw = b'\x12\x1D\xA0\x02\xBF\x00\x26\x17\xAB\x4E\x41\x59\x02\x49\x07\x24\x6A\x88\x22\x7F\x80\x00\x80\x00\x38\x38\xCE\xCE\x09'
            raw = b'\x12\x1D\xA0\x03\x20\x00\x24\x10\xA3\x91\x38\x6A\x01\xB9\x00\xCE\x4E\x22\x1E\x88\x8F\x3A\x6D\xBA\x87\x6C\xCE\xCE\xDD'
            data = struct.unpack('<' + 'B'*len(raw), raw)
            if data[0] == DME:
                self._decode_dme(raw[3:])
            else:
                print(data[0])
            #print("Got:")
            #hexdump(data)
            # Delay to work around unknown issue (perhaps a bug in MultiCom?)
            time.sleep(0.03)

    def _execute(self, address, payload):
        self._write(address, payload)
        echo = self._read()
        #self._device.timeout = 0.1
        reply = self._read()
        if reply is None:
            raise InvalidAddress("invalid address")
        sender = reply[0]
        length = reply[1]
        status = reply[2]
        #sender, payload = reply
        #self._device.timeout = None
        if sender != address:
            raise ProtocolError("unexpected sender")
        #status = payload[0]
        if status == 0xa0:
            return reply
        elif status == 0xa1:
            raise ComputerBusy("computer busy")
        elif status == 0xa2:
            raise InvalidCommand("invalid parameter")
        elif status == 0xff:
            raise InvalidCommand("invalid command")
        else:
            raise ProtocolError("unknown status")

    def _write(self, address, payload):
        size = 2 + len(payload) + 1
        message = bytes([address, size]) + payload
        buf = message + bytes([self._checksum(message)])
        #hexdump(buf)
        self._device.write(buf)

    def _read(self):
        try:
            address = self._device.read(1)[0]
        except IndexError:
            return None
        size = self._device.read(1)[0]
        remaining = size - 3
        if remaining > 0:
            payload = self._device.read(remaining)
        else:
            payload = bytes([])
        expected_checksum = self._checksum(bytes([address, size]) + payload)
        actual_checksum = self._device.read(1)[0]
        if actual_checksum != expected_checksum:
            raise ProtocolError("invalid checksum")
        return (address, payload)

    def _checksum(self, message):
        result = 0
        for b in message:
            result ^= ord(b) 
        return result

    def _decode_dme(self, data):
        print("<<< _decode_dme >>>")        
        engine_speed = struct.unpack('>H'*1, data[0:2])
        print("engine speed : " + str(engine_speed[0]) + " 1/min")
        vehicle_speed = struct.unpack('>B'*1, data[2])
        print("vehicle speed : " + str(vehicle_speed[0]) + " km/h")
        throttle_position = struct.unpack('>B'*1, data[3])
        print("throttle position : " + str(throttle_position[0] * 0.47) + " %")
        engine_load = struct.unpack('>H'*1, data[4:6])
        print("engine load : " + str(engine_load[0] * 0.021) + " mg/stroke")
        air_temp = struct.unpack('>B'*1, data[6])
        print("air temp : " + str(air_temp[0] * (-0.458) + 108) + " C")
        coolant_temp = struct.unpack('>B'*1, data[7])
        print("coolant temp : " + str(coolant_temp[0] * (-0.458) + 108) + " C")
        ignition_time_advance = struct.unpack('>B'*1, data[8])
        print("ignition time advance : " + str(ignition_time_advance[0] * (0.373) + (-23.6)) + " BTDC")
        injector_pulsewidth = struct.unpack('>H'*1, data[9:11])
        print("injector pulse width : " + str(injector_pulsewidth[0] * 0.00534) + " ms")
        IACV = struct.unpack('>H'*1, data[11:13])
        print("IACV : " + str(IACV[0] * 0.00153) + " %")
        # struct.unpack('>H'*1, data[13:15])
        vanos_angle = struct.unpack('>B'*1, data[15])
        print("vanos angle : " + str(vanos_angle[0] * 0.3745) + " KW degrees")
        battery_voltage = struct.unpack('>B'*1, data[16])
        print("battery voltage : " + str(battery_voltage[0] * 0.10196) + " volts")
        # Lambda Integrator 1
        # Lambda Integrator 2
        lambda_upstream_heater_1 = struct.unpack('>B'*1, data[21])
        print("lambda upstream heater 1 : " + str(lambda_upstream_heater_1[0] * 0.3906) + " %")
        lambda_upstream_heater_2 = struct.unpack('>B'*1, data[22])
        print("lambda upstream heater 2 : " + str(lambda_upstream_heater_2[0] * 0.3906) + " %")
        lambda_downstream_heater_1 = struct.unpack('>B'*1, data[23])
        print("lambda downstream heater 1 : " + str(lambda_downstream_heater_1[0] * 0.3906) + " %")
        lambda_downstream_heater_2 = struct.unpack('>B'*1, data[24])
        print("lambda downstream heater 2 : " + str(lambda_downstream_heater_2[0] * 0.3906) + " %")



class ProtocolError(Exception):
    pass

class ComputerBusy(Exception):
    pass

class InvalidAddress(Exception):
    pass

class InvalidCommand(Exception):
    pass


app = Application()
app.run()
