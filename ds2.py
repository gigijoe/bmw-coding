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

#
# MS41
#

class DS2(object):
    def __init__(self):
        self._device = serial.Serial("/dev/ttyUSB0", 9600, parity=serial.PARITY_EVEN, timeout=0.5)    

    def run(self):
        for address in [ DME ]:
            print("Querying DME " + hex(address))
            data = self._execute(address, bytes(b'\x00'))
            #raw = b'\x12\x1D\xA0\x02\xBF\x00\x26\x17\xAB\x4E\x41\x59\x02\x49\x07\x24\x6A\x88\x22\x7F\x80\x00\x80\x00\x38\x38\xCE\xCE\x09'
            #raw = b'\x12\x1D\xA0\x03\x20\x00\x24\x10\xA3\x91\x38\x6A\x01\xB9\x00\xCE\x4E\x22\x1E\x88\x8F\x3A\x6D\xBA\x87\x6C\xCE\xCE\xDD'
            #data = struct.unpack('<' + 'B'*len(raw), raw)
            time.sleep(0.03)

    def sniffer(self):
        self._read()

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
        p = bytearray()
        p.append(address)
        p.append(size)
        for x in payload:
            p.append(x)
        p.append(self._checksum(p))
        print("TX : " + ''.join('{:02x} '.format(x) for x in p))
        self._device.write(p)

    def _read(self):
        p = bytearray()
        try:
            address = self._device.read(1)[0]
        except IndexError:
            return None
        p.append(address)
        size = self._device.read(1)[0]
        #size = 0x1D
        p.append(size)
        remaining = ord(size) - 3
        if remaining > 0:
            payload = self._device.read(remaining)
            for x in payload:
                p.append(x)
        expected_checksum = self._checksum(p)
        actual_checksum = self._device.read(1)[0]
        #actual_checksum = 0xDD
        p.append(actual_checksum)
        print("RX : " + ''.join('{:02x} '.format(x) for x in p))
        if ord(actual_checksum) != expected_checksum:
            raise ProtocolError("invalid checksum")
        return p

    def _checksum(self, message):
        result = 0
        for b in message:
            result ^= b
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

#
# Bosch Motronic v7.2 (M62TU) - KWP2000 protocol
#

class ME72(DS2):
    def run(self):
        for address in [ DME ]:
            print("Querying DME " + hex(address))
            source = bytes(b'\xf1')
            data = self._execute(address, source, bytes(b'\xa2')) # b8 12 f1 01 a2 f8
            time.sleep(1.0)
            data = self._execute(address, source, bytes(b'\x22\x40\x00')) # b8 12 f1 03 22 40 00 3a
            time.sleep(1.0)
            data = self._execute(address, source, bytes(b'\x22\x40\x07')) # b8 12 f1 03 22 40 07 3d 
            time.sleep(1.0)
            #data = self._execute(address, source, bytes(b'\x22\x40\x05'))
            #time.sleep(1.0)

    def _write(self, address, source, payload):
        p = bytearray()
        p.append(0xb8)
        p.append(address)
        p.append(source)
        p.append(len(payload))
        for x in payload:
            p.append(x)
        p.append(self._checksum(p))
        print("TX : " + ''.join('{:02x} '.format(x) for x in p))

        self._device.write(p)        
        return 

    def _read(self):
        p = bytearray()
        try:
            header = self._device.read(1)[0]
        except IndexError:
            return None
        p.append(header)
        source = self._device.read(1)[0]
        p.append(source)
        address = self._device.read(1)[0]
        p.append(address)
        size = self._device.read(1)[0]
        p.append(size)
        remaining = ord(size)
        if remaining > 0:
            payload = self._device.read(remaining)
            for x in payload:
                p.append(x)
        expected_checksum = self._checksum(p)
        actual_checksum = self._device.read(1)[0]
        p.append(actual_checksum)
        print("RX : " + ''.join('{:02x} '.format(x) for x in p))
        if ord(actual_checksum) != expected_checksum:
            raise ProtocolError("invalid checksum")
        return p

    def _execute(self, address, source, payload):
        self._write(address, source, payload)
        echo = self._read()
        #self._device.timeout = 0.1
        reply = self._read()
        if reply is None:
            raise InvalidAddress("invalid header")
        header = reply[0]
        if header != 0xB8:
            raise ProtocolError("unexpected header")

        p = reply[4:]
        if payload == bytes(b'\xa2'):
            """
                b8 12 f1 01 a2 f8

                b8 f1 12 2b  
                e2 
                37 35 30 36 33 36 36 # 7506366 part number
                30 46 # 0F hardware number
                30 31 # 01 coding index
                41 38 # A8 diag number
                36 30 # 60 bus index
                30 38 # 08 build date.week
                30 30 # 00 build date.year
                30 30 31 30 32 31 # 001031 supplier
                33 35 31 30 
                ff ff # software number
                ff ff 30 30 30 30 38 33 38 32 38 99
            """
            part_number = p[1:8]
            print("part number : " + part_number.decode('utf-8'))
            hardware_number = p[8:10]
            print("hardware number : " + hardware_number.decode('utf-8'))
            coding_index = p[10:12]
            print("coding index : " + coding_index.decode('utf-8'))
            diag_index = p[12:14]
            print("diag index : " + diag_index.decode('utf-8'))
            bus_index = p[14:16]
            print("bus index : " + bus_index.decode('utf-8'))
            build_date_week = p[16:18]
            print("build date week : " + build_date_week.decode('utf-8'))
            build_date_year = p[18:20]
            print("build date year : " + build_date_year.decode('utf-8'))
            supplier = p[20:26]
            print("supplier : " + supplier.decode('utf-8'))
        
        elif payload == bytes(b'\x22\x40\x00'):
            """
                b8 12 f1 03 22 40 00 3a

                b8 f1 12 2d
                
                62 40 00 00 00 80 00 80 00 
                00 # speed
                00 00 # rpm
                50 
                00 00 # bsnk 1 camshift intake position  
                00 00 # bsnk 2 camshift intake position
                65 # intake air tempature
                60 # coolant temperature 
                00 # ignition timing 
                05 # throttle angle 
                00 # air mass
                00 a8 d1 7f 00 94 
                5e # coolant outlet temperature
                0c 80 0c 80 0c 80 0c 80 0c 80 0c 80 0c 40 0c 80 08
            """
            speed = p[9]
            print("speed : " + str(speed * 1.25) + " km/h")
            rpm = struct.unpack('>H'*1, p[10:12])
            print("rpm : " + str(rpm[0] * 0.25) + " RPM")
            intake_air_temp = p[17]
            print("intake air temp : " + str(intake_air_temp * 0.75 - 48.0) + " C")
            coolant_temp = p[18]
            print("coolant temp : " + str(coolant_temp * 0.75 - 48.0) + " C")
            engine_throttle_angle = p[20]
            print("engine throttle angle : " + str(engine_throttle_angle * 0.39216) + " %")
            engine_air_mass = p[21]
            print("engine air mass : " + str(engine_air_mass * 0.1) + " kg/h")
            coolant_outlet_temp = p[28]
            print("coolant outlet temp : " + str(coolant_outlet_temp * 0.75 - 48.0) + " C")
        
        elif payload == bytes(b'\x22\x40\x05'):
            print("Unimplement")

        elif payload == bytes(b'\x22\x40\x07'):
            """
            b8 f1 12 05 62 40 07 01 90 ea
            """
            b = p[3]
            print("neutral switch : " + str(b & (1 << 0)))
            print("acceleration enrichment : " + str(b & (1 << 1)))
            print("oxygen sensor after bank 2 ready : " + str(b & (1 << 2)))
            print("oxygen sensor after bank 1 ready : " + str(b & (1 << 3)))
            print("oxygen sensor before bank 2 ready : " + str(b & (1 << 4)))
            print("oxygen sensor before bank 1 ready : " + str(b & (1 << 5)))

    def _checksum(self, message):
        result = 0
        for b in message:
            result ^= b
        return result

#
# Automatic Transmission ZF5HP24 v8.60.2 (E38/E39/E53)
#

class ZF5HP24(DS2):
    def run(self):
        for address in [ EGS ]:
            print("Querying EGS " + hex(address))
            data = self._execute(address, bytes(b'\x00'))
            time.sleep(1.0)
            data = self._execute(address, bytes(b'\x0B\x03'))
            time.sleep(1.0)

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
        if sender != address:
            raise ProtocolError("unexpected sender")
        if status != 0xa0:
            if status == 0xa1:
                raise ComputerBusy("computer busy")
            elif status == 0xa2:
                raise InvalidCommand("invalid parameter")
            elif status == 0xff:
                raise InvalidCommand("invalid command")
            else:
                raise ProtocolError("unknown status")
            return

        p = reply[2:]
        if payload == bytes(b'\x00'):
            part_number = p[1:8]
            print("part number : " + part_number.decode('utf-8'))
            hardware_number = p[8:10]
            print("hardware number : " + hardware_number.decode('utf-8'))
            coding_index = p[10:12]
            print("coding index : " + coding_index.decode('utf-8'))
            diag_index = p[12:14]
            print("diag index : " + diag_index.decode('utf-8'))
            bus_index = p[14:16]
            print("bus index : " + bus_index.decode('utf-8'))

        elif payload == bytes(b'\x0B\x03'):
            """
                32 05 0b 03 3f
                
                32 1c a0 
                00 # rpm
                00 # input turbine rpm 
                00 # output shift rpm 
                00 4f 
                4b # coolant temperature 
                54 # transmission temperature
                93 01 01 01 01 ff ff ff ff ff
                02 
                dc # shifter program 
                00 # cruise control mode
                80 # current gear | shifter steptronic | kick down | vehicle in curve
                00 # last shift 
                00 # user program | actual program
                59 b5

            """
            rpm = p[1] # struct.unpack('>B'*1, p[1])
            print("rpm : " + str(rpm * 32) + " RPM")
            input_turbine_rpm = p[2]
            print("input turbine rpm : " + str(input_turbine_rpm * 32) + " RPM")
            output_shift_rpm = p[3]
            print("output shift rpm : " + str(output_shift_rpm * 32) + " RPM")
            coolant_temp = p[6]
            print("coolant temperature : " + str(coolant_temp - 48) + " C")
            transmission_temp = p[7]
            print("transmission temperature : " + str(transmission_temp - 54) + " C")
            cruise_control = p[20]
            if cruise_control == 0x0:
                print("cruise control mode : off")
            elif cruise_control == 0x20:
                print("cruise control mode : on")
            elif cruise_control == 0x40:
                print("cruise control mode : resume")
            elif cruise_control == 0x60:
                print("cruise control mode : accel")
            elif cruise_control == 0x80:
                print("cruise control mode : decel")
            else:
                printf("cruise control mode : unknown")

            gear = p[21]            
            g = gear >> 5
            if g == 0x6:
                print("gear : 1")
            elif g == 0x7:
                print("gear : reverse")
            else:
                print("gear : " + str(gear >> 5))

            shifter = gear & 0x3
            if shifter == 0x1:
                print("shifter steptronic : up")
            elif shifter == 0x2:
                print("shifter steptronic : down")
            else:
                print("shifter steptronic : neutral")
            kickdown = gear & 0x10
            if kickdown:
                print("kickdown : yes")
            else:
                print("kickdown : no")
            vehicle_in_curve = gear & 0x8
            if vehicle_in_curve:
                print("vehicle in curve : yes")
            else:
                print("vehicle in curve : no")

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
        p = bytearray()
        p.append(address)
        p.append(size)
        for x in payload:
            p.append(x)
        p.append(self._checksum(p))
        print("TX : " + ''.join('{:02x} '.format(x) for x in p))
        self._device.write(p)

    def _read(self):
        p = bytearray()
        try:
            address = self._device.read(1)[0]
        except IndexError:
            return None
        p.append(address)
        size = self._device.read(1)[0]
        p.append(size)
        remaining = ord(size) - 3
        if remaining > 0:
            payload = self._device.read(remaining)
            for x in payload:
                p.append(x)
        expected_checksum = self._checksum(p)
        actual_checksum = self._device.read(1)[0]
        p.append(actual_checksum)
        print("RX : " + ''.join('{:02x} '.format(x) for x in p))
        if ord(actual_checksum) != expected_checksum:
            raise ProtocolError("invalid checksum")
        return p

    def _checksum(self, message):
        result = 0
        for b in message:
            result ^= b
        return result

class ProtocolError(Exception):
    pass

class ComputerBusy(Exception):
    pass

class InvalidAddress(Exception):
    pass

class InvalidCommand(Exception):
    pass


egs = ZF5HP24()
egs.run()

dme = ME72()
dme.run()

while 1:
    dme.sniffer()
