#-*-coding:utf-8 -*-
#from hexdump import hexdump
import serial
import time

import struct
from struct import unpack

def byte_to_int(char):
    if char > 127:
        return (256-char) * (-1)
    else:
        return char

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

class K_Line(object):
    def __init__(self):
        self._device = serial.Serial("/dev/ttyUSB0", 9600, parity=serial.PARITY_EVEN, timeout=0.5)    

    def _checksum(self, message):
        result = 0
        for b in message:
            result ^= b
        return result

#
# DS2
#

class DS2(K_Line):
    def sniffer(self):
        print("DS2 sniffer ...")
        self._read()

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
        p.append(actual_checksum)
        print("RX : " + ''.join('{:02x} '.format(x) for x in p))
        if ord(actual_checksum) != expected_checksum:
            raise ProtocolError("invalid checksum")
        return p

    def _execute(self, address, payload):
        self._write(address, payload)
        echo = self._read()
        #self._device.timeout = 0.1
        reply = self._read()
        if reply is None:
            print("No response - Invalid Address ...")
            return None
        sender = reply[0]
        length = reply[1]
        status = reply[2]
        if sender != address:
            print("Unexpected address")
            return
        if status != 0xa0:
            if status == 0xa1:
                print("Computer busy")
            elif status == 0xa2:
                print("Invalid parameter")
            elif status == 0xff:
                print("Invalid command")
            else:
                print("Unknown status")
            return None
        return reply

#
# KWP2000
#

class KWP2000(K_Line):
    def sniffer(self):
        print("KWP2000 sniffer ...")
        self._read()

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
            print("No response - Invalid Address ...")
            return None
        header = reply[0]
        if header != 0xB8:
            print("Unexpected header")
            return None
        return reply
#
# MS41
#

class MS41(DS2):
    def run(self):
        for address in [ DME ]:
            print("Querying DME " + hex(address))
            data = self._execute(address, bytes(b'\x00'))
            time.sleep(0.2)
            #raw = b'\x12\x1D\xA0\x02\xBF\x00\x26\x17\xAB\x4E\x41\x59\x02\x49\x07\x24\x6A\x88\x22\x7F\x80\x00\x80\x00\x38\x38\xCE\xCE\x09'
            #raw = b'\x12\x1D\xA0\x03\x20\x00\x24\x10\xA3\x91\x38\x6A\x01\xB9\x00\xCE\x4E\x22\x1E\x88\x8F\x3A\x6D\xBA\x87\x6C\xCE\xCE\xDD'
            #data = struct.unpack('<' + 'B'*len(raw), raw)

    def _execute(self, address, payload):
        reply = super(MS41, self)._execute(address, payload)
        if reply is None:
            return

        p = reply[2:]
        if payload == bytes(b'\xa2'):
            engine_speed = struct.unpack('>H'*1, p[0:2])
            print("engine speed : " + str(engine_speed[0]) + " 1/min")
            vehicle_speed = struct.unpack('>B'*1, p[2])
            print("vehicle speed : " + str(vehicle_speed[0]) + " km/h")
            throttle_position = struct.unpack('>B'*1, p[3])
            print("throttle position : " + str(throttle_position[0] * 0.47) + " %")
            engine_load = struct.unpack('>H'*1, p[4:6])
            print("engine load : " + str(engine_load[0] * 0.021) + " mg/stroke")
            air_temp = struct.unpack('>B'*1, p[6])
            print("air temp : " + str(air_temp[0] * (-0.458) + 108) + " C")
            coolant_temp = struct.unpack('>B'*1, p[7])
            print("coolant temp : " + str(coolant_temp[0] * (-0.458) + 108) + " C")
            ignition_time_advance = struct.unpack('>B'*1, p[8])
            print("ignition time advance : " + str(ignition_time_advance[0] * (0.373) + (-23.6)) + " BTDC")
            injector_pulsewidth = struct.unpack('>H'*1, p[9:11])
            print("injector pulse width : " + str(injector_pulsewidth[0] * 0.00534) + " ms")
            IACV = struct.unpack('>H'*1, p[11:13])
            print("IACV : " + str(IACV[0] * 0.00153) + " %")
            # struct.unpack('>H'*1, p[13:15])
            vanos_angle = struct.unpack('>B'*1, p[15])
            print("vanos angle : " + str(vanos_angle[0] * 0.3745) + " KW degrees")
            battery_voltage = struct.unpack('>B'*1, p[16])
            print("battery voltage : " + str(battery_voltage[0] * 0.10196) + " volts")
            # Lambda Integrator 1
            # Lambda Integrator 2
            lambda_upstream_heater_1 = struct.unpack('>B'*1, p[21])
            print("lambda upstream heater 1 : " + str(lambda_upstream_heater_1[0] * 0.3906) + " %")
            lambda_upstream_heater_2 = struct.unpack('>B'*1, p[22])
            print("lambda upstream heater 2 : " + str(lambda_upstream_heater_2[0] * 0.3906) + " %")
            lambda_downstream_heater_1 = struct.unpack('>B'*1, p[23])
            print("lambda downstream heater 1 : " + str(lambda_downstream_heater_1[0] * 0.3906) + " %")
            lambda_downstream_heater_2 = struct.unpack('>B'*1, p[24])
            print("lambda downstream heater 2 : " + str(lambda_downstream_heater_2[0] * 0.3906) + " %")

        else:
            print("Unknown payload")     

#
# Bosch Motronic v7.2 (M62TU) - KWP2000 protocol
#

class ME72(KWP2000):
    def run(self):
        for address in [ DME ]:
            print("Querying DME " + hex(address))
            source = bytes(b'\xf1')
            self._execute(address, source, bytes(b'\xa2')) # b8 12 f1 01 a2 f8
            time.sleep(0.2)
            self._execute(address, source, bytes(b'\x22\x40\x00')) # b8 12 f1 03 22 40 00 3a
            time.sleep(0.2)
            self._execute(address, source, bytes(b'\x22\x40\x03'))
            time.sleep(0.2)
            self._execute(address, source, bytes(b'\x22\x40\x04'))
            time.sleep(0.2)
            self._execute(address, source, bytes(b'\x22\x40\x05'))
            time.sleep(0.2)
            self._execute(address, source, bytes(b'\x22\x40\x07')) # b8 12 f1 03 22 40 07 3d 
            time.sleep(0.2)

    def _execute(self, address, source, payload):
        reply = super(ME72, self)._execute(address, source, payload)
        if reply is None:
            return

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
                62 40 00 
                00 00 # injection time 
                80 00 # Lambdaintegrator 1  
                80 00 # Lambdaintegrator 2
                00 # speed
                00 00 # current rpm
                50 # target rpm
                00 00 # bsnk 1 camshift intake position  
                00 00 # bsnk 2 camshift intake position
                65 # intake air tempature
                60 # coolant temperature 
                00 # ignition angle 
                05 # throttle angle 
                00 00 # air mass
                a8 d1 # Load 
                7f # Battery voltage 
                00 94 # pedal position
                5e # coolant outlet temperature
                0c 80 0c 80 0c 80 0c 80 0c 80 0c 80 0c 40 0c 80 # knock sensor of 8 cyl
                08 # check sum
            """
            injection_time = struct.unpack('>H'*1, p[3:5])
            print("injection time : " + str(injection_time[0] * 0.016) + " ms")
            speed = p[9]
            print("speed : " + str(speed * 1.25) + " km/h")
            rpm = struct.unpack('>H'*1, p[10:12])
            print("current rpm : " + str(rpm[0] * 0.25) + " RPM")
            rpm = p[12]
            print("target rpm : " + str(rpm * 10) + " RPM")
            intake_air_temp = p[17]
            print("intake air temp : " + str(intake_air_temp * 0.75 - 48.0) + " C")
            coolant_temp = p[18]
            print("coolant temp : " + str(coolant_temp * 0.75 - 48.0) + " C")
            #ignation_angle = struct.unpack('>b'*1, str(p[19]))
            ignation_angle = byte_to_int(p[19])
            print("ignation angle : " + str(ignation_angle * 0.75) + " Grad")
            engine_throttle_angle = p[20]
            print("engine throttle angle : " + str(engine_throttle_angle * 0.39216) + " %")
            engine_air_mass = struct.unpack('>H'*1, p[21:23])
            print("engine air mass : " + str(engine_air_mass[0] * 0.1) + " kg/h")
            load = struct.unpack('>H'*1, p[23:25])
            print("load : " + str(load[0] * 0.0015259) + " %")
            battery_voltage = p[25]
            print("battery voltage : " + str(battery_voltage * 0.095) + " V")
            pedal_position = struct.unpack('>H'*1, p[26:28])
            print("pedal position : " + str(pedal_position[0] * 0.0048828) + " V")
            coolant_outlet_temp = p[28]
            print("coolant outlet temp : " + str(coolant_outlet_temp * 0.75 - 48.0) + " C")

            r = struct.unpack('>h'*1, p[29:31])
            print("Knock sensor Cyl. 1 : " + str(r[0] * 0.019531) + " V")
            r = struct.unpack('>h'*1, p[31:33])
            print("Knock sensor Cyl. 2 : " + str(r[0] * 0.019531) + " V")
            r = struct.unpack('>h'*1, p[33:35])
            print("Knock sensor Cyl. 3 : " + str(r[0] * 0.019531) + " V")
            r = struct.unpack('>h'*1, p[35:37])
            print("Knock sensor Cyl. 4 : " + str(r[0] * 0.019531) + " V")
            r = struct.unpack('>h'*1, p[37:39])
            print("Knock sensor Cyl. 5 : " + str(r[0] * 0.019531) + " V")
            r = struct.unpack('>h'*1, p[39:41])
            print("Knock sensor Cyl. 6 : " + str(r[0] * 0.019531) + " V")
            r = struct.unpack('>h'*1, p[41:43])
            print("Knock sensor Cyl. 7 : " + str(r[0] * 0.019531) + " V")
            r = struct.unpack('>h'*1, p[43:45])
            print("Knock sensor Cyl. 8 : " + str(r[0] * 0.019531) + " V")
            
        elif payload == bytes(b'\x22\x40\x03'):
            r = struct.unpack('>h'*1, p[3:5])
            print("Roughness Cyl. 1 " + str(r[0] * 0.0027756) + " sec-1")
            r = struct.unpack('>h'*1, p[5:7])
            print("Roughness Cyl. 2 " + str(r[0] * 0.0027756) + " sec-1")
            r = struct.unpack('>h'*1, p[7:9])
            print("Roughness Cyl. 3 " + str(r[0] * 0.0027756) + " sec-1")
            r = struct.unpack('>h'*1, p[9:11])
            print("Roughness Cyl. 4 " + str(r[0] * 0.0027756) + " sec-1")
            r = struct.unpack('>h'*1, p[11:13])
            print("Roughness Cyl. 5 " + str(r[0] * 0.0027756) + " sec-1")
            r = struct.unpack('>h'*1, p[13:15])
            print("Roughness Cyl. 6 " + str(r[0] * 0.0027756) + " sec-1")
            r = struct.unpack('>h'*1, p[15:17])
            print("Roughness Cyl. 7 " + str(r[0] * 0.0027756) + " sec-1")
            r = struct.unpack('>h'*1, p[17:19])
            print("Roughness Cyl. 8 " + str(r[0] * 0.0027756) + " sec-1")
        
        elif payload == bytes(b'\x22\x40\x04'):
            r = struct.unpack('>H'*1, p[3:5])
            print("Adaptation additive 1 : " + str(r[0] * 0.046875) + " %")
            r = struct.unpack('>H'*1, p[5:7])
            print("Adaptation additive 2 : " + str(r[0] * 0.046875) + " %")
            r = struct.unpack('>H'*1, p[7:9])
            print("Adaptation multiplicative 1 : " + str(r[0] * 0.0000305) + " %")
            r = struct.unpack('>H'*1, p[9:11])
            print("Adaptation multiplicative 2 : " + str(r[0] * 0.0000305) + " %")

        elif payload == bytes(b'\x22\x40\x05'):
            r = p[9]
            print("leak diagnostic pump : " + "On" if (r & 0x01) > 0 else "Off")
            print("secondary air pump value : " + "On" if (r & 0x02) > 0 else "Off")
            print("oxgen sensor heater before bank 1 : " + "On" if (r & 0x10) > 0 else "Off")
            print("oxgen sensor heater before bank 2 : " + "On" if (r & 0x20) > 0 else "Off")
            print("oxgen sensor heater after bank 1 : " + "On" if (r & 0x40) > 0 else "Off")
            print("oxgen sensor heater after bank 2 : " + "On" if (r & 0x80) > 0 else "Off")
            r = p[10]
            print("exhaust gas recirculation : " + "On" if (r & 0x08) > 0 else "Off")
            print("electric fan : " + "On" if (r & 0x10) > 0 else "Off")
            print("fuel pump : " + "On" if (r & 0x20) > 0 else "Off")
            print("thermostat : " + "On" if (r & 0x40) > 0 else "Off")
            print("start mode : " + "On" if (r & 0x80) > 0 else "Off")

        elif payload == bytes(b'\x22\x40\x07'):
            """
            b8 f1 12 05 62 40 07 01 90 ea
            """
            b = p[3]
            print("neutral switch : " + str(b & (1 << 0)))
            print("acceleration enrichment : " + str(b & (1 << 1)))
            print("oxygen sensor after bank 2 ready : " + "Yes" if(b & (1 << 2)) > 0 else "No")
            print("oxygen sensor after bank 1 ready : " + "Yes" if(b & (1 << 3)) > 0 else "No")
            print("oxygen sensor before bank 2 ready : " + "Yes" if(b & (1 << 4)) > 0 else "No")
            print("oxygen sensor before bank 1 ready : " + "Yes" if(b & (1 << 5)) > 0 else "No")
        else:
            print("Unknown payload")

#
# Automatic Transmission ZF5HP24 v8.60.2 (E38/E39/E53)
# Gearbox Controller GS8.60.2 for E38, E39
# Decode reference Ecu/GS8602.PRG
#

error_description = {
    0x01:       "Pressure controller EDS 1",
    0x02:       "Pressure controller EDS 2",
    0x03:       "Pressure controller EDS 3",
    0x04:       "Pressure controller EDS 4",
    0x05:       "Pressure controller EDS 5",
    0x0F:       "Pressure controller EDS Total current",
    0x10:       "Solenoid Valve 1",
    0x11:       "Solenoid Valve 2",
    0x12:       "Solenoid Valve 3",
    0x13:       "Lift magnet Shift-Lock",
    0x20:       "Output rev. sensor (n-ab)",
    0x21:       "Turbine rev. sensor",
    0x22:       "Sump oil temperature sensor",
    0x30:       "Torque Converter Clutch - too much slip",
    0x31:       "Symptom gear check",
    0x32:       "Gear Check 1",
    0x33:       "Gear Check 1M/N",
    0x34:       "Gear Check 2",
    0x35:       "Gear Check 3",
    0x36:       "Gear Check 4",
    0x37:       "Gear Check 5",
    0x38:       "Symptom GLUE",
    0x39:       "GLUE-check 2/3",
    0x3A:       "GLUE-check 3/4",
    0x3B:       "Stalling speed",
    0x3C:       "Gearbox Switch",
    0x3D:       "Gearbox temperature check",
    0x50:       "ECU internal error 1 (EPROM)",
    0x51:       "ECU internal error 2 (EEPROM)",
    0x52:       "ECU internal error 3 (Watchdog)",
    0x53:       "ECU internal error 4 (FET)",
    0x60:       "V-Batt. supply Cl. 87",
    0x61:       "V-Batt. supply Cl. 30",
    0x70:       "Program Switch",
    0x71:       "Kick-Down Switch",
    0x72:       "Steptronic Switch",
    0x80:       "CAN-Bus check",
    0x81:       "CAN-Time-Out DME",
    0x82:       "CAN-Time-Out ASC",
    0x90:       "CAN Version error",
    0x93:       "CAN Throttle valve",
    0x94:       "CAN Engine temperature",
    0x95:       "CAN Wheel speeds",
    0x97:       "CAN Brake signal"
}

error_flags = {
    0x01:       "Plausibility",
    0x02:       "Short circuit to batt+",
    0x03:       "Short circuit to ground",
    0x04:       "Open circuit",
    0x05:       "Open circuit or Short circuit to batt+",
    0x06:       "Open circuit or Short circuit to ground",
    0x07:       "Too large",
    0x08:       "Too small",
    0x09:       "No change",
    0x0F:       "No suitable error code",
    0x10:       "Error activates MIL",
    0x20:       "Sporadic error",
    0x40:       "Replacement function active",
    0x80:       "Error present",
    0xFE:       "Gen. error",
    0xFF:       "Unknown"
}

class ZF5HP24(DS2):
    def run(self):
        for address in [ EGS ]:
            print("Querying EGS " + hex(address))
            data = self._execute(address, bytes(b'\x00'))
            time.sleep(0.2)
            data = self._execute(address, bytes(b'\x0B\x03'))
            time.sleep(0.2)
            data = self._execute(address, bytes(b'\x04\x01'))
            time.sleep(0.2)

    def _execute(self, address, payload):
        reply = super(ZF5HP24, self)._execute(address, payload)
        if reply is None:
            return

        p = reply[2:]
        if payload == bytes(b'\x00'):
            """
                32 04 00 36 
                32 2e 
                a0 # status 
                31 34 32 33 39 35 33 # part number 
                32 42 # hardware number
                30 30 # coding index
                31 31 #diag index 
                36 30 # bus index 
                34 38 # build date week 
                39 39 # build date year 
                30 30 30 30 30 30 30 30 30 30 # life number 
                30 39 # software number 
                31 30 # ai number 
                46 4f 34 38 39 30 32 36 36 # product number 
                cb 
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
            life_number = p[20:30]
            print("life number : " + life_number.decode('utf-8'))
            software_number = p[30:32]
            print("software number : " + software_number.decode('utf-8'))
            ai_number = p[32:34]
            print("ai number : " + ai_number.decode('utf-8'))
            product_number = p[34:43]
            print('product number : ' + product_number.decode('utf-8'))

        elif payload == bytes(b'\x0B\x03'):
            """
                32 05 0b 03 3f                
                32 1c 
                a0 # status
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
                print("cruise control mode : unknown")

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
            print("kickdown : " + "yes" if kickdown > 0 else "No")
            vehicle_in_curve = gear & 0x8
            print("vehicle in curve : " + "yes" if vehicle_in_curve > 0 else "No")

        elif payload == bytes(b'\x04\x01'):
            error_code_count = p[1]
            print("error code count : " + str(error_code_count))
            if len(p) >= 5:
                error_code = p[2:4]
                did = p[2]
                fid = p[3]
                freq = p[4]
                print("(" + str(freg) + ") " + "error code 0 : " + error_code.decode('utf-8') + " : " + error_description[did] + " : " + error_flags[fid])
            if len(p) >= 24:
                error_code = p[21:23]
                did = p[21]
                fid = p[22]
                freq = p[23]
                print("(" + str(freg) + ") " + "error code 1 : " + error_code.decode('utf-8') + " : " + error_description[did] + " : " + error_flags[fid])
            if len(p) >= 43:
                error_code = p[40:42]
                did = p[40]
                fid = p[41]
                freq = p[42]
                print("(" + str(freg) + ") " + "error code 2 : " + error_code.decode('utf-8') + " : " + error_description[did] + " : " + error_flags[fid])
            if len(p) >= 62:
                error_code = p[59:61]
                did = p[59]
                fid = p[60]
                freq = p[61]
                print("(" + str(freg) + ") " + "error code 3 : " + error_code.decode('utf-8') + " : " + error_description[did] + " : " + error_flags[fid])
            if len(p) >= 81:
                error_code = p[78:80]
                did = p[78]
                fid = p[79]
                freq = p[80]
                print("(" + str(freg) + ") " + "error code 4 : " + error_code.decode('utf-8') + " : " + error_description[did] + " : " + error_flags[fid])
         
        else:
            print("Unknown payload")

"""
p = b'\x62\x40\x00\x00\xf6\x7e\x47\x81\x5a\x00\x0b\x19\x46\x00\x83\x01\x81\x5d\x7b\x0c\x07\x00\xdd\x1f\x9f\x91\x00\x94\x57\x00\x1d\x00\x1d\x00\x1d\x00\x1b\x00\x22\x00\x1c\x00\x1b\x00\x1d\x0b'

r = struct.unpack('>h'*1, p[29:31])
print("Knock sensor Cyl. 1 : " + str(r[0] * 0.019531) + " V")
r = struct.unpack('>h'*1, p[31:33])
print("Knock sensor Cyl. 2 : " + str(r[0] * 0.019531) + " V")
r = struct.unpack('>h'*1, p[33:35])
print("Knock sensor Cyl. 3 : " + str(r[0] * 0.019531) + " V")
r = struct.unpack('>h'*1, p[35:37])
print("Knock sensor Cyl. 4 : " + str(r[0] * 0.019531) + " V")
r = struct.unpack('>h'*1, p[37:39])
print("Knock sensor Cyl. 5 : " + str(r[0] * 0.019531) + " V")
r = struct.unpack('>h'*1, p[39:41])
print("Knock sensor Cyl. 6 : " + str(r[0] * 0.019531) + " V")
r = struct.unpack('>h'*1, p[41:43])
print("Knock sensor Cyl. 7 : " + str(r[0] * 0.019531) + " V")
r = struct.unpack('>h'*1, p[43:45])
print("Knock sensor Cyl. 8 : " + str(r[0] * 0.019531) + " V")

r = struct.unpack('>h'*8, p[29:46])
for i in range(1):
    print("Knock sensor Cyl. " + str(i+1) + " : " + str(r[i] * 0.019531) + " V")
"""
"""
p = b'\x62\x40\x03\xff\x6f\xff\xac\xff\xe8\xff\xb1\x00\x32\x00\x7f\x00\x53\x00\x7e\x01\x00\xdf\x01\x66\x21'

r = struct.unpack('>h'*1, p[3:5])
print("Roughness Cyl. 1 " + str(r[0] * 0.0027756) + " sec-1")
r = struct.unpack('>h'*1, p[5:7])
print("Roughness Cyl. 2 " + str(r[0] * 0.0027756) + " sec-1")
r = struct.unpack('>h'*1, p[7:9])
print("Roughness Cyl. 3 " + str(r[0] * 0.0027756) + " sec-1")
r = struct.unpack('>h'*1, p[9:11])
print("Roughness Cyl. 4 " + str(r[0] * 0.0027756) + " sec-1")
r = struct.unpack('>h'*1, p[11:13])
print("Roughness Cyl. 5 " + str(r[0] * 0.0027756) + " sec-1")
r = struct.unpack('>h'*1, p[13:15])
print("Roughness Cyl. 6 " + str(r[0] * 0.0027756) + " sec-1")
r = struct.unpack('>h'*1, p[15:17])
print("Roughness Cyl. 7 " + str(r[0] * 0.0027756) + " sec-1")
r = struct.unpack('>h'*1, p[17:19])
print("Roughness Cyl. 8 " + str(r[0] * 0.0027756) + " sec-1")
"""
"""
b = 16
print("True" if b > 0 else "False")
"""
egs = ZF5HP24()
egs.run()

dme = ME72()
dme.run()
"""
ds2 = DS2()
while 1:
    ds2.sniffer()
"""