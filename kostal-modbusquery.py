#!/usr/bin/env python
# -*- coding: utf-8 -*-

#  kostal_modbusquery - Read only query of the Kostal Plenticore Inverters using TCP/IP modbus protocol
#  Copyright (C) 2018  Kilian Knoll
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
#
#  Please note that any incorrect or careless usage of this module as well as errors in the implementation can damage your Inverter!
#  Therefore, the author does not provide any guarantee or warranty concerning to correctness, functionality or performance and does not accept any liability for damage caused by this module, examples or mentioned information.
#  Thus, use it at your own risk!
#
#
#  Purpose:
#           Query values from Kostal inverter
#           Used with Kostal Plenticore Plus 10
#  Based on the documentation provided by Kostal:
#           https://www.kostal-solar-electric.com/en-gb/download/download#PLENTICORE%20plus/PLENTICORE%20plus%204.2/Worldwide/Interfaces%20protocols/
#
# Requires pymodbus
# Tested with:
#           python 3.5   / 3.6
#           pymodbus 2.10 /2.20
# Please change the IP address of your Inverter (e.g. 192.168.178.41 and Port (default 1502) to suite your environment - see below)
#
# Update Feb 23 2018
# Corrected copy paste error to properly reflect Voltage DC1 (was Register 100 - needs to be Register 266)
#
#---------------------------------------------------------------------------------------
# Update June 11 2019
# Enhanced with additional registers -see:
# PLENTICORE plus - PIKO IQ - UI: 01.07.03325 FW: 01.31 - Software Update
# BYD Firmware 3.012R
#
# Kostal claims the following changes / updates to registers:
#104 U32
#110 FLOAT
#280 FLOAT
#517 String
#525 U32
#527 U32
#529 U32
#586 U32
#588 U16
# Comments:
# Something fishy going on with register 212 (Battery State) - disappeared from documentation without it being mentioned in Kostal´s Readme
# Having issues with the U32 registers - not sure if Registers 535,527,529,577,588 read the correct values
#------------------------------------------------------------------------------------------
# Update March 3 2020
# Enhanced to clean erratic reads greater than 32767 from register 575
# Added Register 56 (Inverter Status)
#
# Update July 11 2020
# Added Option to publish to mqtt
#
# Update July 24 2020
# Corrected error in ReadS16 (Registers 575 and 582 should now read proper negative values)
#
#
# Update Dec 5th 2020
# Added the following Read-only Registers
#(based on Kostal Modbus interface description from Sep 17 2020)
# For Firmware versions 1.4.4 and up
#1046 Total DC charge energy (DC-side to battery) Wh R32 2 RO 0x03 0x418
#1048 Total DC discharge energy (DC-side from battery) Wh R32 2 RO 0x03 0x41A
#1050 Total AC charge energy (AC-side to battery) Wh R32 2 RO 0x03 0x41C
#1052 Total AC discharge energy (battery to grid) Wh R32 2 RO 0x03 0x41E
#1054 Total AC charge energy (grid to battery) Wh R32 2 RO 0x03 0x420
#1056 Total DC PV energy (sum of all PV inputs) Wh R32 2 RO 0x03 0x422
#1058 Total DC energy from PV1 Wh R32 2 RO 0x03 0x424
#1060 Total DC energy from PV2 Wh R32 2 RO 0x03 0x426
#1062 Total DC energy from PV3 Wh R32 2 RO 0x03 0x428
#1064 Total energy AC-side to grid Wh R32 2 RO 0x03 0x42A
#1066 Total DC power (sum of all PV inputs) W R32 2 RO 0x03
#
##1024 Battery charge power (AC) setpoint Note1 W S16 1 WO 0x06
#Added the Following Registers as  "Read-only" readout:
#1025 Power Scale Factor Note2 - S16 1 RO 0x03
#1026 Battery charge power (AC) setpoint, absolute
#1028 Battery charge current (DC) setpoint, relative Note 1,3 % R32 2 RW 0x03/0x10
#1030 Battery charge power (AC) setpoint, relativeNote 1,3% R32 2 RW 0x03/0x10
#1032 Battery charge current (DC) setpoint, absoluteNote 1A R32 2 RW 0x03/0x10
#1034 Battery charge power (DC) setpoint, absoluteNote 1W R32 2 RW 0x03/0x10
#1036 Battery charge power (DC) setpoint, relativeNote 1,3% R32 2 RW 0x03/0x10
#1038 Battery max. charge power limit, absolute W R32 2 RW 0x03/0x10
#1040 Battery max. discharge power limit, absolute W R32 2 RW 0x03/0x10
#1042 Minimum SOC % R32 2 RW 0x03/0x10
#1044 Maximum SOC % R32 2 RW 0x03/0x10
#1068 Battery work capacity Wh R32 2 RO 0x03
#1070 Battery serial number - U32 2 RO 0x03
#1076 Maximum charge power limit (read-out frombattery)W R32 2 RO 0x03
#1078 Maximum discharge power limit (read-out frombattery)W R32 2 RO 0x03
#1080 Battery management mode Note4 - U8 1 RW 0x03/0x06
#1082 Installed sensor type Note 5 - U8 1 RO 0x03
#
#Updated / Created String Read Routines ReadStr8 and ReadStr32
#
# Updated June 4 2021
# Merged Pullrequest from https://github.com/carichte/kostal-modbusquery
# Added commandline Option to write R32 Registers (for battery charge/discharge manipulation)
#
# Update June 12 2021
# Fixed programming error when function Kostalquery.run is called mutliple times





import pymodbus
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder
from pprint import pprint
import time
import collections
import argparse


class kostal_modbusquery:
    def __init__(self):
        #Change the IP address and port to suite your environment:
        self.inverter_ip="192.168.178.41"
        self.inverter_port="1502"
        #No more changes required beyond this point
        self.KostalRegister = []


        self.Adr = collections.OrderedDict()

        self.Adr[6]   = [6,"Inverter article number", "Strg8"]
        self.Adr[46]  = [46,"Software-Version IO-Controller (IOC)", "Strg8"]
        self.Adr[56]  = [56,"Inverter State", "U16"]
        self.Adr[100] = [100,"Total DC power", "Float"]

        self.Adr[104] = [104,"State of energy manager","Float"]
        self.Adr[106] = [106,"Home own consumption from battery", "Float"]

        self.Adr[108] = [108,"Home own consumption from grid", "Float"]
        self.Adr[110] = [110,"Total home consumption Battery", "Float"]

        self.Adr[112] = [112,"Total home consumption Grid", "Float"]
        self.Adr[114] = [114,"Total home consumption PV", "Float"]
        self.Adr[116] = [116,"Home own consumption from PV", "Float"]
        self.Adr[118] = [118,"Total home consumption", "Float"]
        self.Adr[120] = [120,"Isolation resistance", "Float"]
        self.Adr[122] = [122,"Power limit from EVU", "Float"]
        self.Adr[124] = [124,"Total home consumption rate", "Float"]
        self.Adr[144] = [144,"Worktime", "Float"]

        self.Adr[150] = [150,"Actual cos phi", "Float"]
        self.Adr[152] = [152,"Grid frequency", "Float"]
        self.Adr[154] = [154,"Current Phase 1", "Float"]
        self.Adr[156] = [156,"Active power Phase 1", "Float"]
        self.Adr[158] = [158,"Voltage Phase 1", "Float"]
        self.Adr[160] = [160,"Current Phase 2", "Float"]
        self.Adr[162] = [162,"Active power Phase 2", "Float"]
        self.Adr[164] = [164,"Voltage Phase 2", "Float"]
        self.Adr[166] = [166,"Current Phase 3", "Float"]

        self.Adr[168] = [168,"Active power Phase 3", "Float"]
        self.Adr[170] = [170,"Voltage Phase 3", "Float"]
        self.Adr[172] = [172,"Total AC active power", "Float"]
        self.Adr[174] = [174,"Total AC reactive power", "Float"]
        self.Adr[178] = [178,"Total AC apparent power", "Float"]
        self.Adr[190] = [190,"Battery charge current", "Float"]
        self.Adr[194] = [194,"Number of battery cycles", "Float"]
        self.Adr[200] = [200,"Actual battery charge -minus or discharge -plus current", "Float"]
        self.Adr[202] = [202,"PSSB fuse state", "Float"]
        self.Adr[208] = [208,"Battery ready flag", "Float"]
        self.Adr[210] = [210,"Act. state of charge", "Float"]
        # self.Adr[212] = [212,"Battery state", "Float"]
        self.Adr[214] = [214,"Battery temperature", "Float"]
        self.Adr[216] = [216,"Battery voltage", "Float"]
        self.Adr[218] = [218,"Cos phi (powermeter)", "Float"]
        self.Adr[220] = [220,"Frequency (powermeter)", "Float"]
        self.Adr[222] = [222,"Current phase 1 (powermeter)", "Float"]
        self.Adr[224] = [224,"Active power phase 1 (powermeter)", "Float"]
        self.Adr[226] = [226,"Reactive power phase 1 (powermeter)", "Float"]
        self.Adr[228] = [228,"Apparent power phase 1 (powermeter)", "Float"]
        self.Adr[230] = [230,"Voltage phase 1 (powermeter)", "Float"]
        self.Adr[232] = [232,"Current phase 2 (powermeter)", "Float"]
        self.Adr[234] = [234,"Active power phase 2 (powermeter)", "Float"]
        self.Adr[236] = [236,"Reactive power phase 2 (powermeter)", "Float"]
        self.Adr[238] = [238,"Apparent power phase 2 (powermeter)", "Float"]
        self.Adr[240] = [240,"Voltage phase 2 (powermeter)", "Float"]
        self.Adr[242] = [242,"Current phase 3 (powermeter)", "Float"]
        self.Adr[244] = [244,"Active power phase 3 (powermeter)", "Float"]
        self.Adr[246] = [246,"Reactive power phase 3 (powermeter)", "Float"]
        self.Adr[248] = [248,"Apparent power phase 3 (powermeter)", "Float"]
        self.Adr[250] = [250,"Voltage phase 3 (powermeter)", "Float"]
        self.Adr[252] = [252,"Total active power (powermeter)", "Float"]
        self.Adr[254] = [254,"Total reactive power (powermeter)", "Float"]
        self.Adr[256] = [256,"Total apparent power (powermeter)", "Float"]
        self.Adr[258] = [258,"Current DC1", "Float"]
        self.Adr[260] = [260,"Power DC1", "Float"]
        self.Adr[266] = [266,"Voltage DC1", "Float"]
        self.Adr[268] = [268,"Current DC2", "Float"]
        self.Adr[270] = [270,"Power DC2", "Float"]
        self.Adr[276] = [276,"Voltage DC2", "Float"]
        self.Adr[278] = [278,"Current DC3", "Float"]
        self.Adr[280] = [280,"Power DC3", "Float"]
        self.Adr[286] = [286,"Voltage DC3", "Float"]
        self.Adr[320] = [320,"Total yield", "Float"]
        self.Adr[322] = [322,"Daily yield", "Float"]
        self.Adr[324] = [324,"Yearly yield", "Float"]
        self.Adr[326] = [326,"Monthly yield", "Float"]
        self.Adr[512] = [512,"Battery Gross Capacity", "U32"]
        self.Adr[514] = [514,"Battery actual SOC", "U16"]
        self.Adr[515] = [515,"Firmware Maincontroller (MC)", "U32"]
        self.Adr[517] = [517,"Battery Manufacturer", "Strg8"]
        self.Adr[525] = [525,"Battery Model ID", "U32"]
        self.Adr[527] = [527,"Battery Serial Number", "U32"]
        self.Adr[529] = [529,"Battery Operation mode", "U32"]
        self.Adr[531] = [531,"Inverter Max Power", "Float"]
        self.Adr[575] = [575,"Inverter Generation Power (actual)", "S16"]
        self.Adr[577] = [577,"Generation Energy", "U32"]
        # self.Adr[578] = [578,"Total energy", "U32"]
        self.Adr[580] = [580,"Battery Net Capacity", "U32"]
        self.Adr[582] = [582,"Actual battery charge-discharge power", "S16"]
        self.Adr[586] = [586,"Battery Firmware", "U32"]
        self.Adr[588] = [588,"Battery Type", "U16"]
        self.Adr[768] = [768,"Productname", "Strg32"]
        self.Adr[800] = [800,"Power Class", "Strg32"]
        self.Adr[1024] = [1024,"Battery charge power (AC) setpoint", "S16"]
        self.Adr[1025] = [1025,"Power Scale Factor", "S16"]
        self.Adr[1026] = [1026,"Battery charge power (AC) setpoint, absolute", "R32"]
        self.Adr[1028] = [1028,"Battery charge current (DC) setpoint, relative", "R32"]
        self.Adr[1030] = [1030,"Battery charge power (AC) setpoint, relative", "R32"]
        self.Adr[1032] = [1032,"Battery charge current (DC) setpoint, absolute", "R32"]
        self.Adr[1034] = [1034,"Battery charge power (DC) setpoint, absolute", "R32"]
        self.Adr[1036] = [1036,"Battery charge power (DC) setpoint, relative", "R32"]
        self.Adr[1038] = [1038,"Battery max charge power limit, absolute", "U32"]
        self.Adr[1040] = [1040,"Battery max discharge power limit, absolute", "U32"]
        self.Adr[1042] = [1042,"Minimum SOC", "U32"]
        self.Adr[1044] = [1044,"Maximum SOC", "U32"]
        self.Adr[1046] = [1046,"Total DC charge energy (DC-side to battery)", "R32"]
        self.Adr[1048] = [1048,"Total DC discharge energy (DC-side from battery)", "R32"]
        self.Adr[1050] = [1050,"Total AC charge energy (AC-side to battery)", "R32"]
        self.Adr[1052] = [1052,"Total AC discharge energy (Battery to grid)", "R32"]
        self.Adr[1054] = [1054,"Total AC charge energy (grid to battery)", "R32"]
        self.Adr[1056] = [1056,"Total DC PV energy (sum of all PV inputs)", "R32"]
        self.Adr[1058] = [1058,"Total DC energy from PV1", "R32"]
        self.Adr[1060] = [1060,"Total DC energy from PV2", "R32"]
        self.Adr[1062] = [1062,"Total DC energy from PV3", "R32"]
        self.Adr[1064] = [1064,"Total energy AC-side to grid", "R32"]
        self.Adr[1066] = [1066,"Total DC power (sum of all PV inputs)", "R32"]
        self.Adr[1068] = [1068,"Battery work capacity", "R32"]
        self.Adr[1070] = [1070,"Battery serial number", "U32"]
        self.Adr[1076] = [1076,"Maximum charge power limit (readout from battery)", "R32"]
        self.Adr[1078] = [1078,"Maximum discharge power limit (readout from battery)", "R32"]
        self.Adr[1080] = [1080,"Battery management mode", "U8"]
        self.Adr[1082] = [1082,"Installed sensor type", "U8"]


        self.Qty = dict()
        for key in self.Adr:
            self.Adr[key].append(None) # initial value
            self.Qty[self.Adr[key][1]] = self.Adr[key] # to search by name



    #-----------------------------------------
    # Routine to read a string from one address with 8 registers
    def ReadStr8(self,myadr_dec):
        r1=self.client.read_holding_registers(myadr_dec,8,unit=71)
        STRG8Register = BinaryPayloadDecoder.fromRegisters(r1.registers, byteorder=Endian.Big)
        result_STRG8Register =STRG8Register.decode_string(8)
        result_STRG8Register = bytes(filter(None,result_STRG8Register))    #Get rid of the "\X00"s
        return(result_STRG8Register)
    #-----------------------------------------
    # Routine to read a string from one address with 32 registers
    def ReadStr32(self,myadr_dec):
        r1=self.client.read_holding_registers(myadr_dec,32,unit=71)
        STRG32Register = BinaryPayloadDecoder.fromRegisters(r1.registers, byteorder=Endian.Big)
        result_STRG32Register =STRG32Register.decode_string(32)
        result_STRG32Register =bytes(filter(None,result_STRG32Register))    #Get rid of the "\X00"s
        return(result_STRG32Register)
    #-----------------------------------------
    # Routine to read a Float from one address with 2 registers
    def ReadFloat(self,myadr_dec):
        r1=self.client.read_holding_registers(myadr_dec,2,unit=71)
        FloatRegister = BinaryPayloadDecoder.fromRegisters(r1.registers, byteorder=Endian.Big, wordorder=Endian.Little)
        result_FloatRegister =round(FloatRegister.decode_32bit_float(),2)
        return(result_FloatRegister)
    #-----------------------------------------
    # Routine to read a U16 from one address with 1 register
    def ReadU16_1(self,myadr_dec):
        r1=self.client.read_holding_registers(myadr_dec,1,unit=71)
        U16register = BinaryPayloadDecoder.fromRegisters(r1.registers, byteorder=Endian.Big, wordorder=Endian.Little)
        result_U16register = U16register.decode_16bit_uint()
        return(result_U16register)
    #-----------------------------------------
    # Routine to read a U16 from one address with 2 registers
    def ReadU16_2(self,myadr_dec):
        r1=self.client.read_holding_registers(myadr_dec,2,unit=71)
        U16register = BinaryPayloadDecoder.fromRegisters(r1.registers, byteorder=Endian.Big, wordorder=Endian.Little)
        result_U16register = U16register.decode_16bit_uint()
        return(result_U16register)
    #-----------------------------------------
    # Routine to read a U32 from one address with 2 registers
    def ReadU32(self,myadr_dec):
        r1=self.client.read_holding_registers(myadr_dec,2,unit=71)
        #print ("r1 ", rl.registers)
        U32register = BinaryPayloadDecoder.fromRegisters(r1.registers, byteorder=Endian.Big, wordorder=Endian.Little)
        #print ("U32register is", U32register)
        #result_U32register = U32register.decode_32bit_float()
        result_U32register = U32register.decode_32bit_uint()
        return(result_U32register)
    #-----------------------------------------
    def ReadU32new(self,myadr_dec):
        #print ("I am in ReadU32new with", myadr_dec)
        r1=self.client.read_holding_registers(myadr_dec,2,unit=71)
        U32register = BinaryPayloadDecoder.fromRegisters(r1.registers, byteorder=Endian.Big, wordorder=Endian.Little)
        result_U32register = U32register.decode_32bit_uint()
        #result_U32register = U32register.decode_32bit_float()
        #print ("Here is what I got from ReadU32new", result_U32register)
        return(result_U32register)
    #-----------------------------------------
    # Routine to read a S16 from one address with 1 registers
    def ReadS16(self,myadr_dec):
        r1=self.client.read_holding_registers(myadr_dec,1,unit=71)
        S16register = BinaryPayloadDecoder.fromRegisters(r1.registers, byteorder=Endian.Big, wordorder=Endian.Little)
        result_S16register = S16register.decode_16bit_int()
        return(result_S16register)
    #-----------------------------------------
    # Routine to read a U8 from one address with 1 registers
    def ReadU8(self,myadr_dec):
        r1=self.client.read_holding_registers(myadr_dec,1,unit=71)
        U8register = BinaryPayloadDecoder.fromRegisters(r1.registers, byteorder=Endian.Big, wordorder=Endian.Little)
        #result_U8register = U8register.decode_8bit_uint()
        result_U8register = U8register.decode_16bit_uint()
        return(result_U8register)

    def WriteR32(self,myadr_dec,value):

        myreadregister= self.client.read_holding_registers(myadr_dec,2,unit=71)
        myreadregister = BinaryPayloadDecoder.fromRegisters(myreadregister.registers, byteorder=Endian.Big, wordorder=Endian.Little)
        myreadregister =round(myreadregister.decode_32bit_float(),2)
        print ("I read the value before setting it - value was ", myreadregister)
        mybuilder = BinaryPayloadBuilder(byteorder=Endian.Big,wordorder=Endian.Little)
        mybuilder.add_32bit_float(value)
        mypayload = mybuilder.build()
        mywriteregister=self.client.write_registers(myadr_dec,mypayload,skip_encode=True, unit=71)
        """
        print ("From  subroutine WriteS16 - In theory .... - I should get the value back that we pushed ", value ,"----", myreadregister)
        print("Register I wrote",mywriteregister)
        """
        return(mywriteregister)  

    def run(self):

        try:
            self.client = ModbusTcpClient(self.inverter_ip,
                                          port=self.inverter_port)
            self.client.connect()

            for key in self.Adr:
                dtype = self.Adr[key][2]
                if dtype == "Strg8":
                    reader = self.ReadStr8
                elif dtype == "U16":
                    reader = self.ReadU16_1
                elif dtype == "Float":
                    reader = self.ReadFloat
                elif dtype == "U8":
                    reader = self.ReadU8
                elif dtype == "U32":
                    reader = self.ReadU32new
                elif dtype == "S16":
                    reader = self.ReadS16
                elif dtype == "Strg32":
                    reader = self.ReadStr32
                elif dtype == "R32":
                    reader = self.ReadFloat
                else:
                    raise ValueError("Data type not known: %s"%dtype)

                val = reader(key)
                self.Adr[key][3] = val

            self.client.close()

            if self.Adr[575][3] > 32766: #Sometimes we hit the max value of 32767 - which implies a zero value
                self.Adr[575][3] = 0



        except Exception as ex:
            print ("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
            print ("XXX- Hit the following error :From subroutine kostal_modbusquery:", ex)
            print ("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")


if __name__ == "__main__":
    try:
        Kostalquery = kostal_modbusquery()
        print ("Starting QUERY .......... ")

        ts= time.time()
        Kostalquery.run()
        te = time.time()
        print ("Elapsed time is ", te-ts)
    
        for item in Kostalquery.Adr.values():
            if not item[3] is None:
                print("Register", item[0], item[1], item[3])

    
    
        print ("Done...")
        #pprint(Kostalquery.KostalRegister)
        ##########################################
        print ("----------------------------------")
        print ("Doing some Calculations of the received information:")
    
        LeftSidePowerGeneration= round(Kostalquery.Qty['Power DC1'][3] + Kostalquery.Qty['Power DC2'][3], 1)
        print ("Left Side Raw Power Generation of Panels :", LeftSidePowerGeneration)
        BatteryCharge = round(Kostalquery.Qty['Battery voltage'][3] * Kostalquery.Qty['Actual battery charge -minus or discharge -plus current'][3], 1)
        print ("BatteryCharge (-) / Discharge(+) is      :", BatteryCharge)
        TotalHomeconsumption =round((Kostalquery.Qty['Home own consumption from battery'][3]
                                + Kostalquery.Qty['Home own consumption from grid'][3]
                                + Kostalquery.Qty['Home own consumption from PV'][3]), 1)
        PowertoGrid = round(Kostalquery.Qty['Inverter Generation Power (actual)'][3] - TotalHomeconsumption,1)
        print ("Powerfromgrid (-) /To Grid (+) is        :", PowertoGrid)
        print ("Total current Home consumption is        :", TotalHomeconsumption)
        print ("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")

        
        my_parser = argparse.ArgumentParser()
        my_parser.add_argument('--nargs', nargs='+',
                               help='Commandline options allow you to also set parameters via the Modbus TCP connection: python kostal-modbusquery.py  -Batcharge 475')
        my_parser.add_argument('-BatCharge',
                            action='store',
                            type = float,
                            #choices=range(1,999000),
                            help='Sets the Battery Charge/Discharge [W] . Allowed values: Positve to discharge Battery, negative to charge Battery; Please note: This option only works when you have set your Battery management to External via protocol (Modbus TCP)') 

        args = vars(my_parser.parse_args())  
        CommandlineInput = 0
        for i in args:
            if (str(args[i]) != 'None'):
                #print ("Found", i , args[i])
                CommandlineInput = 1        
                #print ("CommandlineInput is ", CommandlineInput , " So will obeye what was specified on the commandline")
        #xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
        # COMMANDLINE ONLY
        #xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
        if (CommandlineInput == 1):
            print ("Now Setting Parameters :")
            if (str(args['BatCharge']) != 'None'):
                print ("Setting BatCharge to : ",args['BatCharge'] ," W") 
                Upperlimit = Kostalquery.WriteR32(1034,args['BatCharge'])
        
                            
        MQTT_Active = 0

        if MQTT_Active == 1:
            try:
                import paho.mqtt.client as mqtt
                broker_address="192.168.178.39"                                                 #IP-Adress of the mqtt broker we subscribe to
                #Publish to mqtt start
                print ("Now publishing data to MQTT Broker with IP Adress : ", broker_address)
    
                modbusmqttclient = mqtt.Client("MyModbusMQTTClient")                            #create new instance
                modbusmqttclient.connect(broker_address)                                        #connect to broker
                params = list(Kostalquery.Qty.keys())                                           #We then need to update our params to include the Numpulses key
    
                if len(params) > 1:
                    for p in sorted(params):
                        print ("entering Kostal mqtt publish")
                        print("{:{width}}: {}".format(p, Kostalquery.Qty[p][3], width=(max(params, key=len))))
                        TOPIC = ("Haus/Kostal/"+p)
                        modbusmqttclient.publish(TOPIC,KostalVal[p])
                elif len(params) == 1:
                    print(val[params[0]])
                    print ("Nothing received from Kostal... ? ")
            except Exception as ErrorMQTT:
                print ("Error Kostal MQTT publish", ErrorMQTT)
                
    except Exception as Badmain:
        print ("Ran into error executing Main kostal-RESTAPI Routine :", Badmain)                


