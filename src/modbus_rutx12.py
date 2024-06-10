#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from modbus.msg import ModbusData
import subprocess
import os
import time

def switch_to_first_modem():
    command = f"modbus write -D 192.168.1.1 %MW328  13101 12544"
    result = subprocess.run(command, shell=True, capture_output=True, text=True, timeout=30)
    if result.returncode == 0:
        rospy.loginfo("Switched to first modem successfully")
    else:
        rospy.logerr("Failed to switch to first modem")

def switch_to_second_modem():
    command = f"modbus write -D 192.168.1.1 %MW328  12589 12590 12800"
    result = subprocess.run(command, shell=True, capture_output=True, text=True, timeout=30)
    if result.returncode == 0:
        rospy.loginfo("Switched to second modem successfully")
    else:
        rospy.logerr("Failed to switch to second modem")

def classify_rssi(rssi, network_type):
    if network_type == "LTE":   # 4G
        if rssi > -65:
            return "Excellent"
        elif -65 >= rssi > -75:
            return "Good"
        elif -75 >= rssi > -85:
            return "Fair"
        elif -85 >= rssi > -95:
            return "Poor"
        else:
            return "No signal"
    else:  # 2G and 3G
        if rssi >= -70:
            return "Excellent"
        elif -70 > rssi >= -85:
            return "Good"
        elif -85 > rssi >= -100:
            return "Fair"
        elif -100 > rssi >= -110:
            return "Poor"
        else:
            return "No signal"

def read_modem_data(modem_number):
    modem_data = ModbusData()

    # Read and publish Modem ID
    modem_id_data = read_modbus_data(328, 329, 8)
    if modem_id_data:
        byte_array = bytearray()
        for data in modem_id_data:
            byte_array.append(data >> 8)
            byte_array.append(data & 0xFF)

        byte_array = byte_array.decode('utf-8', errors='replace')
        if modem_number == 1:
            modem_data.modem_id_1 = byte_array.replace('\x00', '')
        else:
            modem_data.modem_id_2 = byte_array.replace('\x00', '')

    # Read and publish Mobile data received today
    mobile_data_received_today_data = read_modbus_data(185, 186, 2)
    if mobile_data_received_today_data:
        mobile_data_received_today = (mobile_data_received_today_data[0] << 16) + mobile_data_received_today_data[1]
        if modem_number == 1:
            modem_data.mobile_data_received_today_1 = mobile_data_received_today / (1024**2)
        else:
            modem_data.mobile_data_received_today_2 = mobile_data_received_today / (1024**2)

    # Read and publish Mobile data sent today
    mobile_data_sent_today_data = read_modbus_data(187, 188, 2)
    if mobile_data_sent_today_data:
        mobile_data_sent_today = (mobile_data_sent_today_data[0] << 16) + mobile_data_sent_today_data[1]
        if modem_number == 1:
            modem_data.mobile_data_sent_today_1 = mobile_data_sent_today / (1024**2)
        else:
            modem_data.mobile_data_sent_today_2 = mobile_data_sent_today / (1024**2)

    # Read and publish Network type
    network_type_data = read_modbus_data(119, 120, 16)
    if network_type_data:
        byte_array = bytearray()
        for data in network_type_data:
            byte_array.append(data >> 8)
            byte_array.append(data & 0xFF)

        byte_array = byte_array.decode('utf-8', errors='replace')
        if modem_number == 1:
            modem_data.network_type_1 = byte_array.replace('\x00', '')
        else:
            modem_data.network_type_2 = byte_array.replace('\x00', '')

    # Read and publish Router serial number
    operator_name_data = read_modbus_data(23, 24, 16)
    if operator_name_data:
        byte_array = bytearray()
        for data in operator_name_data:
            byte_array.append(data >> 8)
            byte_array.append(data & 0xFF)

        byte_array = byte_array.decode('utf-8', errors='replace')
        if modem_number == 1:
            modem_data.operator_name_1 = byte_array.replace('\x00', '')
        else:
            modem_data.operator_name_2 = byte_array.replace('\x00', '')

    # Read and publish Mobile signal strength (RSSI in dBm)
    rssi_data = read_modbus_data(3, 4, 2)
    if rssi_data:
        if modem_number == 1:
            rssi_1 = rssi_data[1]
            if rssi_data[1] >= 32768:
                rssi_1 = (rssi_data[1] - 65536)
            modem_data.rssi_1 = rssi_1

        else:
            rssi_2 = rssi_data[1]
            if rssi_data[1] >= 32768:
                rssi_2 = (rssi_data[1] - 65536)
            modem_data.rssi_2 = rssi_2

    return modem_data

def read_modbus_data(register_address, register_number, num_registers):
    try:
        # Execute modbus-cli command to read data from Modbus
        command = f"modbus read -w -p 502 192.168.1.1 %MW{register_address} {num_registers}"
        result = subprocess.run(command, shell=True, capture_output=True, text=True, timeout=30)
        
        # Check if command was successful
        if result.returncode == 0:
            # Parse output and return data
            data = [int(value.split()[1]) for value in result.stdout.strip().split('\n')]
            return data
        '''else:
            rospy.logerr("Error reading Modbus data")
            return None'''
    except Exception as e:
        rospy.logerr(f"Error executing modbus-cli command: {e}")
        return None

def modbus_node():
    rospy.init_node('modbus_node', anonymous=True)
    pub = rospy.Publisher('modbus_data', ModbusData, queue_size=10)
    rate = rospy.Rate(0.1) 

    while not rospy.is_shutdown():
        msg = ModbusData()
        msg.header = Header(stamp=rospy.Time.now())

        switch_to_first_modem()
        modem_data_1 = read_modem_data(1)
        if modem_data_1:
            msg.modem_id_1 = modem_data_1.modem_id_1
            msg.mobile_data_received_today_1 = modem_data_1.mobile_data_received_today_1
            msg.mobile_data_sent_today_1 = modem_data_1.mobile_data_sent_today_1
            msg.network_type_1 = modem_data_1.network_type_1
            msg.operator_name_1 = modem_data_1.operator_name_1
            msg.rssi_1 = modem_data_1.rssi_1
            msg.rssi_classification_1 = classify_rssi(modem_data_1.rssi_1, modem_data_1.network_type_1)

        time.sleep(0.1)

        switch_to_second_modem()
        modem_data_2 = read_modem_data(2)
        if modem_data_2:
            msg.modem_id_2 = modem_data_2.modem_id_2
            msg.mobile_data_received_today_2 = modem_data_2.mobile_data_received_today_2
            msg.mobile_data_sent_today_2 = modem_data_2.mobile_data_sent_today_2
            msg.network_type_2 = modem_data_2.network_type_2
            msg.operator_name_2 = modem_data_2.operator_name_2
            msg.rssi_2 = modem_data_2.rssi_2
            msg.rssi_classification_2 = classify_rssi(modem_data_2.rssi_2, modem_data_2.network_type_2)

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        modbus_node()
    except rospy.ROSInterruptException:
        pass
