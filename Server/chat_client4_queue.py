import websockets
import asyncio
import logging
import time
import numpy as np
from pymavlink import mavutil
from pymap3d import geodetic2ned

# WIP: This script should obtain MAVlink messages and send them via websockets.

model_state = None

# Initialize reference point position
location_origin = [42.82920374, -1.76202659, 400.0]

# Define the connection string
#connection_string = 'tcp:127.0.0.1:5762'
connection_string = '/dev/ttyUSB0'
# Create a MAVLink connection
#mav_connection = mavutil.mavlink_connection(connection_string)
mav_connection = mavutil.mavlink_connection(connection_string, baud=57600)

# Define the maximum size of the message queue
MAX_QUEUE_SIZE = 2

def request_message_interval(message_id, interval_ms):
    # Send command to set message interval
    mav_connection.mav.command_long_send(
        mav_connection.target_system, mav_connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,  # Confirmation
        message_id,  # The MAVLink message ID
        interval_ms * 1000,  # Interval in microseconds
        0, 0, 0, 0, 0  # Additional parameters (param3 to param7)
    )

async def send_model_state():
    # Connect to the websockets server
    async with websockets.connect("ws://127.0.0.1:7890", ping_timeout=None, ping_interval=5) as websocket:

        message_queue = []  # Initialize an empty list to act as a message queue

        while True:
            # Wait for a MAVLink message
            msg = mav_connection.recv_match()
            data_string = None
            # Extract relevant data based on message type
            '''
              MESSAGE_LENGTH: 8 VALUES + ID
              BY ID: 
                SYS_STATUS: 1
                GLOBAL_POSITION_INT: 2
                SCALED_IMU: 3
                HOME_POSITION: 4
                HEARTBEAT: 5
                GPS_RAW_INT: 6
                ATTITUDE: 7

            '''
            if msg is not None:
                if msg.get_type() == 'SYS_STATUS':
                    # Extract battery status information from the message
                    voltage = msg.voltage_battery / 1000.0  # Convert to volts
                    current = msg.current_battery / 100.0   # Convert to amperes

                    data = np.round([1, voltage, current, 0, 0, 0, 0, 0, 0], 4)
                    data_string = ','.join(map(str, data))

                elif msg.get_type() == 'GLOBAL_POSITION_INT':
                    # Extract GPS position information from the message
                    lat = msg.lat / 1e7  # Latitude
                    lon = msg.lon / 1e7  # Longitude
                    alt = msg.alt / 1e3  # Altitude
                    vx = msg.vx / 100.0  # Ground speed in m/s
                    vy = msg.vy / 100.0  # Ground speed in m/s
                    vz = msg.vz / 100.0  # Vertical speed in m/s
                    gs = np.sqrt(vx ** 2 + vy ** 2 + vz ** 2)
                    hdg = msg.hdg / 100  # Heading in degrees
                    current_x, current_y, current_z = geodetic2ned(lat, lon, alt, location_origin[0], location_origin[1],
                                                                   location_origin[2], ell=None, deg=True)

                    data = np.round([2, lat, lon, alt, gs, hdg, current_x, current_y, current_z], 4)
                    data_string = ','.join(map(str, data))

                elif msg.get_type() == 'SCALED_IMU':
                    # Extract scaled IMU data from the message
                    xacc = msg.xacc  # Scaled acceleration in the X-axis
                    yacc = msg.yacc
                    zacc = msg.zacc
                    xgyro = msg.xgyro  # Scaled angular velocity in the X-axis
                    ygyro = msg.ygyro
                    zgyro = msg.zgyro

                    data = np.round([3, xacc, yacc, zacc, xgyro, ygyro, zgyro, 0, 0], 4)
                    data_string = ','.join(map(str, data))

                elif msg.get_type() == 'HOME_POSITION':
                    # Extract scaled IMU data from the message
                    home_lat = msg.latitude / 1e7  # Scaled latitude
                    home_lon = msg.longitude / 1e7
                    home_alt = msg.altitude / 1e3

                    home_north, home_east, home_down = geodetic2ned(home_lat, home_lon, home_alt, location_origin[0],
                                                                    location_origin[1], location_origin[2], ell=None, deg=True)
                    home_ned = np.array([4, home_north, home_east, home_down, 0, 0, 0, 0, 0])

                    data_string = ','.join(map(str, home_ned))

                elif msg.get_type() == 'HEARTBEAT':
                    # Extract the autopilot's current mode
                    custom_mode = msg.custom_mode

                    data = np.array([5, custom_mode, 0, 0, 0, 0, 0, 0, 0])
                    data_string = ','.join(map(str, data))

                elif msg.get_type() == 'GPS_RAW_INT':
                    # Extract HDOP and VDOP
                    hdop = msg.eph
                    vdop = msg.epv

                    data = np.array([6, hdop, vdop, 0, 0, 0, 0, 0, 0])

                    data_string = ','.join(map(str, data))

                elif msg.get_type() == 'ATTITUDE':
                    # Extract HDOP and VDOP
                    roll = msg.roll
                    pitch = msg.pitch
                    yaw = msg.yaw

                    data = np.array([7, roll, pitch, yaw, 0, 0, 0, 0, 0])

                    data_string = ','.join(map(str, data))

            if data_string is not None:
                if len(message_queue) >= MAX_QUEUE_SIZE:
                    # If the queue is full, remove the oldest message
                    message_queue.pop(0)
                message_queue.append(data_string)

            # Print the most recent message and remove it from the queue
            if message_queue:
                await websocket.send(str(message_queue.pop(0)))
                msg_recv = await websocket.recv()
                # print("Received: ", msg_recv)
                await asyncio.sleep(0.1)  # Adjust the frequency as needed

# Set the interval for requesting SYS_STATUS messages (e.g., every 2 seconds)
battery_interval_milliseconds = 1000
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, battery_interval_milliseconds)

# Set the interval for requesting GLOBAL_POSITION_INT messages (e.g., every 1 second)
gps_interval_milliseconds = 200
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, gps_interval_milliseconds)

# Set the interval for requesting SCALED_IMU messages (e.g., every 1 second)
imu_interval_milliseconds = 500
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU, imu_interval_milliseconds)

# Set the interval for requesting HOME_POSITION messages (e.g., every 1 second)
home_interval_milliseconds = 1000
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION, home_interval_milliseconds)

# Set the interval for requesting HEARTBEAT messages (e.g., every 1 second)
heartbeat_interval_milliseconds = 1000
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, heartbeat_interval_milliseconds)

# Set the interval for requesting GPS_RAW_INT messages (e.g., every 1 second)
hdop_interval_milliseconds = 1000
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, hdop_interval_milliseconds)

# Set the interval for requesting ATTITUDE messages (e.g., every 1 second)
attitude_interval_milliseconds = 200
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, attitude_interval_milliseconds)


if __name__ == '__main__':
    try:
        loop = asyncio.get_event_loop()
        loop.create_task(send_model_state())  # Create a task for the coroutine
        loop.run_forever()  # Run the event loop indefinitely
    except KeyboardInterrupt:
        print("Shutting down")
