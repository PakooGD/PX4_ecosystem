#!/usr/bin/env python3

"""
Stream ULog data over MAVLink and send via WebSocket.

Hardcoded version for UDP connection and WebSocket server.
"""

from __future__ import print_function
import sys, select, os
from timeit import default_timer as timer
os.environ['MAVLINK20'] = '1' # The commands require mavlink 2
import signal

import asyncio
import websockets
import base64
from hashlib import sha256
from Crypto.Cipher import AES
from Crypto.Util.Padding import pad, unpad
import json
import hmac

# Security configuration
SECRET_KEY = 'dsksupplytech'  # Должен быть bytes для HMAC
SECRET_MESSAGE_KEY = b'kchar-long-secret-key-1234567890'  # 32 байта для AES-256

try:
    from pymavlink import mavutil
except ImportError as e:
    print("Failed to import pymavlink: " + str(e))
    print("")
    print("You may need to install it with:")
    print("    pip3 install --user pymavlink")
    print("")
    sys.exit(1)

class LoggingCompleted(Exception):
    pass

class MavlinkLogStreaming():
    '''Streams log data via MAVLink and sends over WebSocket.'''
    def __init__(self, debug=0):
                
        # Hardcoded connection parameters
        self.mavlink_connection_string = 'udpin:127.0.0.1:14540'
        self.websocket_url = 'ws://localhost:8086'

        self.baudrate = 0
        self._debug = debug
        self.buf = ''  
        self.debug(f"Connecting with MAVLink to {self.mavlink_connection_string}...")
        self.mav = mavutil.mavlink_connection(self.mavlink_connection_string, autoreconnect=True)
        self.mav.wait_heartbeat()
        self.debug("HEARTBEAT OK\n")
        self.debug("MAVLink connection established\n")

        self.got_ulog_header = False
        self.got_header_section = False
        self.ulog_message = []  

        self.websocket = None

        self.start_time = timer()
        self.last_sequence = -1
        self.logging_started = False
        self.num_dropouts = 0
        self.target_component = 1
        self.got_sig_int = False

    def debug(self, s, level=1):
        '''write some debug text'''
        if self._debug >= level:
            print(s)

    def start_log(self):
        self.mav.mav.command_long_send(self.mav.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_LOGGING_START, 0,
                0, 0, 0, 0, 0, 0, 0)

    def stop_log(self):
        self.mav.mav.command_long_send(self.mav.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_LOGGING_STOP, 0,
                0, 0, 0, 0, 0, 0, 0)

    def _int_handler(self, sig, frame):
        self.got_sig_int = True

    async def read_messages(self):
        ''' main loop reading messages '''
        measure_time_start = timer()
        measured_data = 0

        next_heartbeat_time = timer()
        old_handler = signal.signal(signal.SIGINT, self._int_handler)

        # Connect to WebSocket
        await self.connect_websocket()

        while True:
            if self.got_sig_int:
                signal.signal(signal.SIGINT, old_handler)
                self.got_sig_int = False
                print('\nStopping log...')
                self.stop_log()
                # Continue reading until we get an ACK

            # handle heartbeat sending
            heartbeat_time = timer()
            if heartbeat_time > next_heartbeat_time:
                self.debug('sending heartbeat')
                self.mav.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)
                next_heartbeat_time = heartbeat_time + 1

            m, first_msg_start, num_drops = self.read_message()
            if m is not None:
                await self.process_streamed_ulog_data(m, first_msg_start, num_drops)

                # status output
                if self.logging_started:
                    measured_data += len(m)
                    measure_time_cur = timer()
                    dt = measure_time_cur - measure_time_start
                    if dt > 1:
                        sys.stdout.write('\rData Rate: {:0.1f} KB/s  Drops: {:} \033[K'.format(
                            measured_data / dt / 1024, self.num_dropouts))
                        sys.stdout.flush()
                        measure_time_start = measure_time_cur
                        measured_data = 0

            if not self.logging_started and timer()-self.start_time > 4:
                raise Exception('Start timed out. Is the logger running in MAVLink mode?')

    def read_message(self):
        ''' read a single mavlink message, handle ACK & return a tuple of (data, first
        message start, num dropouts) '''
        m = self.mav.recv_match(type=['LOGGING_DATA_ACKED',
                            'LOGGING_DATA', 'COMMAND_ACK'], blocking=True,
                            timeout=0.05)
        if m is not None:
            self.debug(m, 3)

            if m.get_type() == 'COMMAND_ACK':
                if m.command == mavutil.mavlink.MAV_CMD_LOGGING_START and \
                        not self.got_header_section:
                    if m.result == 0:
                        self.logging_started = True
                        print('Logging started. Waiting for Header...')
                    else:
                        raise Exception('Logging start failed', m.result)
                elif m.command == mavutil.mavlink.MAV_CMD_LOGGING_STOP and \
                        m.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    raise LoggingCompleted()
                return None, 0, 0

            # m is either 'LOGGING_DATA_ACKED' or 'LOGGING_DATA':
            is_newer, num_drops = self.check_sequence(m.sequence)

            # return an ack, even we already sent it for the same sequence,
            # because the ack could have been dropped
            if m.get_type() == 'LOGGING_DATA_ACKED':
                self.mav.mav.logging_ack_send(self.mav.target_system,
                        self.target_component, m.sequence)

            if is_newer:
                if num_drops > 0:
                    self.num_dropouts += num_drops

                if m.get_type() == 'LOGGING_DATA':
                    if not self.got_header_section:
                        print('Header received in {:0.2f}s (size: {:.1f} KB)'.format(
                              timer()-self.start_time, len(self.ulog_message)/1024))
                        self.logging_started = True
                        self.got_header_section = True
                self.last_sequence = m.sequence
                return m.data[:m.length], m.first_message_offset, num_drops

            else:
                self.debug('dup/reordered message '+str(m.sequence))

        return None, 0, 0

    def check_sequence(self, seq):
        ''' check if a sequence is newer than the previously received one & if
        there were dropped messages between the last and this '''
        if self.last_sequence == -1:
            return True, 0
        if seq == self.last_sequence: # duplicate
            return False, 0
        if seq > self.last_sequence:
            # account for wrap-arounds, sequence is 2 bytes
            if seq - self.last_sequence > (1<<15): # assume reordered
                return False, 0
            return True, seq - self.last_sequence - 1
        else:
            if self.last_sequence - seq > (1<<15):
                return True, (1<<16) - self.last_sequence - 1 + seq
            return False, 0

    async def process_streamed_ulog_data(self, data, first_msg_start, num_drops):
        ''' process streamed data and send over WebSocket '''
        if not self.got_ulog_header:  # the first 16 bytes need special treatment
            if len(data) < 16:  # that's never the case anyway
                raise Exception('first received message too short')
            await self.send_data(bytearray(data[0:16])) # send instead of write
            data = data[16:]
            self.got_ulog_header = True

        if self.got_header_section and num_drops > 0:
            if num_drops > 25: num_drops = 25
            # write a dropout message. We don't really know the actual duration,
            # so just use the number of drops * 10 ms
            await self.send_data(bytearray([ 2, 0, 79, num_drops*10, 0 ]))

        if num_drops > 0:
            await self.send_ulog_messages(self.ulog_message) # send instead of write
            self.ulog_message = []
            if first_msg_start == 255:
                return # no useful information in this message: drop it
            data = data[first_msg_start:]
            first_msg_start = 0

        if first_msg_start == 255 and len(self.ulog_message) > 0:
            self.ulog_message.extend(data)
            return

        if len(self.ulog_message) > 0:
            await self.send_data(bytearray(self.ulog_message + data[:first_msg_start]))
            self.ulog_message = []

        data = await self.send_ulog_messages(data[first_msg_start:]) # send instead of write
        self.ulog_message = data # store the rest for the next message

    async def send_ulog_messages(self, data):
        ''' send ulog data over WebSocket, assuming data starts with a
        valid ulog message. returns the remaining data at the end. '''
        while len(data) > 2:
            message_length = data[0] + data[1] * 256 + 3  # 3=ULog msg header
            if message_length > len(data):
                break
            await self.send_data(bytearray(data[:message_length]))
            data = data[message_length:]
        return data
    
    async def connect_websocket(self):
        """Connect to WebSocket server"""
        self.debug(f"Connecting to WebSocket at {self.websocket_url}...")
        self.websocket = await websockets.connect(self.websocket_url)
        self.debug("WebSocket connected")

    async def send_data(self, data):
        """Send data over WebSocket"""
        if self.websocket and data:  # Only send if we have data
            try:
                payload = {
                    'type': 'ulog',
                    'data': data.hex(),
                }
                encrypted = self.encrypt_data(payload)
                await self.websocket.send(encrypted)
            except Exception as e:
                self.debug(f"WebSocket failed: {e}", level=1)
                self.websocket = None

    def sign_message(self, message: str) -> str:
        """Generate HMAC signature for message (using string input)"""
        return hmac.new(SECRET_KEY.encode(),message.encode(),sha256).hexdigest()
    
    def encrypt_data(self, data: dict) -> str:
        """Encrypt data with AES-CBC and sign with HMAC"""
        json_data = json.dumps(data).encode()
        
        cipher = AES.new(SECRET_MESSAGE_KEY, AES.MODE_CBC)
        ct_bytes = cipher.encrypt(pad(json_data, AES.block_size))
        iv = base64.b64encode(cipher.iv).decode('utf-8')
        ct = base64.b64encode(ct_bytes).decode('utf-8')
        return json.dumps({
            'iv': iv,
            'ciphertext': ct,
            'signature': self.sign_message(ct)
        })

async def run():
    print("Starting MAVLink to WebSocket logger")
    print(f"MAVLink connection: udpin:127.0.0.1:14540")
    print(f"WebSocket server: ws://localhost:8086")
    
    mav_log_streaming = MavlinkLogStreaming(debug=1)
    
    try:
        print('Starting log...')
        mav_log_streaming.start_log()
        await mav_log_streaming.read_messages()
    except KeyboardInterrupt:
        print('\nAborting')
    except LoggingCompleted:
        print('Done')
    except Exception as e:
        print(f'Error: {e}')

def main():
    asyncio.get_event_loop().run_until_complete(run())

if __name__ == '__main__':
    main()