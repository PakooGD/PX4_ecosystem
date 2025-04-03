#!/usr/bin/env python3

"""
Stream ULog data over MAVLink and send via WebSocket.
Hardcoded version for UDP connection and WebSocket server.
"""

from __future__ import print_function
import sys
import os
from timeit import default_timer as timer
os.environ['MAVLINK20'] = '1'  # The commands require mavlink 2
import asyncio
import json
import signal
from pymavlink import mavutil
from services.websocket_service import WebSocketService
from services.auth_service import AuthHandler


class MavlinkLogStreaming():
    '''Streams log data via MAVLink and sends over WebSocket.'''
    def __init__(self, debug=0):
        # MAVLink configuration
        self.mavlink_connection_string = 'udpin:127.0.0.1:14540'
        self._debug = debug
        
        # Services
        self.auth = AuthHandler()

        # WebSocket configuration
        self.websocket_url = 'ws://localhost:8082'
        self.websocket_service = WebSocketService(url=self.websocket_url)
        
        # MAVLink state
        self.mav = None
        self.got_ulog_header = False
        self.got_header_section = False
        self.ulog_message = []
        self.last_sequence = -1
        self.logging_started = False
        self.num_dropouts = 0
        self.target_component = 1
        self.got_sig_int = False
        
        # Performance tracking
        self.start_time = timer()
        self.measure_time_start = self.start_time
        self.measured_data = 0

        # Task management
        self.running = False
        self.main_task = None

    async def run(self):
        self.running = True
        
        # Setup signal handler
        loop = asyncio.get_event_loop()
        loop.add_signal_handler(signal.SIGINT, self._handle_signal)
        loop.add_signal_handler(signal.SIGTERM, self._handle_signal)
        
        try:
            # Connect to MAVLink first
            await self.connect_mavlink()

            while self.running:  # Main retry loop for connection issues
                try:
                    # Check and refresh tokens
                    access_token = self.auth.load_tokens()
                    if not self.auth.ensure_tokens_are_valid(access_token):
                        self.debug("Invalid tokens, cannot connect", level=1)
                        await asyncio.sleep(5)
                        continue

                    # Connect WebSocket with valid token
                    headers = {'Authorization': f'Bearer {access_token}'}
                    await self.websocket_service.connect(headers=headers)
                    
                    # Start logging if not already started
                    if not self.logging_started:
                        self.debug("Starting MAVLink logging...", level=1)
                        self.start_log()

                    # Main message processing loop
                    while self.running:
                        try:
                            await self.process_messages()
                            await asyncio.sleep(0.01)
                            
                        except Exception as e:
                            self.debug(f"Error processing messages: {e}", level=1)
                            await asyncio.sleep(1)  # Prevent tight error loop

                except Exception as e:
                    if self.running:  # Only log if we're not shutting down
                        self.debug(f"Connection error: {e}", level=1)
                        await asyncio.sleep(5)  # Wait before reconnecting
                    
        except asyncio.CancelledError:
            self.debug("Task cancelled", level=1)
        except Exception as e:
            self.debug(f"Fatal error: {e}", level=1)
            raise
        finally:
            await self.cleanup()

    async def cleanup(self):
        """Proper cleanup of all resources"""
        self.debug("Cleaning up...", level=1)
        self.running = False
        
        # Stop MAVLink logging
        self.stop_log()
        
        # Disconnect WebSocket
        if hasattr(self, 'websocket_service') and self.websocket_service:
            await self.websocket_service.disconnect()
        
        # Close MAVLink connection
        if hasattr(self, 'mav') and self.mav:
            try:
                self.mav.close()
            except:
                pass

    async def connect_mavlink(self):
        self.debug(f"Connecting to MAVLink at {self.mavlink_connection_string}...", level=1)
        self.mav = mavutil.mavlink_connection(self.mavlink_connection_string, autoreconnect=True)
        self.mav.wait_heartbeat()
        self.debug("MAVLink connection established", level=1)

    def debug(self, s, level=1):
        '''write some debug text'''
        if self._debug >= level:
            print(s)

    def start_log(self):
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_LOGGING_START, 0,
            0, 0, 0, 0, 0, 0, 0
        )

    def stop_log(self):
        if hasattr(self, 'mav') and self.mav:
            self.mav.mav.command_long_send(
                self.mav.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_LOGGING_STOP, 0,
                0, 0, 0, 0, 0, 0, 0
            )

    async def process_messages(self):
        # Handle MAVLink messages
        m, first_msg_start, num_drops = await self.read_mavlink_message()
        if m is not None:
            await self.process_ulog_data(m, first_msg_start, num_drops)

            # Update status
            if self.logging_started:
                current_time = timer()
                dt = current_time - self.measure_time_start
                if dt > 1:
                    rate = self.measured_data / dt / 1024
                    print(f'\rData Rate: {rate:0.1f} KB/s  Drops: {self.num_dropouts} \033[K', end='')
                    sys.stdout.flush()
                    self.measure_time_start = current_time
                    self.measured_data = 0

    async def read_mavlink_message(self):
        '''Read a single mavlink message'''
        try:
            m = self.mav.recv_match(
                type=['LOGGING_DATA_ACKED', 'LOGGING_DATA', 'COMMAND_ACK'],
                blocking=False,
                timeout=0.05
            )
            
            if m is None:
                return None, 0, 0
            
            self.debug(str(m), 3)

            if m.get_type() == 'COMMAND_ACK':
                if (m.command == mavutil.mavlink.MAV_CMD_LOGGING_START and 
                    not self.got_header_section):
                    if m.result == 0:
                        self.logging_started = True
                        print('Logging started. Waiting for Header...')
                    else:
                        raise Exception(f'Logging start failed with result: {m.result}')
                return None, 0, 0

            # Check sequence and handle drops
            is_newer, num_drops = self.check_sequence(m.sequence)
            if not is_newer:
                self.debug(f'Duplicate/reordered message {m.sequence}', 2)
                return None, 0, 0

            if num_drops > 0:
                self.num_dropouts += num_drops

            # Send ACK if needed
            if m.get_type() == 'LOGGING_DATA_ACKED':
                self.mav.mav.logging_ack_send(
                    self.mav.target_system,
                    self.target_component, 
                    m.sequence
                )

            if m.get_type() == 'LOGGING_DATA' and not self.got_header_section:
                print(f'Header received in {timer()-self.start_time:0.2f}s '
                      f'(size: {len(self.ulog_message)/1024:.1f} KB)')
                self.logging_started = True
                self.got_header_section = True

            self.last_sequence = m.sequence
            return m.data[:m.length], m.first_message_offset, num_drops

        except Exception as e:
            self.debug(f"Error reading MAVLink message: {e}", level=1)
            return None, 0, 0

    async def process_ulog_data(self, data, first_msg_start, num_drops):
        '''Process streamed ULog data'''
        try:
            if not self.got_ulog_header:
                if len(data) < 16:
                    raise Exception('First message too short')
                await self.send_data(bytearray(data[0:16]))
                data = data[16:]
                self.got_ulog_header = True

            if self.got_header_section and num_drops > 0:
                num_drops = min(num_drops, 25)
                dropout_msg = bytearray([2, 0, 79, num_drops * 10, 0])
                await self.send_data(dropout_msg)

            if num_drops > 0:
                await self.process_ulog_packets(self.ulog_message)
                self.ulog_message = []
                if first_msg_start == 255:
                    return
                data = data[first_msg_start:]

            if first_msg_start == 255 and len(self.ulog_message) > 0:
                self.ulog_message.extend(data)
                return

            if len(self.ulog_message) > 0:
                await self.send_data(bytearray(self.ulog_message + data[:first_msg_start]))
                self.ulog_message = []

            remaining_data = await self.process_ulog_packets(data[first_msg_start:])
            self.ulog_message = remaining_data

        except Exception as e:
            self.debug(f"Error processing ULog data: {e}", level=1)
            raise

    async def process_ulog_packets(self, data):
        '''Process complete ULog packets in data buffer'''
        try:
            while len(data) > 2:
                msg_len = data[0] + data[1] * 256 + 3  # 3=ULog msg header
                if msg_len > len(data):
                    break
                await self.send_data(bytearray(data[:msg_len]))
                data = data[msg_len:]
            return data
        except Exception as e:
            self.debug(f"Error processing ULog packets: {e}", level=1)
            raise

    async def send_data(self, data):
        """Send data over WebSocket"""
        if not data:
            return

        try:
            self.measured_data += len(data)
            
            payload = {
                'type': 'ulog',
                'data': data.hex(),
                'timestamp': timer()
            }
            
            await self.websocket_service.send(payload)
        except Exception as e:
            self.debug(f"Error sending data: {e}", level=1)
            raise

    def check_sequence(self, seq):
        '''Check if sequence is newer than last received'''
        if self.last_sequence == -1:
            return True, 0
        if seq == self.last_sequence:
            return False, 0
        if seq > self.last_sequence:
            if seq - self.last_sequence > (1 << 15):
                return False, 0
            return True, seq - self.last_sequence - 1
        else:
            if self.last_sequence - seq > (1 << 15):
                return True, (1 << 16) - self.last_sequence - 1 + seq
            return False, 0

    def _handle_signal(self):
        """Handle interrupt signals"""
        if not self.got_sig_int:  # Only handle first signal
            self.got_sig_int = True
            print('\nReceived interrupt signal, shutting down...')
            self.running = False
            if self.main_task:
                self.main_task.cancel()


async def main():
    mav_log_streaming = MavlinkLogStreaming(debug=1)
    try:
        # Store the main task reference
        mav_log_streaming.main_task = asyncio.current_task()
        await mav_log_streaming.run()
    except asyncio.CancelledError:
        print("\nMain task cancelled, shutting down...")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        await mav_log_streaming.cleanup()
        print("Cleanup completed")


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nApplication terminated by user")