"""
Stream ULog data over MAVLink and send via WebSocket using secure WebSocketService.
"""

import sys
import os
import signal
import threading
import queue
import asyncio
from timeit import default_timer as timer
from services.auth_service import AuthHandler
from pymavlink import mavutil
from services.websocket_service import WebSocketService

os.environ['MAVLINK20'] = '1'  # The commands require mavlink 2

class LoggingCompleted(Exception):
    pass

class MavlinkLogStreaming:
    def __init__(self, debug=0):
        self.mavlink_connection_string = 'udpin:127.0.0.1:14540'
        self._debug = debug
        
        # MAVLink
        self.mav = None

        # Logging state
        self.got_ulog_header = False
        self.got_header_section = False
        self.ulog_message = []
        self.last_sequence = -1
        self.logging_started = False
        self.num_dropouts = 0
        self.target_component = 1
        
        # WebSocket setup
        self.websocket_url = 'ws://localhost:8082'
        self.websocket_service = WebSocketService(self.websocket_url)
        self.websocket_service.register_message_handler('start_log', self._handle_start_command)
        self.websocket_service.register_message_handler('stop_log', self._handle_stop_command)
        self._connected = False

        self.auth = AuthHandler()
        # Threading and async
        self.message_queue = queue.Queue()
        self.running = False
        self.mavlink_thread = None
        self.websocket_task = None
        self.sender_task = None
        
        self.unsent_messages = []
        self.max_unsent_messages = 1000 
        # Timing
        self.start_time = timer()

    def debug(self, message, level=1):
        """Debug output"""
        if self._debug >= level:
            print(message)
    
    async def _handle_start_command(self, data: dict):
        """Handle start command from server"""
        self.debug("Received START command from server")
        if not self.logging_started:
            # Initialize MAVLink connection
            try:
                self.debug(f"Connecting with MAVLink to {self.mavlink_connection_string}...")
                self.mav = mavutil.mavlink_connection(self.mavlink_connection_string, autoreconnect=True)
                self.mav.wait_heartbeat()
                self.debug("MAVLink connection established\n")
                
                # Start logging
                self.start_log()

                # Start MAVLink reader thread
                self.mavlink_thread = threading.Thread(
                    target=self.mavlink_reader,
                    daemon=True
                )
                self.mavlink_thread.start()
    
                
            except Exception as e:
                self.debug(f"Error starting MAVLink: {e}")
                await self.websocket_service.send({'type': 'error', 'message': f'Failed to start MAVLink: {e}'})

    async def _handle_stop_command(self, data: dict):
        """Handle stop command from server"""
        self.debug("Received STOP command from server")
        if self.logging_started:
            self.stop_log()
            # Reset state
            self.logging_started = False
            self.got_ulog_header = False
            self.got_header_section = False
            self.last_sequence = -1
            
            # Clean up MAVLink connection
            if self.mav:
                try:
                    self.mav.close()
                except Exception as e:
                    self.debug(f"Error closing MAVLink: {e}")
                finally:
                    self.mav = None
            
            # Stop MAVLink thread if running
            if self.mavlink_thread and self.mavlink_thread.is_alive():
                self.mavlink_thread.join(timeout=2)
                if self.mavlink_thread.is_alive():
                    self.debug("Warning: MAVLink thread did not exit cleanly")

    async def socket_connection(self):
        """Establish and maintain WebSocket connection"""
        retry_delay = 1
        max_retry_delay = 30

        while self.running:
            try:
                access_token = self.auth.load_tokens()
                if not self.auth.ensure_tokens_are_valid(access_token):
                    self.debug("Invalid tokens, cannot connect", level=1)
                    await asyncio.sleep(5)
                    continue

                headers = {'Authorization': f'Bearer {access_token}'}
                self._connected = await self.websocket_service.connect(headers=headers)
                retry_delay = 1

                self.debug("WebSocket connection established")

                # Start sender task after successful connection
                self.sender_task = asyncio.create_task(self.message_sender())
                
                # Keep connection alive
                while self.running and self.websocket_service.websocket:
                    await asyncio.sleep(1)
                    
            except Exception as e:
                self.debug(f"WebSocket connection error: {e}. Retrying in {retry_delay} seconds...")
                await asyncio.sleep(retry_delay)
                retry_delay = min(retry_delay * 2, max_retry_delay)

    async def message_sender(self):
        """Send messages through WebSocket with retries"""
        while self.running:
            try:
                if not self.websocket_service or not self._connected:
                    await asyncio.sleep(1)
                    continue
                    
                # First send unsent messages
                while self.unsent_messages:
                    data = self.unsent_messages.pop(0)
                    success = await self.try_send_message(data)
                    if not success:
                        self.unsent_messages.insert(0, data)
                        break
                        
                # Then new messages from queue
                while not self.message_queue.empty():
                    data = self.message_queue.get_nowait()
                    success = await self.try_send_message(data)
                    if not success:
                        self.unsent_messages.append(data)
                        if len(self.unsent_messages) > self.max_unsent_messages:
                            self.unsent_messages.pop(0)
                        break
                        
                await asyncio.sleep(0.1)
                
            except Exception as e:
                self.debug(f"Message sender error: {e}")
                await asyncio.sleep(1)

    async def try_send_message(self, data):
        """Try to send message with error handling"""
        try:
            return await self.websocket_service.send(data)
        except Exception as e:
            self.debug(f"Send message error: {e}")
            return False


    def start_log(self):
        """Start MAVLink logging"""
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_LOGGING_START, 0,
            0, 0, 0, 0, 0, 0, 0
        )
        self.logging_started = True


    def stop_log(self):
        """Stop MAVLink logging"""
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_LOGGING_STOP, 0,
            0, 0, 0, 0, 0, 0, 0
        )
        self.logging_started = False


    def mavlink_reader(self):
        """Thread for reading MAVLink messages"""
        measure_time_start = timer()
        measured_data = 0
        next_heartbeat_time = timer()

        while self.running and self.logging_started:
            try:
                # Send heartbeat periodically
                heartbeat_time = timer()
                try:
                    if heartbeat_time > next_heartbeat_time:
                        self.debug('sending heartbeat')
                        self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)
                        next_heartbeat_time = heartbeat_time + 1
                except (AttributeError, OSError) as e:
                    self.debug(f"Heartbeat send failed: {e}")
                    break

                try:
                    m, first_msg_start, num_drops = self.read_message()

                    if m is not None:
                        if isinstance(m, list):
                            m = bytes(m) 
                        payload = {
                            'type': 'ulog',
                            'data': m.hex(),
                            'firstMsgStart': first_msg_start,
                            'numDrops': num_drops
                        }
                        self.message_queue.put(payload)

                        # status output
                        measured_data += len(m)
                        measure_time_cur = timer()
                        dt = measure_time_cur - measure_time_start
                        if dt > 1:
                            sys.stdout.write('\rData Rate: {:0.1f} KB/s  Drops: {:} \033[K'.format(
                                measured_data / dt / 1024, self.num_dropouts))
                            sys.stdout.flush()
                            measure_time_start = measure_time_cur
                            measured_data = 0

                except (OSError, IOError) as e:
                    if "Bad file descriptor" in str(e):
                        break
                    raise
            except LoggingCompleted:
                break
            except Exception as e:
                self.debug(f"MAVLink reader error: {e}")
                break


    def read_message(self):
        ''' read a single mavlink message, handle ACK & return a tuple of (data, first
        message start, num dropouts) '''
        m = self.mav.recv_match(type=['LOGGING_DATA_ACKED',
                            'LOGGING_DATA', 'COMMAND_ACK'], blocking=True,
                            timeout=0.05)
        
        if m is not None:
            self.debug(m, 3)

            if m.get_type() == 'COMMAND_ACK':
                if m.command == mavutil.mavlink.MAV_CMD_LOGGING_START and not self.got_header_section:
                    if m.result == 0:
                        self.logging_started = True
                        print('Logging started. Waiting for Header...')
                    else:
                        raise Exception('Logging start failed', m.result)
                elif m.command == mavutil.mavlink.MAV_CMD_LOGGING_STOP and m.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
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
                        print(f'{m}')
                        self.logging_started = True
                        self.got_header_section = True

                data = m.data[:m.length]
                if not self.got_ulog_header:
                    if len(data) >= 16:
                        # Verify ULog magic bytes
                        if data[:7] == b'ULog\x01\x12\x35':
                            self.got_ulog_header = True
                            print('Valid ULog header received')
                    else:
                        print('Initial message too short for ULog header')


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


    async def cleanup(self):
        """Improved cleanup method"""
        self.debug("Starting cleanup...")
        self.running = False

        # Stop MAVLink thread first
        if hasattr(self, 'mavlink_thread') and self.mavlink_thread:
            self.mavlink_thread.join(timeout=1)
        
        # Then close MAVLink connection
        if hasattr(self, 'mav') and self.mav:
            try:
                self.mav.close()
            except Exception as e:
                self.debug(f"Error closing MAVLink: {e}")
        
        # Cancel async tasks
        tasks = []
        if hasattr(self, 'sender_task') and self.sender_task:
            tasks.append(self.sender_task)
        if hasattr(self, 'websocket_task') and self.websocket_task:
            tasks.append(self.websocket_task)
        
        for task in tasks:
            try:
                task.cancel()
                await task
            except asyncio.CancelledError:
                pass
            except Exception as e:
                self.debug(f"Error cancelling task: {e}")
        
        # Finally disconnect WebSocket
        if hasattr(self, 'websocket_service') and self.websocket_service:
            await self.websocket_service.disconnect()
        
        self.debug("Cleanup completed")

async def run(mav_log_streaming):
    """Improved main run loop with better signal handling"""
    try:
        loop = asyncio.get_running_loop()
        
        # Use signals only if in main thread
        if threading.current_thread() is threading.main_thread():
            for sig in (signal.SIGINT, signal.SIGTERM):
                try:
                    loop.add_signal_handler(
                        sig,
                        lambda: asyncio.create_task(mav_log_streaming.cleanup())
                    )
                except NotImplementedError:
                    # Not all platforms support signals
                    pass
        
        mav_log_streaming.running = True
        
        # Start WebSocket connection
        mav_log_streaming.websocket_task = asyncio.create_task(
            mav_log_streaming.socket_connection()
        )

        # Main loop
        while mav_log_streaming.running:
            await asyncio.sleep(1)
            
    except asyncio.CancelledError:
        pass
    except Exception as e:
        mav_log_streaming.debug(f"Error in main loop: {e}")
    finally:
        if mav_log_streaming.running:
            await mav_log_streaming.cleanup()

def main():
    mav_log_streaming = MavlinkLogStreaming(debug=1)
    
    try:
        asyncio.run(run(mav_log_streaming))
    except KeyboardInterrupt:
        print("\nShutting down gracefully...")
    except Exception as e:
        print(f"Fatal error: {e}")
    finally:
        print("Application terminated")

if __name__ == '__main__':
    main()