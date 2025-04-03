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
        
        # MAVLink connection
        self.debug(f"Connecting with MAVLink to {self.mavlink_connection_string}...")
        self.mav = mavutil.mavlink_connection(self.mavlink_connection_string, autoreconnect=True)
        self.mav.wait_heartbeat()
        self.debug("HEARTBEAT OK\nMAVLink connection established\n")
        
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
        self.auth = AuthHandler()
        self.websocket_service.register_message_handler('fetchInfo', self.handle_fetch_info)
        # Threading and async
        self.message_queue = queue.Queue()
        self.running = False
        self.mavlink_thread = None
        self.websocket_task = None
        self.sender_task = None
        
        # Timing
        self.start_time = timer()

    def debug(self, message, level=1):
        """Debug output"""
        if self._debug >= level:
            print(message)
            
    async def handle_fetch_info(self, message):
        """Обработчик сообщения fetchInfo"""
        self.debug("Received fetchInfo request")
        # Можно отправить какую-то информацию в ответ
        return True
    
    async def socket_connection(self):
        """Establish and maintain WebSocket connection"""
        while self.running:
            try:
                access_token = self.auth.load_tokens()
                if not self.auth.ensure_tokens_are_valid(access_token):
                    self.debug("Invalid tokens, cannot connect", level=1)
                    await asyncio.sleep(5)
                    continue

                headers = {'Authorization': f'Bearer {access_token}'}
                await self.websocket_service.connect(headers=headers)
                
                # Start sender task after successful connection
                self.sender_task = asyncio.create_task(self.message_sender())
                await asyncio.sleep(1)  # Small delay to stabilize
                
                # Keep connection alive
                while self.running and self.websocket_service.websocket:
                    await asyncio.sleep(1)
                    
            except Exception as e:
                self.debug(f"WebSocket error: {e}")
                await asyncio.sleep(5)

    async def message_sender(self):
        """Отправка сообщений через WebSocket"""
        while self.running and self.websocket_service.websocket:
            try:
                if not self.message_queue.empty():
                    data = self.message_queue.get_nowait()
                    
                    # Подготовка сообщения в правильном формате
                    payload = {
                        'type': 'ulog',
                        'data': data.hex()  # Преобразование в hex строку
                    }
                    
                    # Отправка через WebSocketService с шифрованием
                    success = await self.websocket_service.send(payload)
                    if not success:
                        self.debug("Failed to send message, requeuing...")
                        self.message_queue.put(data)
                        await asyncio.sleep(1)
                else:
                    await asyncio.sleep(0.1)
            except Exception as e:
                self.debug(f"Message sender error: {e}")
                await asyncio.sleep(1)

    def start_log(self):
        """Start MAVLink logging"""
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_LOGGING_START, 0,
            0, 0, 0, 0, 0, 0, 0
        )

    def stop_log(self):
        """Stop MAVLink logging"""
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_LOGGING_STOP, 0,
            0, 0, 0, 0, 0, 0, 0
        )

    def mavlink_reader(self):
        """Thread for reading MAVLink messages"""
        measure_time_start = timer()
        measured_data = 0
        next_heartbeat_time = timer()

        while self.running:
            try:
                # Send heartbeat periodically
                if timer() > next_heartbeat_time:
                    self.mav.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0
                    )
                    next_heartbeat_time = timer() + 1

                # Read and process messages
                m, first_msg_start, num_drops = self.read_message()
                if m is not None:
                    self.process_streamed_ulog_data(m, first_msg_start, num_drops)

                    # Status output
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

                if not self.logging_started and timer() - self.start_time > 4:
                    self.debug("Start timed out. Is the logger running in MAVLink mode?")
                    self.running = False

            except Exception as e:
                self.debug(f"MAVLink reader error: {e}")
                self.running = False

    def process_streamed_ulog_data(self, data, first_msg_start, num_drops):
        """Process received ULog data"""
        try:
            if not self.got_ulog_header:
                if len(data) < 16:
                    raise Exception('First message too short')
                self.message_queue.put(bytearray(data[0:16]))
                data = data[16:]
                self.got_ulog_header = True

            if self.got_header_section and num_drops > 0:
                num_drops = min(num_drops, 25)
                self.message_queue.put(bytearray([2, 0, 79, num_drops*10, 0]))

            if num_drops > 0:
                if self.ulog_message:
                    self.message_queue.put(bytearray(self.ulog_message))
                self.ulog_message = []
                if first_msg_start == 255:
                    return
                data = data[first_msg_start:]
                first_msg_start = 0

            if first_msg_start == 255 and self.ulog_message:
                self.ulog_message.extend(data)
                return

            if self.ulog_message:
                self.message_queue.put(bytearray(self.ulog_message + data[:first_msg_start]))
                self.ulog_message = []

            remaining_data = self.send_ulog_messages(data[first_msg_start:])
            self.ulog_message = remaining_data

        except Exception as e:
            self.debug(f"ULog processing error: {e}")

    def send_ulog_messages(self, data):
        """Send complete ULog messages from data buffer"""
        while len(data) > 2:
            message_length = data[0] + data[1] * 256 + 3
            if message_length > len(data):
                break
            self.message_queue.put(bytearray(data[:message_length]))
            data = data[message_length:]
        return data

    def read_message(self):
        """Read a MAVLink message"""
        m = self.mav.recv_match(
            type=['LOGGING_DATA_ACKED', 'LOGGING_DATA', 'COMMAND_ACK'],
            blocking=True,
            timeout=0.05
        )
        
        if m is None:
            return None, 0, 0

        self.debug(m, 3)

        if m.get_type() == 'COMMAND_ACK':
            if m.command == mavutil.mavlink.MAV_CMD_LOGGING_START and not self.got_header_section:
                if m.result == 0:
                    self.logging_started = True
                    self.debug('Logging started. Waiting for Header...')
                else:
                    self.debug(f'Logging start failed: {m.result}')
            elif m.command == mavutil.mavlink.MAV_CMD_LOGGING_STOP and m.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                raise LoggingCompleted()
            return None, 0, 0

        is_newer, num_drops = self.check_sequence(m.sequence)

        if m.get_type() == 'LOGGING_DATA_ACKED':
            self.mav.mav.logging_ack_send(
                self.mav.target_system,
                self.target_component,
                m.sequence
            )

        if is_newer:
            if num_drops > 0:
                self.num_dropouts += num_drops

            if m.get_type() == 'LOGGING_DATA' and not self.got_header_section:
                self.debug(f'Header received in {timer()-self.start_time:.2f}s')
                self.logging_started = True
                self.got_header_section = True

            self.last_sequence = m.sequence
            return m.data[:m.length], m.first_message_offset, num_drops

        return None, 0, 0

    def check_sequence(self, seq):
        """Check message sequence number"""
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

    async def cleanup(self):
        """Clean up resources"""
        self.debug("Starting cleanup...")
        self.running = False

        # Stop MAVLink thread
        if self.mavlink_thread and self.mavlink_thread.is_alive():
            self.mavlink_thread.join(timeout=2)
            if self.mavlink_thread.is_alive():
                self.debug("Warning: MAVLink thread did not exit cleanly")

        # Stop sender task
        if self.sender_task and not self.sender_task.done():
            self.sender_task.cancel()
            try:
                await self.sender_task
            except asyncio.CancelledError:
                pass

        # Disconnect WebSocket
        if self.websocket_service:
            await self.websocket_service.disconnect()

        # Close MAVLink connection
        if self.mav:
            self.stop_log()
            self.mav.close()

        self.debug("Cleanup completed")

async def run(mav_log_streaming):
    """Main async run loop"""
    try:
        # Set up signal handlers
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(
                sig,
                lambda: asyncio.create_task(mav_log_streaming.cleanup())
            )

        mav_log_streaming.running = True
        
        # Start MAVLink logging
        mav_log_streaming.start_log()
        
        # Start MAVLink reader thread
        mav_log_streaming.mavlink_thread = threading.Thread(
            target=mav_log_streaming.mavlink_reader,
            daemon=True
        )
        mav_log_streaming.mavlink_thread.start()
        
        # Start WebSocket connection
        mav_log_streaming.websocket_task = asyncio.create_task(
            mav_log_streaming.socket_connection()
        )
        
        # Main loop
        while mav_log_streaming.running:
            await asyncio.sleep(1)
            
    except Exception as e:
        mav_log_streaming.debug(f"Error in main loop: {e}")
    finally:
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