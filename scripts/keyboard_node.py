import time
import keyboard
from pymavlink import mavutil

# Подключение к PX4 через MAVLink
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550') 
# Ожидание подключения
master.wait_heartbeat()
print("Подключено к PX4!")


class InputControl:
    def __init__(self): 
        self.movement_speed = 0.5 
        self.throttle_increment = 5  
        self.input_values = {
            'pitch': 0.0,
            'roll': 0.0,
            'yaw': 0.0,
            'throttle': 500, 
        }

    def update(self):
        self.input_values['pitch'] = 0.0
        if keyboard.is_pressed('w'):
            self.input_values['pitch'] = self.movement_speed
        if keyboard.is_pressed('s'):
            self.input_values['pitch'] = -self.movement_speed

        self.input_values['roll'] = 0.0
        if keyboard.is_pressed('a'):
            self.input_values['roll'] = -self.movement_speed
        if keyboard.is_pressed('d'):
            self.input_values['roll'] = self.movement_speed

        self.input_values['yaw'] = 0.0
        if keyboard.is_pressed('q'):
            self.input_values['yaw'] = -self.movement_speed
        if keyboard.is_pressed('e'):
            self.input_values['yaw'] = self.movement_speed
        
        #регулировка газа
        if keyboard.is_pressed('z'):
            self.input_values['throttle'] -= self.throttle_increment
        if keyboard.is_pressed('x'):
            self.input_values['throttle'] += self.throttle_increment
        
        #ограничение газа
        self.input_values['throttle'] = max(0, min(self.input_values['throttle'], 1000))


    def get_input_values(self):
        return self.input_values

# Основной цикл
input_control = InputControl()

try:
    while True:
        input_control.update()
        input_values = input_control.get_input_values()

        # Отправка команд ручного управления
        master.mav.manual_control_send(
            master.target_system,
            int(input_values['pitch'] * 1000),  # pitch
            int(input_values['roll'] * 1000),  # roll
            int(input_values['throttle']),  # throttle
            int(input_values['yaw'] * 1000),  # yaw
            0  # buttons
        )

        time.sleep(0.02)  

        if keyboard.is_pressed('esc'):  # Выход
            break

except Exception as e:
    print(f"Ошибка: {e}")

finally:
    master.close()


