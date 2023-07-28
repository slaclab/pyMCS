import numpy as np
from SmarAct import SmarActController
                
smart = SmarActController('COM3')
smart.connect()
print("Physical position known: {}".format(smart.get_physical_position_known(0)))
print("Closed loop acc: {}".format(smart.get_closed_loop_acceleration(0)))
print("Closed loop velocity: {}".format(smart.get_closed_loop_speed(0)))
print("Scale: {}".format(smart.get_scale(0)))
print("Safe direction: {}".format(smart.get_safe_direction(0)))
print("Sensor type: {}".format(smart.get_sensor_type(0)))
print("Set closed loop acceleration")
smart.set_closed_loop_acceleration(0,1E6)
print("Closed loop acc: {}".format(smart.get_closed_loop_acceleration(0)))
smart.set_closed_loop_max_frequency(0, 5000)
smart.set_closed_loop_move_speed(0, 1E6)
print("Closed loop velocity: {}".format(smart.get_closed_loop_speed(0)))
smart.set_safe_direction(0,1)
print("Safe direction: {}".format(smart.get_safe_direction(0)))
smart.find_reference_mark(0, 0)
smart.status_polling(0, 0, 15)
print("Position: {}".format(smart.get_position(0)))
smart.move_position_absolute(0, -5E6)
smart.status_polling(0, 3, 15)

print("Position: {}".format(smart.get_position(0)))

smart.disconnect()