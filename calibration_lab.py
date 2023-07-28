import numpy as np
from SmarAct import SmarActController
from datetime import datetime
import logging
import matplotlib.pyplot as plt

now = datetime.now()

'''Here insert the calibration stage'''
stage_SN  = 'SLC1740su-42'


date_time = now.strftime("%m%d%Y_%H_%M")
file_name = 'logs/{}_{}.log'.format(stage_SN, date_time)

#start the logger
logging.basicConfig(filename=file_name, filemode='w', format='%(name)s - %(levelname)s - %(message)s', level= logging.INFO)
logging.info("BEGINNING TEST")
logging.info("Note: XY stage. Bottom actuator")
logging.info("Date: {}, Time: {}".format(now.strftime("%m/%d/%Y"), now.strftime("%H:%M:%S")))
logging.info("Stage serial number: {}".format(stage_SN))

smart = SmarActController('COM3')
smart.connect()
smart.get_system_id()
smart.get_sensor_type(0)
smart.get_firmware_version(0)
smart.get_serial_number(0)
smart.get_position(0)

smart.set_closed_loop_max_frequency(0, 100)
smart.set_closed_loop_move_speed(0, 5E5)
smart.get_closed_loop_speed(0)

smart.get_physical_position_known(0)
smart.calibrate_sensor(0)
smart.find_reference_mark(0, 1, 2000, 1)
#wait until the operation ends
smart.status_polling(0, 3, 30)
position_positive = smart.get_position(0)
smart.get_physical_position_known(0)

#move to the other side
smart.find_reference_mark(0, 0, 2000, 1)
#wait until the operation ends
#probably the best is to wait for the holding state... TBC
smart.status_polling(0, 0, 15)
position_negative = smart.get_position(0)
smart.get_physical_position_known(0)

#set the zero position
smart.set_position(0, 0)
smart.move_position_absolute(0, position_positive-position_negative, 1000)
#wait until the operation ends
smart.status_polling(0, 0, 15)
smart.get_position(0)

#try to go as right af possible
smart.move_position_relative(0, 1000)
smart.status_polling(0, 0, 15)
smart.get_position(0)

#go to zero
smart.move_position_absolute(0, 0, 1000)
#wait until the operation ends
smart.status_polling(0, 0, 15)
smart.get_position(0)

#try to go as left af possible
smart.move_position_relative(0, -1000)
smart.status_polling(0, 0, 15)
smart.get_position(0)

#go to the middle
smart.move_position_absolute(0, int((position_positive-position_negative)/2), 1000)
smart.status_polling(0, 3, 15)
smart.get_position(0)


smart.disconnect()
