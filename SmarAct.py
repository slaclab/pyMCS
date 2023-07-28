import serial
import json
import time
import logging


class SmarActController:
    def __init__(self, com_port) -> None:
        self.com = com_port
        self.ser = None
        self.num_of_channel = 0
        self.max_acceleration = 10E6
        self.max_speed = 100E6
        self.freq_min = 50
        self.freq_max = 18500
        self.status = {}
        logging.debug("Loading error codes json")
        with open('error_code.json') as file:
            self.error_codes = json.load(file)
            logging.debug("Error codes json loaded")
        logging.debug("Loading status json")
        with open('status.json') as file:
            self.status = json.load(file)
            logging.debug("Status json loaded")
        pass
    
    def connect(self):
        try:
            logging.info("Connecting to {} serial port".format(self.com))
            self.ser = serial.Serial(self.com, 9600, timeout=1)
            logging.info("Connected to {}".format(self.com))
            self.num_of_channel = self.get_number_of_channels()
            logging.info("Detected {} channels".format(self.num_of_channel))
        except Exception as e:
            print("Unable to connect")
            logging.critical("Unable to connect: " + str(e))
            raise Exception("Unable to connect: " + str(e))
        return self.connected()

    def connected(self):
        return self.ser.is_open
    
    def disconnect(self):
        if self.connected():
            self.ser.close()
            logging.info("Disconnected to {}".format(self.com))
    
    def send(self,command):
        if self.connected():
            self.ser.write(bytes(":{}\n".format(command), 'utf-8'))
            message = self.ser.readline()
            if message.startswith(b':') and message.endswith(b'\n'):
                message = message.removeprefix(b':')
                message = message.removesuffix(b'\n')
                message_string = message.decode()
                error = self.error_check(message_string)
                if error[0] > 0:
                    print(error[1])
                    print(error[2])
                return message_string
            else:
                return None
    
    def status_polling(self, channel:int, target_state, timeout = 60, verbose:bool = True):
        status = self.get_status(channel)
        previous_state = status
        if verbose:
            print(self.status[status])
            logging.info("Channel {} status: {}". format(channel, self.status[status]))
        timeout_elapsed = False
        start_time = time.time()
        while (status != str(target_state) and not timeout_elapsed):
            status = self.get_status(channel)
            if status != previous_state:
                if verbose:
                    print(self.status[status])
                    logging.info("Channel {} status: {}". format(channel, self.status[status]))
            previous_state = status
            if (time.time()-start_time) > timeout:
                logging.info("Channel {} polling timeout elapsed ({}s)". format(channel, timeout))
                timeout_elapsed = True
            time.sleep(0.1)
    
    def homing(self, channel:int, relative_position, timeout, verbose:bool = True):
        positions = []
        time_elapsed = []
        target_state = 0
        self.move_position_relative(0, relative_position, 10)
        status = self.get_status(channel)
        previous_state = status
        if verbose:
            print(self.status[status])
            logging.info("Channel {} status: {}". format(channel, self.status[status]))
        timeout_elapsed = False
        start_time = time.time()
        while (status != str(target_state) and not timeout_elapsed):
            status = self.get_status(channel)
            positions.append(self.get_position(0, True))
            time_elapsed.append(time.time())
            if status != previous_state:
                if verbose:
                    print(self.status[status])
                    logging.info("Channel {} status: {}". format(channel, self.status[status]))
            previous_state = status
            if (time.time()-start_time) > timeout:
                logging.info("Channel {} polling timeout elapsed ({}s)". format(channel, timeout))
                timeout_elapsed = True
            time.sleep(0.1)
        return time_elapsed, positions

    
    def get_system_id(self):
        raw = self.send("GSI")
        logging.info("Get System ID: {}". format(raw))
        return raw
    
    def get_number_of_channels(self):
        raw = self.send("GNC")
        raw = raw.removeprefix('N')
        logging.info("Get number of channel: {}". format(raw))
        return int(raw)
    
    def error_check(self, raw:str):
        if raw.startswith('E-1'):
           error_code = raw.removeprefix('E-1,')
           if error_code in self.error_codes.keys():
               return [int(error_code), self.error_codes[error_code]["description"], self.error_codes[error_code]["long_description"]]
           else:
               return [int(error_code), "Error not found","Error not found"]
        else:
            return [0, self.error_codes["0"]["description"],self.error_codes["0"]["long_description"]]
    
    def command_status(self, raw):
        error = self.error_check(raw)
        if error[0] == 0:
            print("Command OK")
            logging.info("Command OK")
        else:
            print("Error code: {}\n {}".format(error[0], error[1]))
            logging.info("Error code: {}\n {}".format(error[0], error[1]))
            logging.debug(error[2])
    
    def write_channel(self, channel:int, command):
         if channel >= 0 and channel < self.num_of_channel:
            raw = self.send("{}{}".format(command,channel))
            return raw
         else:
             return None
    
    def write_channel_argument(self, channel:int, command, arguments):
         if channel >= 0 and channel < self.num_of_channel:
            raw = self.send("{}{},{}".format(command, channel, arguments))
            return raw
         else:
             return None
        
    def get_closed_loop_acceleration(self, channel:int):
        raw = self.write_channel(channel, 'GCLA')
        if raw is not None:
            raw = raw.removeprefix("CLA{},".format(channel))
            logging.info("Get close loop acceleration ch{}: {} nm/s^2".format(channel, raw))
        return int(raw)
    
    def get_closed_loop_speed(self, channel:int):
        raw = self.write_channel(channel, 'GCLS')
        if raw is not None:
            raw = raw.removeprefix("CLS{},".format(channel))
            logging.info("Get close loop speed ch{}: {} nm/s".format(channel, raw))
        return int(raw)
    
    def get_scale(self, channel:int):
        raw = self.write_channel(channel, 'GSC')
        if raw is not None:
            raw = raw.removeprefix("SC{},".format(channel))
            part = raw.partition(',')
        return [int(part[0]), int(part[2])]
    
    def get_safe_direction(self, channel:int):
        raw = self.write_channel(channel, 'GSD')
        if raw is not None:
            raw = raw.removeprefix("SD{},".format(channel))
            logging.info("Get safe direction ch{}: {}".format(channel, raw))
        return int(raw)
    
    def get_sensor_type(self, channel:int):
        raw = self.write_channel(channel, 'GST')
        if raw is not None:
            raw = raw.removeprefix("ST{},".format(channel))
            logging.info("Get sensor type ch{}: {}".format(channel, raw))
        return int(raw)
    
    def set_closed_loop_acceleration(self, channel:int, acceleration:int):
            if acceleration >= 0 and acceleration < self.max_acceleration:
                raw = self.write_channel_argument(channel, 'SCLA', str(int(acceleration)))
                logging.info("Set close loop acceleration ch{}: {} nm/s".format(channel, acceleration))
                self.command_status(raw)

    def set_closed_loop_max_frequency(self, channel:int, frequency:int):
            if frequency >= self.freq_min and frequency <= self.freq_max:
                raw = self.write_channel_argument(channel, 'SCLF', str(int(frequency)))
                logging.info("Set close loop frequency ch{}: {} Hz".format(channel, frequency))
                self.command_status(raw)

    def set_closed_loop_move_speed(self, channel:int, speed:int):
            if speed >= 0 and speed <= self.max_speed:
                raw = self.write_channel_argument(channel, 'SCLS', str(int(speed)))
                logging.info("Set close loop speed ch{}: {} nm/s".format(channel, speed))
                self.command_status(raw)

    def set_safe_direction(self, channel:int, direction:int):
            if direction == 0 or direction == 1:
                raw = self.write_channel_argument(channel, 'SSD', str(int(direction)))
                logging.info("Set safe direction ch{}: {} ".format(channel, direction))
                self.command_status(raw)

    def set_sensor_type(self, channel:int, sensor_type:int):
            if sensor_type >= 0 and sensor_type <= 56:
                raw = self.write_channel_argument(channel, 'SST', str(int(sensor_type)))
                self.command_status(raw)
    
    def calibrate_sensor(self, channel:int):
        raw = self.write_channel(channel, 'CS')
        logging.info("Calibrating sensors ch{}".format(channel))
        self.command_status(raw)

    def find_reference_mark(self, channel:int, direction:int, hold_time:int = 1000, auto_zero:int = 0):
        if (direction == 0 or direction) == 1 and (auto_zero == 0 or auto_zero == 1) and hold_time > 0:
            raw = self.write_channel_argument(channel, 'FRM', "{},{},{}".format(int(direction), int(hold_time), int(auto_zero)))
            logging.info("Find reference mark ch{}".format(channel))
            self.command_status(raw)
        
    def move_position_absolute(self, channel:int, position: int, hold_time: int = 1000):
        if  hold_time > 0:
            raw = self.write_channel_argument(channel, 'MPA', "{},{}".format(int(position), int(hold_time)))
            logging.info("Move position absolute ch{}: {} nm".format(channel, position))
            self.command_status(raw)
    
    def move_position_relative(self, channel:int, position: int, hold_time: int = 1000):
        if  hold_time > 0:
            raw = self.write_channel_argument(channel, 'MPR', "{},{}".format(int(position), int(hold_time)))
            logging.info("Move position relative ch{}: {} nm".format(channel, position))
            self.command_status(raw)
    
    def get_position(self, channel:int, no_log = False):
        raw = self.write_channel(channel, 'GP')
        if raw is not None:
            raw = raw.removeprefix("P{},".format(channel))
            if not no_log:
                logging.info("Get position ch{}: {} nm".format(channel, int(raw)))
        return int(raw)
    
    def get_physical_position_known(self, channel:int):
        raw = self.write_channel(channel, 'GPPK')
        if raw is not None:
            raw = raw.removeprefix("PPK{},".format(channel))
            logging.info("Get physical position known ch{}: {}".format(channel, int(raw)))
        return int(raw)
    
    def get_status(self, channel:int):
        raw = self.write_channel(channel, 'GS')
        if raw is not None:
            raw = raw.removeprefix("S{},".format(channel))
        return str(raw)

    def set_position(self, channel:int, position: int):
        if  position > 0:
            raw = self.write_channel_argument(channel, 'SP', "{}".format(int(position)))
            logging.info("Set position ch{}: {}".format(channel, position))
            self.command_status(raw)

    def get_firmware_version(self, channel:int):
        raw = self.write_channel(channel, 'GFV')
        if raw is not None:
            raw = raw.removeprefix("FV{},".format(channel))
            part = raw.partition(',')
            logging.info("Get firmware version ch{}: {}".format(channel, raw))
        return part
    
    def get_serial_number(self, channel:int):
        raw = self.write_channel(channel, 'GSN')
        if raw is not None:
            raw = raw.removeprefix("SN{},".format(channel))
            logging.info("Get serial number ch{}: {}".format(channel, raw))
        return raw
