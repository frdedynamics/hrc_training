from queue import Queue, Empty
import threading
import csv
import data

# Private Globals
_DATA = Queue()
_DATA_LOGGER = None
_logging_enabled = False


# exported functions
def log_metrics(mark, pedal_presses, angle_x, angle_y, angle_z, distance_from_target, t):
    if _DATA_LOGGER.running:
        new_data = [mark, pedal_presses, angle_x, angle_y, angle_z, distance_from_target, t]
        _DATA.put(new_data)
    else:
        return


def enable_logging():
    global _DATA_LOGGER
    global _logging_enabled
    _logging_enabled = True
    _DATA_LOGGER = DataLogger()
    _DATA_LOGGER.start()


def disable_logging():
    print("Trying to disable logging...")
    global _logging_enabled
    if _logging_enabled:
        if _DATA_LOGGER.running:
            print("Telling thread to stop...")
            _DATA_LOGGER.stop()
        _logging_enabled = False


class DataLogger(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.daemon = True
        self.filename = data.get_new_filename()
        self.fp = open(self.filename, 'w')
        self.writer = csv.writer(self.fp, lineterminator='\n')
        # write the header of the CSV file (the labels of each field/feature)
        self.writer.writerow(data.DATA_LABELS)
        self.running = True

    def run(self):
        while self.running:
            try:
                row = _DATA.get(timeout=1)
                self.writer.writerow(row)
            except Empty:
                continue
                
        self.close()

    def close(self):
        print("Closing...")
        if self.fp is not None:
            self.fp.close()
            self.fp = None
            print("Closed!")

    def stop(self):
        self.running = False

    def __del__(self):
       self.close()



