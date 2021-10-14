from datetime import datetime
import uuid

class Frame:

    def __init__(self, _timestamp):
        self.id = uuid.uuid4()
        self.timestamp = _timestamp
        self.annotations = []
    
    def print_datetime(self):
        print(datetime.fromtimestamp(self.timestamp.to_sec()))

