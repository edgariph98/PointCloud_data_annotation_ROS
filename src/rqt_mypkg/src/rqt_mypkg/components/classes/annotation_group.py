from PyQt5.QtGui import QColor
import uuid

class AnnotationGroup:

    def __init__(self, _name, _color, _id=None):
        if _id is None:
            self.id = str(uuid.uuid4())
        else:
            self.id = _id
        self.name = _name
        self.color = _color
