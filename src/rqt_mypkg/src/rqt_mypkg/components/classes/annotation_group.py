from PyQt5.QtGui import QColor
import uuid

class AnnotationGroup:

    def __init__(self, _name, _color):
        self.id = str(uuid.uuid4())
        self.name = _name
        self.color = _color
