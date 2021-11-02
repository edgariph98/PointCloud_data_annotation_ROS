import uuid

class Annotation:
    def __init__(self, _label, _group_id, _x_min, _x_max, _y_min, _y_max, _z_min, _z_max):
        self.id = str(uuid.uuid4())
        self.label = _label
        self.group_id = _group_id
        self.x_min = _x_min
        self.x_max = _x_max
        self.y_min = _y_min
        self.y_max = _y_max
        self.z_min = _z_min
        self.z_max = _z_max

        #include points in annotation
        # Unique id for an annotation, then put that id into field for PC2 points
        