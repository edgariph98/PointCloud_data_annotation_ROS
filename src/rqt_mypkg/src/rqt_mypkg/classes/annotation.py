class Annotation:

    def __init__(self, _label, _group, _x_min, _x_max, _y_min, _y_max, _z_min, _z_max):
        self.label = _label
        self.group = _group
        self.x_min = _x_min
        self.x_max = _x_max
        self.y_min = _y_min
        self.y_max = _y_max
        self.z_min = _z_min
        self.z_max = _z_max

        #include points in annotation
        # Unique id for an annotation, then put that id into field for PC2 points
        