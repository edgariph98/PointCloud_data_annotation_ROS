def deleteItemsOfLayout(layout):
     if layout is not None:
         while layout.count():
             item = layout.takeAt(0)
             widget = item.widget()
             if widget is not None:
                 widget.setParent(None)
             else:
                 deleteItemsOfLayout(item.layout())

def get_annotation_group_by_id(annotation_groups, group_id):
    # Returns the annotation_group with the matching id or None if no match is found
    return next((elem for elem in annotation_groups if elem.id == group_id), None)

def get_annotation_group_by_name(annotation_groups, name):
    # Returns the annotation_group with the matching name or None if no match is found
    return next((elem for elem in annotation_groups if elem.name == name), None)