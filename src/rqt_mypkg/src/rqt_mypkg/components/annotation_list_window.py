from python_qt_binding.QtWidgets import QWidget, QLabel, QVBoxLayout, QHBoxLayout, QPushButton, QMessageBox, QLineEdit, QComboBox, QTreeView, QAbstractItemView
from python_qt_binding.QtGui import QDoubleValidator, QColor, QStandardItemModel, QStandardItem, QBrush


class AnnotationListWindow(QWidget):

    def __init__(self, annotation_groups):
        QWidget.__init__(self)
        
        # self.tree = QTreeView(self)



        self.treeview = QTreeView(self)
        self.treeview.setHeaderHidden(True)
        self.treeview.setEditTriggers(QAbstractItemView.NoEditTriggers)
 
        # model = QStandardItemModel()
        # rootNode = model.invisibleRootItem()
        # branch1 = QStandardItem("Branch 1")
        # branch1.appendRow(QStandardItem("Child A"))
        # childnode = QStandardItem("Child B")
        # branch1.appendRow(childnode)
         
        # branch2 = QStandardItem("Branch 2")
        # branch2.appendRow(QStandardItem("Child C"))
        # branch2.appendRow(QStandardItem("Child D"))
         
        # rootNode.appendRow(branch1)
        # rootNode.appendRow(branch2)

        self.model = QStandardItemModel()
        rootNode = self.model.invisibleRootItem()
        for group in annotation_groups:

            rootNode.appendRow([QStandardItem(group.name), None])


        branch1 = QStandardItem("Branch 1")
        # branch1.setForeground(QBrush(QColor('white')))
        branch1.appendRow([QStandardItem("Child A"), QStandardItem("test A")])
        childnode = QStandardItem("Child B")
        branch1.appendRow([childnode, QStandardItem("test B")])
         
        branch2 = QStandardItem("Branch 2")
        branch2.appendRow([QStandardItem("Child C"), QStandardItem("test C")])
        branch2.appendRow([QStandardItem("Child D"), QStandardItem("test D")])
         
        rootNode.appendRow([ branch1, QStandardItem("test b1") ])
        rootNode.appendRow([ branch2, QStandardItem("test b2") ])
         
        self.treeview.setModel(self.model)
        self.treeview.setColumnWidth(0, 150)
        # self.setCentralWidget(self.treeview)
        self.treeview.hideColumn(1)

        layout = QVBoxLayout(self)
        layout.addWidget(self.treeview)
        self.setLayout(layout)

        self.treeview.selectionModel().selectionChanged.connect(self.on_selection_change)

    def on_selection_change(self, selected, deselected):
        selected_index = self.treeview.currentIndex()
        id_index = selected_index.sibling(selected_index.row(), 1)
        print(self.model.data( selected_index ))
        print(self.model.data( id_index ))