from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QStandardItemModel, QStandardItem, QColor


class Item(QStandardItem):

    def __init__(self, text):
        super(Item, self).__init__(text)

    def done(self):
        self.setBackground( QColor('green') )

def init_model(title, view):
    model = QStandardItemModel(view)
    model.setColumnCount(1)
    model.setHeaderData(0, Qt.Horizontal, title)
    return model

def load_model(model, view):
    view.setModel(model) # Apply the model to the view
    view.show()          # Show the window and run the app

def load_elements(model, elements, checkable=False):
    for i in elements:
        item = Item(i)         # create an item with a caption
        item.setCheckable(checkable)    # add a checkbox to it if needed
        model.appendRow(item)           # Add the item to the model

def is_checked(item):
    return item.isCheckable() and item.checkState() == Qt.Checked

def edit_title(obj, text):
    obj.setText(text)
    obj.setAlignment(Qt.AlignCenter)
