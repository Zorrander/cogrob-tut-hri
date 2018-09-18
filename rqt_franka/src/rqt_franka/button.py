import os
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

class Button(QWidget):
    """
    """

    def __init__(self, text, callback):
        super(Button, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_franka'), 'resource', 'button.ui')
        loadUi(ui_file, self)
        self.button.setText(text)
        self.button.clicked.connect(lambda: callback(text))

    def get_index(self):
        return self.index

class SkillButton(Button):

    def __init__(self, text, callback):
        super(SkillButton, self).__init__(text, callback)
