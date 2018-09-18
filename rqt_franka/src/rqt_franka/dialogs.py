import os
import rospkg
import rospy
from std_msgs.msg import Empty
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QDialog, QDialogButtonBox

class Dialog(object):
    def __init__(self):
        super(Dialog, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_franka'), 'resource', 'check_dialog.ui')
        self.dialog = QDialog()
        loadUi(ui_file, self.dialog)

    def show(self):
        self.dialog.exec_()

    def accept(self):
        self.dialog.accept()

    def rejected(self, fun=None):
        self.dialog.reject()


class ConfirmationDialog(Dialog):
    def __init__(self):
        super(ConfirmationDialog, self).__init__()

class InfoDialog(Dialog):
    def __init__(self, message, accept_txt):
        super(InfoDialog, self).__init__()
        self.dialog.button_box.button(QDialogButtonBox.Ok).setText(accept_txt)
        self.dialog.button_box.button(QDialogButtonBox.Cancel).setEnabled(False)
        self.dialog.button_box.button(QDialogButtonBox.Cancel).setVisible(False)
        self.dialog.check_question.setText(message)
