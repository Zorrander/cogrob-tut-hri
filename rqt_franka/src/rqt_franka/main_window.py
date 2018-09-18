import os
import rospy
import rospkg
import actionlib
from .dialogs import *
from std_msgs.msg import String, Empty
from franka_tut_msgs.msg import *
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow, QTreeWidgetItem, QTableWidgetItem, QVBoxLayout
from rqt_franka.button import Button, SkillButton
from sensor_msgs.msg import JointState
from franka_tut_reasoning.ontology_manager import RdfManager

class MainWindow(QMainWindow):
    """
    """
    def __init__(self, context, pub, human_pub, pub_processed_inputs, record_pub, pub_end_motion):
        super(MainWindow, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_franka'), 'resource', 'main.ui')
        loadUi(ui_file, self)

        self.pub = pub
        self.human_pub = human_pub
        self.pub_processed_inputs = pub_processed_inputs
        self.record_pub = record_pub
        self.pub_end_motion = pub_end_motion

        self.active = False
        self.kb_interface = RdfManager()
        self.add_skill()

        self.windows.setCurrentIndex(0)
        btn = Button("Send command", self.handle_send_command_clicked)
        vbox = QVBoxLayout()
        vbox.addWidget(btn)
        self.command_actions.setLayout(vbox)

        self.teach_button.clicked.connect(self.handle_teach_clicked)
        self.step_done_button.clicked.connect(self.handle_done_clicked)
        self.execute_button.clicked.connect(lambda: self.handle_execute_clicked("Assemble", "Tool"))

        self.skill_tree.clicked.connect(self.handle_action_clicked)

    def add_skill(self):
        skills = self.kb_interface.get_skills()
        for skl in skills:
            btn = SkillButton(skl, self.handle_skill_clicked)
            self.skills_list.addWidget(btn)

    def handle_skill_clicked(self, skill):
        self.windows.setCurrentIndex(1)
        self.load_steps(skill)
        print skill

    def load_steps(self, skill):
        #skill += "_Skill"
        steps = self.kb_interface.get_task_steps(skill)
        for s in steps:
            step = QTreeWidgetItem(self.skill_tree)
            step.setText(0, s)
            tasks = self.kb_interface.extract_actions_from_step(s)
            print tasks
            for t in tasks:
                if not t.startswith('Null'):
                    task = QTreeWidgetItem(step)
                    task.setText(0, t)

    def handle_action_clicked(self, index):
        item = self.skill_tree.itemFromIndex(index)
        #self.update_description(item)
        #self.update_properties(item)
        self.toogle_teach_button(item)

    def handle_send_command_clicked(self, btn):
        command = self.commands.toPlainText()
        self.pub.publish(String(command))
        self.commands.setPlainText("")

    '''
    def update_description(self, item):
        description = self.kb_interface.retrieve_description(item.text(0))
        self.description.setText(description)

    def update_properties(self, item):
        properties = self.kb_interface.get_action_preconditions(item.text(0))
        if properties:
            count = 0
            for prop in properties:
                self.properties.setItem(count, 0, QTableWidgetItem(prop))
                count+=1
        else:
            self.properties.clear()
    '''

    def toogle_teach_button(self, item):
        if not self.active:
            if item.childCount() > 0 :
                self.teach_button.setEnabled(False)
            else:
                self.teach_button.setEnabled(True)

    def stop_recording(self):
        self.active = False
        self.pub_end_motion.publish(Empty())
        self.dialog.accept()

    def handle_teach_clicked(self):
        if not self.active:
            self.active = True
            selected = self.skill_tree.selectedItems()
            if len(selected) == 1:
                self.record_pub.publish(String(selected[0].text(0)))
                self.dialog = InfoDialog("Recording poses...", "Done")
                self.dialog.dialog.button_box.accepted.connect(self.stop_recording)
                self.dialog.show()
            else:
                pass

    def handle_done_clicked(self):
        self.human_pub.publish(Empty())

    def handle_execute_clicked(self, action, target):
        print "{} : {}".format(action, target)
        self.pub_processed_inputs.publish(Instruction(action, target))
