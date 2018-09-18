#! /usr/bin/env python
import argparse
import actionlib
import rospy
from .main_window import MainWindow
from qt_gui.plugin import Plugin
from franka_tut_msgs.msg import *
from std_msgs.msg import Empty, String

class Interface(Plugin):

    def __init__(self, context):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        super(Interface, self).__init__(context)
        self.setObjectName('Interface')
        args = self._parse_args(context.argv())

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        pub = rospy.Publisher('/speech_to_text', String, queue_size= 10)
        human_pub = rospy.Publisher('human_done', Empty, queue_size= 10)
        pub_processed_inputs = rospy.Publisher('/processed_inputs', Instruction, queue_size=10)

        record_pub = rospy.Publisher('start_recording', String, queue_size=10)
        pub_end_motion = rospy.Publisher('end_record_motion', Empty, queue_size=10)

        self._widget = MainWindow(context, pub, human_pub, pub_processed_inputs,
                                        record_pub, pub_end_motion)
        context.add_widget(self._widget)

    def _parse_args(self, argv):
        parser = argparse.ArgumentParser(prog='rqt_franka', add_help=False)
        Interface.add_arguments(parser)
        return parser.parse_args(argv)

    @staticmethod
    def add_arguments(parser):
        group = parser.add_argument_group('Options for rqt_bag plugin')
        group.add_argument("-q", "--quiet", action="store_true", dest="quiet", help="Put plugin in silent mode")

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
