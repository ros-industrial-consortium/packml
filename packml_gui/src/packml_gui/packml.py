"""
   Copyright 2017 Shaun Edwards

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
"""

import os
import rospy
import rospkg

from threading import Thread
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QThread, QRunnable, QThreadPool
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QPalette
from std_srvs.srv import Trigger
from packml_msgs.srv import Transition
from packml_msgs.srv import TransitionRequest
from packml_msgs.msg import Status
from packml_msgs.msg import State
from packml_msgs.msg import Mode

class WorkerThread(QRunnable):
    def __init__(self, service, req, set_msg):
        super(WorkerThread, self).__init__()
        self.service = service
        self.req = req
        self.set_msg = set_msg

    def run(self):
        res = self.service(self.req)
        self.set_msg(res.message)

class Packml(Plugin):

    def __init__(self, context):
        super(Packml, self).__init__(context)
        self.setObjectName('Packml')

        from argparse import ArgumentParser
        parser = ArgumentParser()

        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('packml_gui'), 'resource', 'packml.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('Packml')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        context.add_widget(self._widget)

        # Custom code begins here
        self._widget.reset_button.clicked[bool].connect(self.__handle_reset_clicked)
        self._widget.start_button.clicked[bool].connect(self.__handle_start_clicked)
        self._widget.stop_button.clicked[bool].connect(self.__handle_stop_clicked)
        self._widget.clear_button.clicked[bool].connect(self.__handle_clear_clicked)
        self._widget.hold_button.clicked[bool].connect(self.__handle_hold_clicked)
        self._widget.unhold_button.clicked[bool].connect(self.__handle_unhold_clicked)
        self._widget.suspend_button.clicked[bool].connect(self.__handle_suspend_clicked)
        self._widget.unsuspend_button.clicked[bool].connect(self.__handle_unsuspend_clicked)
        self._widget.abort_button.clicked[bool].connect(self.__handle_abort_clicked)

        self._service_thread = Thread(target=self.wait_for_services, args=())
        self._service_thread.start()

        self._status_sub = rospy.Subscriber('packml/status', Status, self.status_callback)

        self.threadpool = QThreadPool()

    def disable_all_buttons(self):
        self._widget.clear_button.setEnabled(False)
        self._widget.reset_button.setEnabled(False)
        self._widget.start_button.setEnabled(False)
        self._widget.stop_button.setEnabled(False)
        self._widget.hold_button.setEnabled(False)
        self._widget.suspend_button.setEnabled(False)
        self._widget.unhold_button.setEnabled(False)
        self._widget.unsuspend_button.setEnabled(False)
        self._widget.abort_button.setEnabled(False)

    def set_message_text(self, text):
        self._widget.message_box.setText("Message: " + text)

    def status_callback(self, msg):
        self.update_button_states(msg.state.val)
        self.update_status_fields(msg)


    def update_button_states(self, state):
        self.disable_all_buttons()
        if state == State.ABORTED:
            self._widget.clear_button.setEnabled(True)
        elif state == State.STOPPED:
            self._widget.reset_button.setEnabled(True)
        elif state == State.IDLE:
            self._widget.start_button.setEnabled(True)
        elif state == State.EXECUTE:
            self._widget.hold_button.setEnabled(True)
            self._widget.suspend_button.setEnabled(True)
        elif state == State.HELD:
            self._widget.unhold_button.setEnabled(True)
        elif state == State.SUSPENDED:
            self._widget.unsuspend_button.setEnabled(True)
        elif state == State.COMPLETE:
            self._widget.reset_button.setEnabled(True)

        if state != State.STOPPED and \
        state != State.STOPPING and \
        state != State.ABORTED and \
        state != State.ABORTING and \
        state != State.CLEARING:
            self._widget.stop_button.setEnabled(True)


        if state != State.ABORTED and \
        state != State.ABORTING:
            self._widget.abort_button.setEnabled(True)

    def update_status_fields(self, msg):
        self.update_state_field(msg.state.val)
        self._widget.substate.setText(str(msg.sub_state))
        self.update_mode_field(msg.mode.val)
        self._widget.error_code.setText(str(msg.error))
        self._widget.suberror_code.setText(str(msg.sub_error))


    def update_state_field(self, state):
        if state == State.UNDEFINED:
            self._widget.state_name.setText("UNDEFINED")
        elif state == State.OFF:
            self._widget.state_name.setText("OFF")
        elif state == State.STOPPED:
            self._widget.state_name.setText("STOPPED")
        elif state == State.STARTING:
            self._widget.state_name.setText("STARTING")
        elif state == State.IDLE:
            self._widget.state_name.setText("IDLE")
        elif state == State.SUSPENDED:
            self._widget.state_name.setText("SUSPENDED")
        elif state == State.EXECUTE:
            self._widget.state_name.setText("EXECUTE")
        elif state == State.STOPPING:
            self._widget.state_name.setText("STOPPING")
        elif state == State.ABORTING:
            self._widget.state_name.setText("ABORTING")
        elif state == State.ABORTED:
            self._widget.state_name.setText("ABORTED")
        elif state == State.HOLDING:
            self._widget.state_name.setText("HOLDING")
        elif state == State.HELD:
            self._widget.state_name.setText("HELD")
        elif state == State.RESETTING:
            self._widget.state_name.setText("RESETTING")
        elif state == State.SUSPENDING:
            self._widget.state_name.setText("SUSPENDING")
        elif state == State.UNSUSPENDING:
            self._widget.state_name.setText("UNSUSPENDING")
        elif state == State.CLEARING:
            self._widget.state_name.setText("CLEARING")
        elif state == State.UNHOLDING:
            self._widget.state_name.setText("UNHOLDING")
        elif state == State.COMPLETING:
            self._widget.state_name.setText("COMPLETING")
        elif state == State.COMPLETE:
            self._widget.state_name.setText("COMPLETE")
        else:
            self._widget.state_name.setTest("UNKNOWN")



    def update_mode_field(self, mode):
        if mode == Mode.UNDEFINED:
            self._widget.mode_name.setText("UNDEFINED")
        elif mode == Mode.AUTOMATIC:
            self._widget.mode_name.setText("AUTOMATIC")
        elif mode == Mode.SEMI_AUTOMATIC:
            self._widget.mode_name.setText("SEMI-AUTOMATIC")
        elif mode == Mode.MANUAL:
            self._widget.mode_name.setText("MANUAL")
        elif mode == Mode.IDLE:
            self._widget.mode_name.setText("IDLE")
        elif mode == Mode.SETUP:
            self._widget.mode_name.setText("SETUP")
        else:
            self._widget.mode_name.setText("UNKNOWN")



    def wait_for_services(self):
        self._widget.setEnabled(False)
        transition_service_name = 'packml/transition'
        rospy.wait_for_service(transition_service_name, 30)
        self.transition_service = rospy.ServiceProxy(transition_service_name, Transition)
        self._widget.setEnabled(True)

    def shutdown_plugin(self):
        self._status_sub.unregister()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass


    def __handle_start_clicked(self, checked):
        rospy.loginfo("Start button press")
        service_thread = WorkerThread(self.transition_service, TransitionRequest.START, self.set_message_text)
        self.threadpool.start(service_thread)

    def __handle_stop_clicked(self, checked):
        rospy.loginfo("Stop button press")
        service_thread = WorkerThread(self.transition_service, TransitionRequest.STOP, self.set_message_text)
        self.threadpool.start(service_thread)

    def __handle_reset_clicked(self, checked):
        rospy.loginfo("Reset button press")
        service_thread = WorkerThread(self.transition_service, TransitionRequest.RESET, self.set_message_text)
        self.threadpool.start(service_thread)

    def __handle_clear_clicked(self, checked):
        rospy.loginfo("Clear button press")
        service_thread = WorkerThread(self.transition_service, TransitionRequest.CLEAR, self.set_message_text)
        self.threadpool.start(service_thread)

    def __handle_hold_clicked(self, checked):
        rospy.loginfo("Hold button press")
        service_thread = WorkerThread(self.transition_service, TransitionRequest.HOLD, self.set_message_text)
        self.threadpool.start(service_thread)

    def __handle_unhold_clicked(self, checked):
        rospy.loginfo("Unhold button press")
        service_thread = WorkerThread(self.transition_service, TransitionRequest.UNHOLD, self.set_message_text)
        self.threadpool.start(service_thread)

    def __handle_suspend_clicked(self, checked):
        rospy.loginfo("Suspend button press")
        service_thread = WorkerThread(self.transition_service, TransitionRequest.SUSPEND, self.set_message_text)
        self.threadpool.start(service_thread)

    def __handle_unsuspend_clicked(self, checked):
        rospy.loginfo("Unsuspend button press")
        service_thread = WorkerThread(self.transition_service, TransitionRequest.UNSUSPEND, self.set_message_text)
        self.threadpool.start(service_thread)

    def __handle_abort_clicked(self, checked):
        rospy.loginfo("Abort button press")
        service_thread = WorkerThread(self.transition_service, TransitionRequest.ABORT, self.set_message_text)
        self.threadpool.start(service_thread)




    @staticmethod
    def add_arguments(parser):
        rospy.loginfo("Add arguments callback")
        group = parser.add_argument_group('Options for PackML plugin')
        group.add_argument('--arg1', action='store_true', help='arg1 help')

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure it
        # Usually used to open a configuration dialog
