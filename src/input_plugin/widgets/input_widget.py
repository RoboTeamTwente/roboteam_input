import rospy

from python_qt_binding import QtWidgets, QtGui, QtCore
from python_qt_binding.QtCore import pyqtSlot


class InputWidget(QtWidgets.QFrame):

    def __init__(self, input_id):
        super(InputWidget, self).__init__()

        self._id = input_id

        self.setLayout(QtWidgets.QHBoxLayout())

        self._id_label = QtWidgets.QLabel("Input " + str(self._id))
        self.layout().addWidget(self._id_label)

        # ---- Input port input ----

        self._input_port_input = QtWidgets.QLineEdit()
        self.port_validator = QtGui.QRegExpValidator(QtCore.QRegExp("[a-z]+[0-9]+"))
        self._input_port_input.setValidator(self.port_validator)
        self.layout().addWidget(self._input_port_input)

        self._input_port_input.setText(str(rospy.get_param(self.get_param_base() + "input")))
        self._input_port_input.textChanged.connect(self.change_input_port)

        # ---- /Input selector ----

        # ---- Type selector ----

        self._type_selector = QtWidgets.QComboBox()
        self.layout().addWidget(self._type_selector)

        self._type_selector.insertItem(0, "xbox")
        self._type_selector.insertItem(0, "playstation")
        self._type_selector.insertItem(0, "gioteck")

        currentJoyType = rospy.get_param(self.get_param_base() + "joyType");
        self._type_selector.setCurrentIndex(self._type_selector.findText(currentJoyType))

        self._type_selector.currentIndexChanged.connect(self.change_input_type)

        # ---- /Type selector ----

        self._robot_id_label = QtWidgets.QLabel("Robot id")
        self.layout().addWidget(self._robot_id_label)

        # --- Robot id input ----

        self._robot_id_input = QtWidgets.QLineEdit()
        self.int_validator = QtGui.QRegExpValidator(QtCore.QRegExp("[0-9]+"))
        self._robot_id_input.setValidator(self.int_validator)
        self.layout().addWidget(self._robot_id_input)

        self._robot_id_input.setText(str(rospy.get_param(self.get_param_base() + "robot")))
        self._robot_id_input.textChanged.connect(self.change_bot_id)

        # --- /Robot id input ----

    def get_param_base(self):
        return "input" + str(self._id) + "/"


    def change_input_port(self, input_port):
        rospy.set_param(self.get_param_base() + "input", input_port)


    @pyqtSlot(int)
    def change_input_type(self, type_index):
        input_type = self._type_selector.itemText(type_index)

        rospy.set_param(self.get_param_base() + "joyType", input_type)


    def change_bot_id(self, bot_id):
        try:
            bot_id_int = int(bot_id)
        except ValueError:
            # Default to 0.
            bot_id_int = 0

        rospy.set_param(self.get_param_base() + "robot", bot_id_int)
