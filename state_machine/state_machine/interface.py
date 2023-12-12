import PySimpleGUI as sg
import rclpy
from std_msgs.msg import Bool, String
from eufs_msgs.msg import MissionSelect
from rclpy.node import Node

class Interface(Node):
    def __init__(self):
        super().__init__("interface")

        """
        TODO:
            - Subscribe to can state for updated buttons
            - Add mission select text
        """

        self.asMasterSwitch = False
        self.asMasterSwitchPub = self.create_publisher(
            Bool,"/state_machine/as_master_switch", 1
        )
        self.asMasterSwitchMsg = Bool()

        self.tsMasterSwitch = False
        self.tsMasterSwitchPub = self.create_publisher(
             Bool,"/state_machine/ts_master_switch", 1
        )
        self.tsMasterSwitchMsg = Bool()
        self.goSignal = False
        self.goSignalPub = self.create_publisher( Bool,"/state_machine/go_signal", 1)
        self.goSignalMsg = Bool()

        self.ebs = False
        self.ebsPub = self.create_publisher( Bool,"/state_machine/ebs", 1)
        self.ebsMsg = Bool()
        self.ebs_fail = False
        self.ebsFailPub = self.create_publisher( Bool,"/state_machine/ebs_fail", 1)
        self.ebsFailMsg = Bool()
        self.flag = False
        self.flagPub = self.create_publisher(Bool,"/state_machine/flag",  1)
        self.flagMsg = Bool()
        self.drivingFlag = False
        self.drivingFlagPub = self.create_publisher(Bool,"/state_machine/driving_flag", 1)
        self.drivingFlagMsg = Bool()
        self.missionSelectPub = self.create_publisher(
             MissionSelect,"/state_machine/mission_select", 1
        )
        self.missionSelectMsg = MissionSelect()
        self.state = ""

        self.stateSub = self.create_subscription(String,"/state_machine/state_str",  self.stateCB, 10)
        self.window = self.create_window("dark")

    def stateCB(self,data: String):
        self.state = data.data
        self.window["-TEXT-"].update(self.state)
        self.window.refresh()


    def create_window(self,theme):
        sg.theme(theme)
        sg.set_options(font="Franklin 14", button_element_size=(6, 3))
        layout = [
            [
                sg.Text(
                    "",
                    font="Franklin 18",
                    justification="center",
                    expand_x=True,
                    pad=(10, 20),
                    key="-TEXT-",
                )
            ],
            [
                sg.Button(
                    "AS MASTER: OFF",
                    key="-ASMASTER-",
                    size=(23, 3),
                    button_color=("white", "red"),
                ),
                sg.Button(
                    "TS MASTER: OFF",
                    key="-TSMASTER-",
                    size=(23, 3),
                    button_color=("white", "red"),
                ),
            ],
            [
                sg.Button(
                    "GO SIGNAL: OFF",
                    key="-GOSIGNAL-",
                    size=(23, 3),
                    button_color=("white", "red"),
                ),
                sg.Button(
                    "EBS: ARMED",
                    key="-EBS-",
                    size=(10, 3),
                    button_color=("white", "green"),
                ),
                sg.Button(
                    "EBS: AVAILABLE",
                    key="-EBSFAIL-",
                    size=(10, 3),
                    button_color=("white", "green"),
                ),
            ],
            [
                sg.Button(
                    "FLAG: OFF",
                    key="-FLAG-",
                    size=(23, 3),
                    button_color=("white", "red"),
                ),
                sg.Button(
                    "DRIVING FLAG: OFF",
                    key="-DRIVINGFLAG-",
                    size=(23, 3),
                    button_color=("white", "red"),
                ),
            ],
            [
                sg.Button(0, expand_x=True),
                sg.Button(1, expand_x=True),
                sg.Button(2, expand_x=True),
                sg.Button(3, expand_x=True),
            ],
            [
                sg.Button(4, expand_x=True),
                sg.Button(5, expand_x=True),
                sg.Button(6, expand_x=True),
                sg.Button(7, expand_x=True),
            ],
        ]

        return sg.Window("IPG MODULE INTERFACE ", layout)

    def main(self):
        theme_menu = ["menu", ["LightGrey1", "dark", "DarkGray8", "random"]]


        current_num = []
        full_operation = []

        event, values = self.window.read()
        if event == sg.WIN_CLOSED:
            return

        if event in theme_menu[1]:
            self.window.close()
            self.window = self.create_window(event)

        if event == "-ASMASTER-":
            self.asMasterSwitch = not self.asMasterSwitch
            if self.asMasterSwitch == False:
                self.window["-ASMASTER-"].update(button_color=("white", "red"))
                self.window["-ASMASTER-"].update("AS MASTER: OFF")
                self.asMasterSwitchMsg.data = False
                self.asMasterSwitchPub.publish(self.asMasterSwitchMsg)
            elif self.asMasterSwitch == True:
                self.window["-ASMASTER-"].update(button_color=("white", "green"))
                self.window["-ASMASTER-"].update("AS MASTER: ON")
                self.asMasterSwitchMsg.data = True
                self.asMasterSwitchPub.publish(self.asMasterSwitchMsg)

        if event == "-TSMASTER-":
            self.tsMasterSwitch = not self.tsMasterSwitch
            if self.tsMasterSwitch == False:
                self.window["-TSMASTER-"].update(button_color=("white", "red"))
                self.window["-TSMASTER-"].update("TS MASTER: OFF")
                self.tsMasterSwitchMsg.data = False
                self.tsMasterSwitchPub.publish(self.tsMasterSwitchMsg)
            elif self.tsMasterSwitch == True:
                self.window["-TSMASTER-"].update(button_color=("white", "green"))
                self.window["-TSMASTER-"].update("TS MASTER: ON")
                self.tsMasterSwitchMsg.data = True
                self.tsMasterSwitchPub.publish(self.tsMasterSwitchMsg)

        if event == "-GOSIGNAL-":
            self.goSignal = not self.goSignal
            if self.goSignal == False:
                self.window["-GOSIGNAL-"].update(button_color=("white", "red"))
                self.window["-GOSIGNAL-"].update("GO SIGNAL: OFF")
                self.goSignalMsg.data = False
                self.goSignalPub.publish(self.goSignalMsg)
            elif self.goSignal == True:
                self.window["-GOSIGNAL-"].update(button_color=("white", "green"))
                self.window["-GOSIGNAL-"].update("GO SIGNAL: ON")
                self.goSignalMsg.data = True
                self.goSignalPub.publish(self.goSignalMsg)

        if event == "-EBS-":
            self.ebs = not self.ebs
            if self.ebs_fail == True:
                self.window["-EBS-"].update(button_color=("white", "grey"))
                self.window["-EBS-"].update("EBS: UNAVAILABLE")
            else:
                if self.ebs == False:
                    self.window["-EBS-"].update(button_color=("white", "green"))
                    self.window["-EBS-"].update("EBS: ARMED")
                    self.ebsMsg.data = False
                    self.ebsPub.publish(self.ebsMsg)
                elif self.ebs == True:
                    self.window["-EBS-"].update(button_color=("white", "red"))
                    self.window["-EBS-"].update("EBS: TRIGGERED")
                    self.ebsMsg.data = True
                    self.ebsPub.publish(self.ebsMsg)

        if event == "-EBSFAIL-":
            self.ebs_fail = not self.ebs_fail
            if self.ebs_fail == False:
                self.window["-EBSFAIL-"].update(button_color=("white", "green"))
                self.window["-EBSFAIL-"].update("EBS: AVAILABLE")
                self.window["-EBS-"].update(button_color=("white", "green"))
                self.window["-EBS-"].update("EBS: ARMED")
                self.ebs = False
                self.ebsMsg.data = False
                self.ebsPub.publish(self.ebsMsg)
                self.ebsFailMsg.data = False
                self.ebsFailPub.publish(self.ebsFailMsg)
            elif self.ebs_fail == True:
                self.window["-EBSFAIL-"].update(button_color=("white", "red"))
                self.window["-EBSFAIL-"].update("EBS: UNAVAILABLE")
                self.window["-EBS-"].update(button_color=("white", "grey"))
                self.window["-EBS-"].update("EBS: UNAVAILABLE")
                self.ebsFailMsg.data = True
                self.ebsFailPub.publish(self.ebsFailMsg)

        if event == "-FLAG-":
            self.flag = not self.flag
            if self.flag == False:
                self.window["-FLAG-"].update(button_color=("white", "red"))
                self.window["-FLAG-"].update("FLAG: OFF")
                self.flagMsg.data = False
                self.flagPub.publish(self.flagMsg)
            elif self.flag == True:
                self.window["-FLAG-"].update(button_color=("white", "green"))
                self.window["-FLAG-"].update("FLAG: ON")
                self.flagMsg.data = True
                self.flagPub.publish(self.flagMsg)

        if event == "-DRIVINGFLAG-":
            self.drivingFlag = not self.drivingFlag
            if self.drivingFlag == False:
                self.window["-DRIVINGFLAG-"].update(button_color=("white", "red"))
                self.window["-DRIVINGFLAG-"].update("DRIVING FLAG: OFF")
                self.drivingFlagMsg.data = False
                self.drivingFlagPub.publish(self.drivingFlagMsg)
            elif self.drivingFlag == True:
                self.window["-DRIVINGFLAG-"].update(button_color=("white", "green"))
                self.window["-DRIVINGFLAG-"].update("DRIVING FLAG: ON")
                self.drivingFlagMsg.data = True
                self.drivingFlagPub.publish(self.drivingFlagMsg)

        if event in ["0", "1", "2", "3", "4", "5", "6", "7"]:
            self.missionSelectMsg.ami_state = int(event)
            self.missionSelectPub.publish(self.missionSelectMsg)

def main(args=None):
    rclpy.init(args=args)
    node = Interface()
    while rclpy.ok():
        node.main()
        try:
            rclpy.spin_once(node)
        except KeyboardInterrupt:
            break
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()