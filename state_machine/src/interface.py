import PySimpleGUI as sg
import rospy
from std_msgs.msg import Bool, String
from eufs_msgs.msg import MissionSelect

rospy.init_node("interface")

"""
TODO:
    - Subscribe to can state for updated buttons
    - Add mission select text
"""

asMasterSwitch = False
asMasterSwitchPub = rospy.Publisher(
    "/state_machine/as_master_switch", Bool, queue_size=1
)
asMasterSwitchMsg = Bool()

tsMasterSwitch = False
tsMasterSwitchPub = rospy.Publisher(
    "/state_machine/ts_master_switch", Bool, queue_size=1
)
tsMasterSwitchMsg = Bool()

goSignal = False
goSignalPub = rospy.Publisher("/state_machine/go_signal", Bool, queue_size=1)
goSignalMsg = Bool()

ebs = False
ebsPub = rospy.Publisher("/state_machine/ebs", Bool, queue_size=1)
ebsMsg = Bool()

ebs_fail = False
ebsFailPub = rospy.Publisher("/state_machine/ebs_fail", Bool, queue_size=1)
ebsFailMsg = Bool()

flag = False
flagPub = rospy.Publisher("/state_machine/flag", Bool, queue_size=1)
flagMsg = Bool()

drivingFlag = False
drivingFlagPub = rospy.Publisher("/state_machine/driving_flag", Bool, queue_size=1)
drivingFlagMsg = Bool()

missionSelectPub = rospy.Publisher(
    "/state_machine/mission_select", MissionSelect, queue_size=1
)
missionSelectMsg = MissionSelect()

state = ""


def stateCB(data: String):
    global state
    state = data.data
    window["-TEXT-"].update(data.data)


def create_window(theme):
    global asMasterSwitch
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
                right_click_menu=theme_menu,
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


theme_menu = ["menu", ["LightGrey1", "dark", "DarkGray8", "random"]]
window = create_window("dark")

current_num = []
full_operation = []

while True:
    event, values = window.read()
    rospy.Subscriber("state_machine/state_str", String, stateCB)
    if event == sg.WIN_CLOSED:
        break

    if event in theme_menu[1]:
        window.close()
        window = create_window(event)

    if event == "-ASMASTER-":
        asMasterSwitch = not asMasterSwitch
        if asMasterSwitch == False:
            window["-ASMASTER-"].update(button_color=("white", "red"))
            window["-ASMASTER-"].update("AS MASTER: OFF")
            asMasterSwitchMsg.data = False
            asMasterSwitchPub.publish(asMasterSwitchMsg)
        elif asMasterSwitch == True:
            window["-ASMASTER-"].update(button_color=("white", "green"))
            window["-ASMASTER-"].update("AS MASTER: ON")
            asMasterSwitchMsg.data = True
            asMasterSwitchPub.publish(asMasterSwitchMsg)

    if event == "-TSMASTER-":
        tsMasterSwitch = not tsMasterSwitch
        if tsMasterSwitch == False:
            window["-TSMASTER-"].update(button_color=("white", "red"))
            window["-TSMASTER-"].update("TS MASTER: OFF")
            tsMasterSwitchMsg.data = False
            tsMasterSwitchPub.publish(tsMasterSwitchMsg)
        elif tsMasterSwitch == True:
            window["-TSMASTER-"].update(button_color=("white", "green"))
            window["-TSMASTER-"].update("TS MASTER: ON")
            tsMasterSwitchMsg.data = True
            tsMasterSwitchPub.publish(tsMasterSwitchMsg)

    if event == "-GOSIGNAL-":
        goSignal = not goSignal
        if goSignal == False:
            window["-GOSIGNAL-"].update(button_color=("white", "red"))
            window["-GOSIGNAL-"].update("GO SIGNAL: OFF")
            goSignalMsg.data = False
            goSignalPub.publish(goSignalMsg)
        elif goSignal == True:
            window["-GOSIGNAL-"].update(button_color=("white", "green"))
            window["-GOSIGNAL-"].update("GO SIGNAL: ON")
            goSignalMsg.data = True
            goSignalPub.publish(goSignalMsg)

    if event == "-EBS-":
        ebs = not ebs
        if ebs_fail == True:
            window["-EBS-"].update(button_color=("white", "grey"))
            window["-EBS-"].update("EBS: UNAVAILABLE")
        else:
            if ebs == False:
                window["-EBS-"].update(button_color=("white", "green"))
                window["-EBS-"].update("EBS: ARMED")
                ebsMsg.data = False
                ebsPub.publish(ebsMsg)
            elif ebs == True:
                window["-EBS-"].update(button_color=("white", "red"))
                window["-EBS-"].update("EBS: TRIGGERED")
                ebsMsg.data = True
                ebsPub.publish(ebsMsg)

    if event == "-EBSFAIL-":
        ebs_fail = not ebs_fail
        if ebs_fail == False:
            window["-EBSFAIL-"].update(button_color=("white", "green"))
            window["-EBSFAIL-"].update("EBS: AVAILABLE")
            window["-EBS-"].update(button_color=("white", "green"))
            window["-EBS-"].update("EBS: ARMED")
            ebs = False
            ebsMsg.data = False
            ebsPub.publish(ebsMsg)
            ebsFailMsg.data = False
            ebsFailPub.publish(ebsFailMsg)
        elif ebs_fail == True:
            window["-EBSFAIL-"].update(button_color=("white", "red"))
            window["-EBSFAIL-"].update("EBS: UNAVAILABLE")
            window["-EBS-"].update(button_color=("white", "grey"))
            window["-EBS-"].update("EBS: UNAVAILABLE")
            ebsFailMsg.data = True
            ebsFailPub.publish(ebsFailMsg)

    if event == "-FLAG-":
        flag = not flag
        if flag == False:
            window["-FLAG-"].update(button_color=("white", "red"))
            window["-FLAG-"].update("FLAG: OFF")
            flagMsg.data = False
            flagPub.publish(flagMsg)
        elif flag == True:
            window["-FLAG-"].update(button_color=("white", "green"))
            window["-FLAG-"].update("FLAG: ON")
            flagMsg.data = True
            flagPub.publish(flagMsg)

    if event == "-DRIVINGFLAG-":
        drivingFlag = not drivingFlag
        if drivingFlag == False:
            window["-DRIVINGFLAG-"].update(button_color=("white", "red"))
            window["-DRIVINGFLAG-"].update("DRIVING FLAG: OFF")
            drivingFlagMsg.data = False
            drivingFlagPub.publish(drivingFlagMsg)
        elif drivingFlag == True:
            window["-DRIVINGFLAG-"].update(button_color=("white", "green"))
            window["-DRIVINGFLAG-"].update("DRIVING FLAG: ON")
            drivingFlagMsg.data = True
            drivingFlagPub.publish(drivingFlagMsg)

    if event in ["0", "1", "2", "3", "4", "5", "6", "7"]:
        missionSelectMsg.ami_state = int(event)
        missionSelectPub.publish(missionSelectMsg)


window.close()
