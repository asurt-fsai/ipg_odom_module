from std_msgs.msg import Bool, String
from ackermann_msgs.msg import AckermannDriveStamped
from eufs_msgs.msg import (
    WheelSpeedsStamped,
    CanState,
    VehicleCommandsStamped,
    MissionSelect,
)
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu, NavSatFix
import time
from enum import Enum
import math
import rclpy
from rclpy.node import Node

MAX_RPM = 4000
WHEEL_RADIUS = 0.3
ENGINE_THRESHHOLD = -10
MAX_DEC =  -5
MAX_BRAKE = 0.5
MAX_TORQUE =  1000
MAX_STEER_ANGLE_DEG = 30   
WHEELBASE =  1.5
CMD_TIMEOUT =  0.5
FRICTION_CONSTANT =  0.5
TOTAL_MASS =  300

class ASStateE(Enum):
    AS_OFF = 1
    AS_READY = 2
    AS_DRIVING = 3
    EMERGENCY_BRAKE = 4
    AS_FINISHED = 5


class MissionStatusE(Enum):
    MISSION_NOT_SELECTED = 0
    MISSION_SELECTED = 1
    MISSION_RUNNING = 2
    MISSION_FINISHED = 3


class EBSStateE(Enum):
    EBS_UNAVAILABLE = 0
    EBS_ARMED = 1
    EBS_TRIGGERED = 2


class DirectionE(Enum):
    NEUTRAL = 0
    FORWARD = 1


class AMIStateE(Enum):
    AMI_NOT_SELECTED = 0
    AMI_ACCELERATION = 1
    AMI_SKIDPAD = 2
    AMI_AUTOCROSS = 3
    AMI_TRACK_DRIVE = 4
    AMI_STATIC_INSPECTION_A = 5
    AMI_STATIC_INSPECTION_B = 6
    AMI_AUTONOMOUS_DEMO = 7


class StateMachine(Node):
    def __init__(self):
        super().__init__("state_machine")
        self.asState = ASStateE.AS_OFF
        self.asMasterSwitch: bool = False
        self.tsMasterSwitch: bool = False
        self.missionStatus = MissionStatusE.MISSION_NOT_SELECTED
        self.ebsState = EBSStateE.EBS_ARMED
        self.ebsTimer = 0.0
        self.frAxleTrqReq = 0.0
        self.rrAxleTrqReq = 0.0
        self.steerAngleReq = 0.0
        self.steerAngleAct = 0.0
        self.directionReq = DirectionE.NEUTRAL
        self.rpmReq = 0.0
        self.missionComplete: bool = False
        self.sdcOpen: bool = False
        self.drivingFlag: bool = False
        self.amiState = AMIStateE.AMI_NOT_SELECTED
        self.readyTimer = 0.0
        self.goSignal = False
        self.goSignalWas = False
        self.brake = 0.0
        self.wheelspeed = [0.0, 0.0, 0.0, 0.0]  # fl,fr,rl,rr
        self.imuIn = Imu()
        self.timeLastCmd = time.time()

        # Publishers
        self.statePub = self.create_publisher(CanState,"/state_machine/state",10)
        self.stateStrPub = self.create_publisher(String,"/state_machine/state_str",10)
        self.wheelPub = self.create_publisher(WheelSpeedsStamped,"/state_machine/wheel_speeds",10)
        self.cmdPub = self.create_publisher(VehicleCommandsStamped,"/state_machine/cmd_pub",10)
        self.twistPub = self.create_publisher(TwistWithCovarianceStamped,"/state_machine/twist",10)
        self.imuPub = self.create_publisher(Imu,"/state_machine/imu",10)
        self.gpsPub = self.create_publisher(NavSatFix,"/state_machine/gps",10)

        # Subscribers
        self.goSignalSub = self.create_subscription(Bool, "/state_machine/go_signal", self.goSignalCB, 10)
        self.asMasterSub = self.create_subscription(
            Bool,"state_machine/as_master_switch",  self.asMasterSwitchCB,10
        )
        self.tsMasterSub = self.create_subscription(
             Bool,"state_machine/ts_master_switch", self.tsMasterSwitchCB,10
        )
        self.missionSelectSub = self.create_subscription(
             MissionSelect,"state_machine/mission_select", self.missionSelectCB,10
        )
        self.wheelSpeedSub = self.create_subscription(
            WheelSpeedsStamped,"carmaker/wheel_speeds",  self.wheelSpeedCB,10
        )
        self.ebsSub = self.create_subscription(Bool,"state_machine/ebs",  self.ebsStateCB, 10)
        self.ebsFailSub = self.create_subscription( Bool,"state_machine/ebs_fail", self.ebsFailCB, 10)
        self.flagSub = self.create_subscription( Bool,"state_machine/flag", self.flagCB, 10)
        self.drivingFlagSub = self.create_subscription( Bool,"state_machine/driving_flag", self.drivingFlagCB, 10)
        self.cmdSub = self.create_subscription( AckermannDriveStamped,"state_machine/cmd", self.cmdCB, 10)
    def wheelSpeedCB(self, wheelSpeed: WheelSpeedsStamped):
        # wheel speeds in rpm
        self.wheelspeed[0] = wheelSpeed.speeds.lf_speed
        self.wheelspeed[1] = wheelSpeed.speeds.rf_speed
        self.wheelspeed[2] = wheelSpeed.speeds.lb_speed
        self.wheelspeed[3] = wheelSpeed.speeds.rb_speed
        self.steerAngleAct = wheelSpeed.speeds.steering

    def wheelMsg(self):
        msg = WheelSpeedsStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.speeds.lf_speed = self.wheelspeed[0]
        msg.speeds.rf_speed = self.wheelspeed[1]
        msg.speeds.lb_speed = self.wheelspeed[2]
        msg.speeds.rb_speed = self.wheelspeed[3]
        msg.speeds.steering = self.steerAngleAct * math.pi / 180
        return msg

    def missionSelectCB(self, state: MissionSelect):
        self.amiState = AMIStateE(state.ami_state)

    def goSignalCB(self, state: Bool):
        self.goSignal = state.data

    def ebsStateCB(self, state: Bool):
        if state.data is True:
            self.ebsState = EBSStateE.EBS_TRIGGERED
            self.sdcOpen = True
        else:
            self.ebsState = EBSStateE.EBS_ARMED
            self.sdcOpen = False

    def ebsFailCB(self, state: Bool):
        if state.data is True:
            self.ebsState = EBSStateE.EBS_UNAVAILABLE
            self.sdcOpen = True
        else:
            pass

    def asMasterSwitchCB(self, state: Bool):
        self.asMasterSwitch = state.data

    def tsMasterSwitchCB(self, state: Bool):
        self.tsMasterSwitch = state.data

    def flagCB(self, flag: Bool):
        self.missionComplete = flag.data

    def drivingFlagCB(self, flag: Bool):
        self.drivingFlag = flag.data

    def cmdCB(self, cmd: AckermannDriveStamped):
        if self.drivingFlag is True:
            acceleration = cmd.drive.acceleration
            velocity = cmd.drive.speed
            self.steerAngleReq = cmd.drive.steering_angle * 180 / math.pi
            if acceleration > 0.0:
                #self.frAxleTrqReq = self.rrAxleTrqReq = 195
                self.frAxleTrqReq = self.rrAxleTrqReq = (TOTAL_MASS * WHEEL_RADIUS * abs(acceleration + FRICTION_CONSTANT))/ 2.0
                self.rpmReq = MAX_RPM
            elif acceleration == 0.0:
                self.frAxleTrqReq = self.rrAxleTrqReq = 0.0
                self.rpmReq = (velocity / WHEEL_RADIUS)*(60/(2*math.pi))
                self.brake = 0.0
            elif acceleration > ENGINE_THRESHHOLD:
                self.brake = 0.0
                self.rpmReq = 0.0
            else:
                self.frAxleTrqReq = self.rrAxleTrqReq = 0.0
                self.brake = float((-acceleration / MAX_DEC) * MAX_BRAKE)
        else:
            self.frAxleTrqReq = self.rrAxleTrqReq = 0.0
            self.rpmReq = 0.0
            self.steerAngleReq = 0.0
            self.brake = 0.0

    def getMissionStatus(self):
        if self.asState is ASStateE.AS_OFF:
            if self.amiState is not AMIStateE.AMI_NOT_SELECTED:
                self.missionStatus = MissionStatusE.MISSION_SELECTED
            else:
                self.missionStatus = MissionStatusE.MISSION_NOT_SELECTED
        elif self.asState is ASStateE.AS_READY:
            if self.drivingFlag:
                self.missionStatus = MissionStatusE.MISSION_RUNNING
            else:
                self.missionStatus = MissionStatusE.MISSION_SELECTED
        elif self.asState is ASStateE.AS_DRIVING:
            if self.missionComplete:
                self.missionStatus = MissionStatusE.MISSION_FINISHED
            else:
                self.missionStatus = MissionStatusE.MISSION_RUNNING
        elif self.asState is ASStateE.AS_FINISHED:
            self.missionStatus = MissionStatusE.MISSION_FINISHED
        else:
            self.missionStatus = MissionStatusE.MISSION_NOT_SELECTED

    def directionRequestState(self):
        if self.asState == ASStateE.AS_DRIVING and self.drivingFlag == True:
            self.directionReq = DirectionE.FORWARD
        else:
            self.directionReq = DirectionE.NEUTRAL

    def cmdMsg(self):
        cmd = VehicleCommandsStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"  
        cmd.commands.handshake = 1 #????????????????????????????????????????????????????????????
        cmd.commands.ebs = self.ebsState.value
        cmd.commands.direction = self.directionReq.value
        cmd.commands.mission_status = self.missionStatus.value
        cmd.commands.braking = self.brake
        cmd.commands.rpm = self.rpmReq
        cmd.commands.torque = self.frAxleTrqReq
        cmd.commands.steering = self.steerAngleReq

        return cmd

    def stateMsg(self):
        state = CanState()
        if 1 <= self.asState.value <= 5:
            state.as_state = self.asState.value - 1
        else:
            state.as_state = CanState.AS_OFF

        if self.amiState is AMIStateE.AMI_NOT_SELECTED:
            state.ami_state = CanState.AMI_NOT_SELECTED
        elif self.amiState is AMIStateE.AMI_ACCELERATION:
            state.ami_state = CanState.AMI_ACCELERATION
        elif self.amiState is AMIStateE.AMI_SKIDPAD:
            state.ami_state = CanState.AMI_SKIDPAD
        elif self.amiState is AMIStateE.AMI_AUTOCROSS:
            state.ami_state = CanState.AMI_AUTOCROSS
        elif self.amiState is AMIStateE.AMI_TRACK_DRIVE:
            state.ami_state = CanState.AMI_TRACK_DRIVE
        elif self.amiState is AMIStateE.AMI_STATIC_INSPECTION_A:
            state.ami_state = CanState.AMI_DDT_INSPECTION_A
        elif self.amiState is AMIStateE.AMI_STATIC_INSPECTION_B:
            state.ami_state = CanState.AMI_DDT_INSPECTION_B
        elif self.amiState is AMIStateE.AMI_AUTONOMOUS_DEMO:
            state.ami_state = CanState.AMI_AUTONOMOUS_DEMO
        else:
            state.ami_state = CanState.AMI_NOT_SELECTED

        return state

    def stateStrMsg(self, state: CanState):
        msg = String()

        str1 = "NO_SUCH_MESSAGE"
        if state.as_state is CanState.AS_OFF:
            str1 = "AS:OFF"
        elif state.as_state is CanState.AS_READY:
            str1 = "AS:READY"
        elif state.as_state is CanState.AS_DRIVING:
            str1 = "AS:DRIVING"
        elif state.as_state is CanState.AS_FINISHED:
            str1 = "AS:FINISHED"
        elif state.as_state is CanState.AS_EMERGENCY_BRAKE:
            str1 = "AS:EMERGENCY"

        str2 = "NO_SUCH_MESSAGE"
        if state.ami_state is CanState.AMI_NOT_SELECTED:
            str2 = "AMI:NOT_SELECTED"
        elif state.ami_state is CanState.AMI_ACCELERATION:
            str2 = "AMI:ACCELERATION"
        elif state.ami_state is CanState.AMI_SKIDPAD:
            str2 = "AMI:SKIDPAD"
        elif state.ami_state is CanState.AMI_AUTOCROSS:
            str2 = "AMI:AUTOCROSS"
        elif state.ami_state is CanState.AMI_TRACK_DRIVE:
            str2 = "AMI:TRACKDRIVE"
        elif state.ami_state is CanState.AMI_DDT_INSPECTION_A:
            str2 = "AMI:INSPECTION"
        elif state.ami_state is CanState.AMI_MANUAL:
            str2 = "AMI:MANUAL"

        if self.drivingFlag is True:
            str3 = "DRIVING:TRUE"
        else:
            str3 = "DRIVING:FALSE"

        msg.data = str1 + " " + str2 + " " + str3
        return msg

    def twistMsg(self):
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        avgWheelSpeed = (self.wheelspeed[2] + self.wheelspeed[3]) / 2
        msg.twist.twist.linear.x = avgWheelSpeed * math.pi * WHEEL_RADIUS / 30
        steeringAngle = -self.steerAngleAct * math.pi / 180
        msg.twist.twist.angular.z = (
            msg.twist.twist.linear.x * math.sin(steeringAngle) / WHEELBASE
        )
        msg.twist.covariance = [
            1e-9,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            1e-9,
        ]
        return msg

    def imuMsg(self):
        G_VALUE = 9.80665
        msg = Imu()
        msg = self.imuIn

    def getState(self):
        maxWheelSpeed = max(self.wheelspeed)
        if self.asState is ASStateE.AS_OFF:
            if (
                self.asMasterSwitch is True
                and self.tsMasterSwitch is True
                and self.missionStatus is MissionStatusE.MISSION_SELECTED
                and self.ebsState is EBSStateE.EBS_ARMED
            ):
                self.asState = ASStateE.AS_READY
                self.readyTimer = time.time()
            else:
                pass
        elif self.asState is ASStateE.AS_READY:
            if self.asMasterSwitch is False:
                self.asState = ASStateE.AS_OFF
            elif self.sdcOpen is True:
                self.asState = ASStateE.EMERGENCY_BRAKE
                self.ebsTimer = time.time()
            elif time.time() - self.readyTimer > 5:
                if (
                    self.frAxleTrqReq == 0
                    and self.rrAxleTrqReq == 0
                    and self.steerAngleReq == 0
                    and abs(self.steerAngleAct) < 5
                    and self.directionReq is DirectionE.NEUTRAL
                ):
                    if self.goSignalWas is False and self.goSignal is True:
                        self.asState = ASStateE.AS_DRIVING
                    else:
                        pass
                else:
                    pass
        elif self.asState is ASStateE.AS_DRIVING:
            if (
                self.missionStatus is MissionStatusE.MISSION_FINISHED
                and maxWheelSpeed < 10
            ):
                self.asState = ASStateE.AS_FINISHED
            elif (
                self.sdcOpen is True
                or self.asMasterSwitch is False
                or self.goSignal is False
                or (
                    self.missionStatus is MissionStatusE.MISSION_FINISHED
                    and maxWheelSpeed > 10
                )
                or (self.directionReq is DirectionE.NEUTRAL and maxWheelSpeed > 10)
            ):
                self.asState = ASStateE.EMERGENCY_BRAKE
                self.ebsTimer = time.time()
            else:
                pass
        elif self.asState is ASStateE.AS_FINISHED:
            if self.sdcOpen is True:
                self.asState = ASStateE.EMERGENCY_BRAKE
                self.ebsTimer = time.time()
            elif self.asMasterSwitch is False:
                self.asState = ASStateE.AS_OFF
            else:
                pass
        elif self.asState is ASStateE.EMERGENCY_BRAKE:
            if time.time() - self.ebsTimer > 15:
                if self.asMasterSwitch is False:
                    self.asState = ASStateE.AS_OFF
            else:
                pass
        self.goSignalWas = self.goSignal

    def reset(self):
        self.ebsState = EBSStateE.EBS_ARMED
        self.drivingFlag = False
        self.missionComplete = False
        self.steerAngleReq = 0.0
        self.frAxleTrqReq = self.rrAxleTrqReq = 0.0
        self.rpmReq = 0.0
        self.brake = 0.0

    def run(self):
        self.getMissionStatus()
        self.directionRequestState()
        self.getState()
        self.checkCMDTimeout()
        self.publish()
        if self.asState == ASStateE.AS_OFF:
            self.reset()

    def publish(self):
        stateMsgPub = self.stateMsg()
        stateStrMsgPub = self.stateStrMsg(stateMsgPub)
        wheelMsgPub = self.wheelMsg()
        cmdMsgPub = self.cmdMsg()

        self.statePub.publish(stateMsgPub)
        self.stateStrPub.publish(stateStrMsgPub)
        self.wheelPub.publish(wheelMsgPub)
        self.cmdPub.publish(cmdMsgPub)

    def checkCMDTimeout(self):
        if time.time() - self.timeLastCmd > CMD_TIMEOUT:
            self.ebsState = EBSStateE.EBS_TRIGGERED
        self.timeLastCmd = time.time()

        

def main(args=None):
    rclpy.init()
    stateMachine = StateMachine()
    while rclpy.ok():
        stateMachine.run()
        rclpy.spin_once(stateMachine)
    stateMachine.destroy_node()

if __name__ == "__main__":
    main()