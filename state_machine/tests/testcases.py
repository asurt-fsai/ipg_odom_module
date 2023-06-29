import unittest
import rospy
from src.ros_can import *
import time
import enum

class test_state_machine(unittest.TestCase):
    def test_asoff_asready(self):
        sm = StateMachine()
        sm.asState = ASStateE.AS_OFF
        sm.asMasterSwitch = True
        sm.tsMasterSwitch = True
        sm.missionStatus = MissionStatusE.MISSION_SELECTED
        sm.ebsState = EBSStateE.EBS_ARMED
        sm.getState()
        self.assertEqual(sm.asState, ASStateE.AS_READY)
        
    def test_asready_asoff(self):
        sm = StateMachine()
        sm.asState = ASStateE.AS_READY
        sm.asMasterSwitch = False
        sm.getState()
        self.assertEqual(sm.asState.name , "AS_OFF")

    def test_asready_asdriving(self):
        sm = StateMachine()
        sm.asState = ASStateE.AS_READY
        sm.asMasterSwitch = True
        sm.readyTimer = time.time()
        sm.frAxleTrqReq = 0
        sm.rrAxleTrqReq = 0
        sm.steerAngleReq = 0
        abs(sm.steerAngleAct) < 5
        sm.directionReq = DirectionE.NEUTRAL
        sm.goSignal = False
        sm.getState()
        time.sleep(6)
        sm.goSignal = True
        sm.getState()
        self.assertEqual(sm.asState, ASStateE.AS_DRIVING)
    
    def test_asready_asemergency(self):
        sm = StateMachine()
        sm.asState = ASStateE.AS_READY
        sm.asMasterSwitch = True
        sm.sdcOpen = True
        sm.getState()
        self.assertEqual(sm.asState, ASStateE.EMERGENCY_BRAKE)

    def test_asdriving_asemergency_sdc(self):
        sm = StateMachine()
        sm.asState = ASStateE.AS_DRIVING
        sm.asMasterSwitch = True
        sm.goSignal = True
        sm.sdcOpen = True
        sm.getState()
        self.assertEqual(sm.asState, ASStateE.EMERGENCY_BRAKE)
    
    def test_asdriving_asemergency_asswitch(self):
        sm = StateMachine()
        sm.asState = ASStateE.AS_DRIVING
        sm.asMasterSwitch = False
        sm.goSignal = True
        sm.sdcOpen = False
        sm.getState()
        self.assertEqual(sm.asState, ASStateE.EMERGENCY_BRAKE)

    def test_asdriving_asemergency_gosignal(self):
        sm = StateMachine()
        sm.asState = ASStateE.AS_DRIVING
        sm.asMasterSwitch = True
        sm.goSignal = False
        sm.sdcOpen = False
        sm.getState()
        self.assertEqual(sm.asState, ASStateE.EMERGENCY_BRAKE)

    def test_asdriving_asemergency_mission(self):
        sm = StateMachine()
        sm.asState = ASStateE.AS_DRIVING
        sm.asMasterSwitch = True
        sm.goSignal = True
        sm.sdcOpen = False
        sm.missionStatus = MissionStatusE.MISSION_FINISHED
        sm.wheelspeed = [0,5,10,15]
        sm.getState()
        self.assertEqual(sm.asState, ASStateE.EMERGENCY_BRAKE)

    def test_asdriving_asemergency_neutral(self):
        sm = StateMachine()
        sm.asState = ASStateE.AS_DRIVING
        sm.asMasterSwitch = True
        sm.goSignal = True
        sm.sdcOpen = False
        sm.directionReq = DirectionE.NEUTRAL
        sm.wheelspeed = [0,5,10,15]
        sm.getState()
        self.assertEqual(sm.asState, ASStateE.EMERGENCY_BRAKE)

    def test_asdriving_asfinished(self):
        sm = StateMachine()
        sm.asState = ASStateE.AS_DRIVING
        sm.asMasterSwitch = True
        sm.goSignal = True
        sm.sdcOpen = False
        sm.missionStatus = MissionStatusE.MISSION_FINISHED
        sm.wheelspeed = [0,0,0,0]
        sm.getState()
        self.assertEqual(sm.asState, ASStateE.AS_FINISHED)

    def test_asfinished_asemergency(self):
        sm = StateMachine()
        sm.asState = ASStateE.AS_FINISHED
        sm.asMasterSwitch = True
        sm.sdcOpen = True
        sm.getState()
        self.assertEqual(sm.asState, ASStateE.EMERGENCY_BRAKE)

    def test_asfinished_asoff(self):
        sm = StateMachine()
        sm.asState = ASStateE.AS_FINISHED
        sm.asMasterSwitch = False
        sm.sdcOpen = False
        sm.getState()
        self.assertEqual(sm.asState, ASStateE.AS_OFF)

    def test_asemergency_asoff(self):
        sm = StateMachine()
        sm.asState = ASStateE.EMERGENCY_BRAKE
        sm.ebsTimer = time.time()
        time.sleep(15)
        sm.getState()
        self.assertEqual(sm.asState, ASStateE.AS_OFF)
    
if __name__ == "__main__":
    unittest.main()