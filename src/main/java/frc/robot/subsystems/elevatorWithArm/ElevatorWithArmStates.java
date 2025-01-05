package frc.robot.subsystems.elevatorWithArm;

import frc.robot.util.IState;

public enum ElevatorWithArmStates implements IState {
  DISABLED,
  IDLE,
  TRAVELLINGELEVATOR,
  HOLDINGELEVATOR,
  TRAVELLINGARM,
  BOTHTRAVELLING,
  BOTHHOLDING
}
