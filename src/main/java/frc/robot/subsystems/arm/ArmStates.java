package frc.robot.subsystems.arm;

import frc.robot.util.IState;

public enum ArmStates implements IState {
  IDLE,
  DISABLED,
  TRAVELLING,
  HOLDING
}
