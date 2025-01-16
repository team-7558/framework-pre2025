package frc.robot.subsystems.arm;

import frc.robot.util.IState;

public enum ArmStates implements IState {
  DISABLED,
  IDLE,
  TRAVELLING,
  HOLDING,
}
