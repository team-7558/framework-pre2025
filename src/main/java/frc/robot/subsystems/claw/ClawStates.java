package frc.robot.subsystems.claw;

import frc.robot.util.IState;

public enum ClawStates implements IState {
  DISABLED,
  IDLE,
  TRAVELLING,
  HOLDING,
  OPEN,
  CLOSE,
  ZEROING
}
