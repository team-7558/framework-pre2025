package frc.robot.superstructure;

import frc.robot.util.IState;

public enum InternalState implements IState {
  DISABLED,
  IDLE,
  BOOT,
  SCORING_UP,
  SCORING_DOWN,
  INTAKING
}
