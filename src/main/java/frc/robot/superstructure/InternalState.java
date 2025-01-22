package frc.robot.superstructure;

import frc.robot.util.IState;

public enum InternalState implements IState {
  DISABLED,
  IDLE,
  BOOT,
  INTAKE_GND,
  INTAKE_HP,
  INTAKING_STAND,
  PRESCORE_REEF,
  SCORE_REEF,
  SCORE_DESCORE,
  PROCESS,
  NET,
  INTAKE_BALL,
  PREHANG,
  HANG
}
