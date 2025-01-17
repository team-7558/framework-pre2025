package frc.robot.subsystems.intake;

import frc.robot.util.IState;

public enum IntakeStates implements IState {
  IDLE,
  DISABLED,
  HOLDING,
  TRAVELLING,
  INTAKING,
  SPITTING
}
