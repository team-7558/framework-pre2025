package frc.robot.subsystems.elevator;

import frc.robot.util.IState;

public enum ElevatorState implements IState {
  DISABLED,
  IDLE,
  ZEROING,
  HOLDING,
  MANUAL,
}
