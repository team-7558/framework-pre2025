package frc.robot.subsystems.arm;

import frc.robot.util.IState;

public enum ArmState implements IState {
  IDLE,
  DISABLED,
  HOLDING,
  ZEROING,
  MANUAL,
}
