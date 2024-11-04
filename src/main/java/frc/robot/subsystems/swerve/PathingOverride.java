package frc.robot.subsystems.swerve;

import frc.robot.util.IState;

public enum PathingOverride implements IState {
  NONE,
  INTAKING,
  TRACKING,
  BASELOCK
}
