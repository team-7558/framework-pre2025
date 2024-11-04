package frc.robot.subsystems.swerve;

import frc.robot.util.IState;

public enum PathingMode implements IState {
  DISABLED,
  FIELD_RELATIVE,
  POSE_FOLLOWING,
  PATH_FOLLOWING
}
