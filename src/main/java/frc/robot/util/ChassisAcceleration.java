package frc.robot.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

public class ChassisAcceleration {

  public static ChassisSpeeds fromChassisSpeeds(
      ChassisSpeeds current, ChassisSpeeds next, double delta_ms) {
    return next.minus(current).div(Constants.globalDelta_s);
  }

  public static ChassisSpeeds fromAcceleration(
      ChassisSpeeds speed, ChassisSpeeds acc, double delta_ms) {
    return acc.times(delta_ms).plus(speed);
  }

  public static double magnitude(ChassisSpeeds s) {
    return Math.hypot(s.vxMetersPerSecond, s.vyMetersPerSecond);
  }
}
