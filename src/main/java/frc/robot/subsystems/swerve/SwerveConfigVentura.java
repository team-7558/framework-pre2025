package frc.robot.subsystems.swerve;

import edu.wpi.first.math.util.Units;

public class SwerveConfigVentura implements ISwerveConfig {
  // Indexing
  public final int FL = 0, FR = 1, BL = 2, BR = 3;

  // Measured
  public final double TRACK_WIDTH_X_m = Units.inchesToMeters(18.75);
  public final double TRACK_WIDTH_Y_m = Units.inchesToMeters(18.75);
  public final double COR_OFFSET_X_m = 0.0;
  public final double COR_OFFSET_Y_m = 0.0;
  public final double DRIVE_BASE_RADIUS_m =
      Math.hypot(
          TRACK_WIDTH_X_m / 2.0,
          TRACK_WIDTH_Y_m / 2.0); // TODO: Figure out if center of rotation is non center case

  private final double WHEEL_RADIUS_FUDGE_FACTOR = 1.0 / 1.0;
  public final double WHEEL_RADIUS_m = WHEEL_RADIUS_FUDGE_FACTOR * Units.inchesToMeters(2.0);
  public final double R2M = WHEEL_RADIUS_m * Math.PI * 2.0;

  // Odometry
  public final double ODOMETRY_FREQUENCY_Hz = 250.0;

  // Constraints
  public final double MAX_LINEAR_VEL_mps = 5.575;
  public final double MAX_LINEAR_VEL_CONTROLLED_mps = 3.575;
  public final double MAX_LINEAR_VEL_THROTTLED_mps = 3.0;
  public final double MAX_ANGULAR_VEL_radps = MAX_LINEAR_VEL_mps / DRIVE_BASE_RADIUS_m;
  public final double MAX_ANGULAR_VEL_THROTTLED_radps = MAX_ANGULAR_VEL_radps * 0.5;

  // Gains
  public final double KV_T = 1.0;
  public final double KP_T = 2.5;
  public final double KP_R = 3.0;
}
