package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public double drivePos_r = 0.0;
    public double driveVel_mps = 0.0;
    public double driveVolts_V = 0.0;
    public double[] driveCurrent_A = new double[] {};

    public Rotation2d turnAbsPos_Rot2d = new Rotation2d();
    public Rotation2d turnPos_Rot2d = new Rotation2d();
    public double turnVel_rps = 0.0;
    public double turnVolts_V = 0.0;
    public double[] turnCurrent_A = new double[] {};

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePos_r = new double[] {};
    public Rotation2d[] odometryTurnPos_Rot2d = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void setDriveDC(double percentage) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(double vel_mps) {}  

  /** Run the turn motor to the specified angle in rotations */
  public default void setTurnAngle(double measured_r, double setpoint_r) {}

  public default void stop() {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setBrake(boolean brake) {}
}
