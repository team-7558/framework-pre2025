package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/** Convention: CCW+ Steer angle */
public class Module {

  public enum Mode {
    HIGH_SPEED,
    HIGH_CONTROL
  }

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private Rotation2d turnSetpoint_Rot2d =
      null; // Setpoint for closed loop control, null for open loop
  private Double driveSetpoint_mps = null; // Setpoint for closed loop control, null for open loop
  private Rotation2d turnRelativeOffset_Rot2d = null; // Relative + Offset = Absolute
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    switch (Constants.currentMode) {
      case REAL:
        break;
      case SIM:
        break;
      case REPLAY:
        break;
    }
  }

  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void inputPeriodic() {
    Logger.processInputs("Swerve/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters =
          Units.rotationsToRadians(inputs.odometryDrivePos_r[i]) * Swerve.CFG.WHEEL_RADIUS_m;
      Rotation2d angle =
          inputs.odometryTurnPos_Rot2d[i].plus(
              turnRelativeOffset_Rot2d != null ? turnRelativeOffset_Rot2d : new Rotation2d());
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // On first cycle, reset relative turn encoder
    // Wait until absolute angle is nonzero in case it wasn't initialized yet
    if (turnRelativeOffset_Rot2d == null && inputs.turnAbsPos_Rot2d.getRadians() != 0.0) {
      turnRelativeOffset_Rot2d = inputs.turnAbsPos_Rot2d.minus(inputs.turnPos_Rot2d);
    }
  }

  public void outputPeriodic(Mode mode) {

    if (turnSetpoint_Rot2d != null) {
      io.setTurnAngle(getAngle().getRotations(), turnSetpoint_Rot2d.getRotations());

      if (driveSetpoint_mps != null) {

        // Cosine compensation
        double cosineCompensatedDriveSetpoint_mps =
            driveSetpoint_mps
                * Math.cos(turnSetpoint_Rot2d.getRadians() - inputs.turnPos_Rot2d.getRadians());

        if (mode == Mode.HIGH_CONTROL) {
          io.setDriveVelocity(cosineCompensatedDriveSetpoint_mps);
        } else {
          io.setDriveDC(Swerve.CFG.KV_T * cosineCompensatedDriveSetpoint_mps);
        }
      }
    }
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState writeState(SwerveModuleState state) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    // Update setpoints, controllers run in "outputPeriodic"
    turnSetpoint_Rot2d = optimizedState.angle;
    driveSetpoint_mps = optimizedState.speedMetersPerSecond;

    return optimizedState;
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.stop();

    // Disable closed loop control for turn and drive
    turnSetpoint_Rot2d = null;
    driveSetpoint_mps = null;
  }

  /** Sets whether brake mode is enabled. */
  public void setBrake(boolean brake) {
    io.setBrake(brake);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    if (turnRelativeOffset_Rot2d == null) {
      return inputs.turnPos_Rot2d;
    } else {
      return inputs.turnPos_Rot2d.plus(turnRelativeOffset_Rot2d);
    }
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return Units.rotationsToRadians(inputs.drivePos_r) * Swerve.CFG.WHEEL_RADIUS_m;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVel_mps;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }
}
