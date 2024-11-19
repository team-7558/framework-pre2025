// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class Module {

  public enum Mode {
    HIGH_SPEED,
    HIGH_CONTROL
  }

  private static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
  // static final double ODOMETRY_FREQUENCY = 250.0;

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final PIDController turnFeedback;
  private Rotation2d turnSetpoint_Rot2d =
      null; // Setpoint for closed loop control, null for open loop
  private Double driveSetpoint_mps = null; // Setpoint for closed loop control, null for open loop
  private Rotation2d turnRelativeOffset_Rot2d = null; // Relative + Offset = Absolute
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    turnFeedback = new PIDController(7.0, 0.0, 0.0);
    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);

    setBrakeMode(true);
  }

  /**
   * Update inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void inputPeriodic() {
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps_s.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = Units.rotationsToRadians(inputs.odometryDrivePos_r[i]) * WHEEL_RADIUS;
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

    // Run closed loop turn control
    if (turnSetpoint_Rot2d != null) {
      io.setTurnPos(getAngle().getRadians(), turnSetpoint_Rot2d.getRadians());
      // io.setTurnVoltage(
      //    turnFeedback.calculate(getAngle().getRadians(), turnSetpoint_Rot2d.getRadians()));

      // Run closed loop drive control
      // Only allowed if closed loop turn control is running
      if (driveSetpoint_mps != null) {
        // Scale velocity based on turn error
        //
        // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
        // towards the setpoint, its velocity should increase. This is achieved by
        // taking the component of the velocity in the direction of the setpoint.
        double compensatedDriveVel_mps =
            driveSetpoint_mps * Math.cos(turnFeedback.getPositionError());

        // Run drive controller
        if (mode == Mode.HIGH_CONTROL) {
          io.setDriveVel(compensatedDriveVel_mps);
        } else {
          io.setDriveDC(compensatedDriveVel_mps / Drive.CFG.MAX_LINEAR_VEL_mps);
        }
      }
    }
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    // Update setpoints, controllers run in "periodic"
    turnSetpoint_Rot2d = optimizedState.angle;
    driveSetpoint_mps = optimizedState.speedMetersPerSecond;

    return optimizedState;
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveDC(0.0);

    // Disable closed loop control for turn and drive
    turnSetpoint_Rot2d = null;
    driveSetpoint_mps = null;
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    io.setBrake(enabled);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    if (turnRelativeOffset_Rot2d == null) {
      return new Rotation2d();
    } else {
      return inputs.turnPos_Rot2d.plus(turnRelativeOffset_Rot2d);
    }
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return Units.rotationsToRadians(inputs.drivePos_r) * WHEEL_RADIUS;
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
    return inputs.odometryTimestamps_s;
  }
}
