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

// Modified by 5516 "IRON MAPLE", original source:
// https://github.com/Shenzhen-Robotics-Alliance/maple-sim

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.OdometryTimeStampsSim;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/** Wrapper class around {@link SwerveModuleSimulation} that implements ModuleIO */
public class ModuleIOSim implements ModuleIO {
  private final SwerveModuleSimulation sim;

  private final SimpleMotorFeedforward driveFeedforward;
  private final PIDController driveFeedback;
  private final PIDController turnFeedback;

  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    this.sim = moduleSimulation;

    // Unlike the original project, the physics simulator robot can be treated exactly like the real
    // robot
    driveFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
    driveFeedback = new PIDController(0.05, 0.0, 0.0);
    turnFeedback = new PIDController(7.0, 0.0, 0.0);
    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePos_r = Units.radiansToRotations(sim.getDriveWheelFinalPositionRad());
    inputs.driveVel_mps = sim.getDriveWheelFinalSpeedRadPerSec() * Drive.CFG.WHEEL_RADIUS_m;
    inputs.driveVolts_V = sim.getDriveMotorAppliedVolts();
    inputs.driveCurrent_A = new double[] {Math.abs(sim.getDriveMotorSupplyCurrentAmps())};

    inputs.turnAbsPos_Rot2d = sim.getSteerAbsoluteFacing();
    inputs.turnPos_Rot2d = Rotation2d.fromRadians(sim.getSteerRelativeEncoderPositionRad());
    inputs.turnVel_rps = Units.radiansToRotations(sim.getSteerRelativeEncoderSpeedRadPerSec());
    inputs.turnVolts_V = sim.getSteerMotorAppliedVolts();
    inputs.turnCurrent_A = new double[] {Math.abs(sim.getSteerMotorSupplyCurrentAmps())};

    inputs.odometryTimestamps_s = OdometryTimeStampsSim.getTimeStamps();
    inputs.odometryDrivePos_r =
        Arrays.stream(sim.getCachedDriveWheelFinalPositionsRad())
            .map(Units::radiansToRotations)
            .toArray();
    inputs.odometryTurnPos_Rot2d =
        Arrays.stream(sim.getCachedSteerRelativeEncoderPositions())
            .mapToObj(Rotation2d::fromRadians)
            .toArray(Rotation2d[]::new);
  }

  @Override
  public void setDriveDC(double vel_mps) {
    sim.requestDriveVoltageOut(vel_mps / Drive.CFG.MAX_LINEAR_VEL_mps * 12.0);
  }

  @Override
  public void setDriveVel(double vel_mps) {
    double vel_radps = vel_mps / Drive.CFG.WHEEL_RADIUS_m;
    double output =
        driveFeedforward.calculate(vel_radps)
            + driveFeedback.calculate(sim.getDriveWheelFinalSpeedRadPerSec(), vel_radps);
    sim.requestDriveVoltageOut(output);
  }

  @Override
  public void setTurnVoltage(double volts) {
    sim.requestSteerVoltageOut(volts);
  }

  @Override
  public void setTurnPos(double measured_rad, double pos_rad) {
    double output = turnFeedback.calculate(measured_rad, pos_rad);
    sim.requestSteerVoltageOut(output);
  }

  @Override
  public void stop() {
    setDriveDC(0);
    setTurnVoltage(0);
  }
}
