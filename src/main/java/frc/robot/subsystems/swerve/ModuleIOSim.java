// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.OdometryTimeStampsSim;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/** Add your docs here. */
public class ModuleIOSim implements ModuleIO {

  private final SwerveModuleSimulation sim;
  private final SimpleMotorFeedforward driveFeedforward;
  private final PIDController driveFeedback;
  private final PIDController turnFeedback;

  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    this.sim = moduleSimulation;

    driveFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
    driveFeedback = new PIDController(0.05, 0.0, 0.0);
    turnFeedback = new PIDController(7.0, 0.0, 0.0);

    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePos_r = Units.radiansToRotations(sim.getDriveWheelFinalPositionRad());
    inputs.driveVel_mps = Swerve.CFG.WHEEL_RADIUS_m * sim.getDriveWheelFinalSpeedRadPerSec();
    inputs.driveVolts_V = sim.getDriveMotorAppliedVolts();
    inputs.driveCurrent_A = new double[] {Math.abs(sim.getDriveMotorSupplyCurrentAmps())};

    inputs.turnAbsPos_Rot2d = sim.getSteerAbsoluteFacing();
    inputs.turnPos_Rot2d = Rotation2d.fromRadians(sim.getSteerRelativeEncoderPositionRad());
    inputs.turnVel_rps = Units.radiansToRotations(sim.getSteerRelativeEncoderSpeedRadPerSec());
    inputs.turnVolts_V = sim.getSteerMotorAppliedVolts();
    inputs.turnCurrent_A = new double[] {Math.abs(sim.getSteerMotorSupplyCurrentAmps())};

    inputs.odometryTimestamps = OdometryTimeStampsSim.getTimeStamps();
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
  public void setDriveDC(double percentage) {
    sim.requestDriveVoltageOut(percentage * 12.0);
  }

  @Override
  public void setDriveVelocity(double vel_mps) {
    double vel_radps = vel_mps / Swerve.CFG.WHEEL_RADIUS_m;
    double measured_radps = sim.getDriveWheelFinalSpeedRadPerSec();
    double ff = driveFeedforward.calculate(vel_radps);
    double u = driveFeedback.calculate(measured_radps, vel_radps);
    sim.requestDriveVoltageOut(ff + u);
  }

  @Override
  public void setTurnAngle(double measured_r, double setpoint_r) {
    double output =
        turnFeedback.calculate(Math.PI * 2.0 * measured_r, Math.PI * 2.0 * setpoint_r);
    sim.requestSteerVoltageOut(output);
  }
}
