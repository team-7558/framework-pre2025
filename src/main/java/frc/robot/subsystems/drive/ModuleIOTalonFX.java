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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  private final Queue<Double> timestampQueue;

  private final StatusSignal<Double> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;

  private final double index;

  private final boolean isDriveMotorInverted = false;
  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOTalonFX(int index) {
    this.index = index;
    switch (index) {
      case 0:
        driveTalon = new TalonFX(0);
        turnTalon = new TalonFX(1);
        cancoder = new CANcoder(2);
        absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 1:
        driveTalon = new TalonFX(3);
        turnTalon = new TalonFX(4);
        cancoder = new CANcoder(5);
        absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 2:
        driveTalon = new TalonFX(6);
        turnTalon = new TalonFX(7);
        cancoder = new CANcoder(8);
        absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 3:
        driveTalon = new TalonFX(9);
        turnTalon = new TalonFX(10);
        cancoder = new CANcoder(11);
        absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.SupplyCurrentLimit = Drive.CFG.DRIVE_SUPPLY_LIMIT_A;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveTalon.getConfigurator().apply(driveConfig);

    var turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.SupplyCurrentLimit = Drive.CFG.TURN_SUPPLY_LIMIT_A;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turnTalon.getConfigurator().apply(turnConfig);

    setBrake(true);

    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getSupplyCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Drive.CFG.ODOMETRY_FREQUENCY_Hz, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.globalDelta_Hz,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    inputs.drivePos_r = drivePosition.getValueAsDouble() / Drive.CFG.DRIVE_GEAR_RATIO;
    inputs.driveVel_mps =
        Drive.CFG.WHEEL_RADIUS_m
            * Units.rotationsToRadians(driveVelocity.getValueAsDouble())
            / Drive.CFG.DRIVE_GEAR_RATIO;
    inputs.driveVolts_V = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrent_A = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnAbsPos_Rot2d =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPos_Rot2d =
        Rotation2d.fromRotations(turnPosition.getValueAsDouble() / Drive.CFG.TURN_GEAR_RATIO);
    inputs.turnVel_rps = turnVelocity.getValueAsDouble() / Drive.CFG.TURN_GEAR_RATIO;
    inputs.turnVolts_V = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrent_A = new double[] {turnCurrent.getValueAsDouble()};

    inputs.odometryTimestamps_s =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePos_r =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> value / Drive.CFG.DRIVE_GEAR_RATIO)
            .toArray();
    inputs.odometryTurnPos_Rot2d =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value / Drive.CFG.TURN_GEAR_RATIO))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveDC(double percentage) {
    driveTalon.setControl(new DutyCycleOut(percentage));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setBrake(boolean enable) {
    var dconfig = new MotorOutputConfigs();
    var tconfig = new MotorOutputConfigs();

    dconfig.Inverted =
        isDriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    dconfig.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    tconfig.Inverted =
        isTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tconfig.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(dconfig);
    turnTalon.getConfigurator().apply(tconfig);
  }
}
