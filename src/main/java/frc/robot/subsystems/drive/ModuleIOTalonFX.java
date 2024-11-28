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
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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
  private final TalonFX steerTalon;
  private final CANcoder cancoder;

  private final Queue<Double> timestampQueue;

  private final StatusSignal<Double> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Double> steerAbsolutePosition;
  private final StatusSignal<Double> steerPosition;
  private final Queue<Double> steerPositionQueue;
  private final StatusSignal<Double> steerVelocity;
  private final StatusSignal<Double> steerAppliedVolts;
  private final StatusSignal<Double> steerCurrent;

  // Controls
  private final DutyCycleOut driveDCOut = new DutyCycleOut(0);
  private final VelocityVoltage driveVelOut = new VelocityVoltage(0).withEnableFOC(true);
  private final VoltageOut steerVOut = new VoltageOut(0);
  private final MotionMagicVoltage steerPosOut = new MotionMagicVoltage(0);
  private final MotionMagicExpoVoltage steerPosOut2 = new MotionMagicExpoVoltage(0);
  private final PositionVoltage steerPosOut3 = new PositionVoltage(0);

  private final boolean isDriveMotorInverted;
  private final boolean isSteerMotorInverted;
  private final Rotation2d absoluteEncoderOffset;

  private final double driveFactor_rpmeter;

  public ModuleIOTalonFX(
      int driveID,
      int steerID,
      int cancoderID,
      double absOffset,
      boolean invertDrive,
      boolean invertSteer) {

    driveTalon = new TalonFX(driveID, Drive.CFG.CAN_BUS);
    steerTalon = new TalonFX(steerID, Drive.CFG.CAN_BUS);
    cancoder = new CANcoder(cancoderID, Drive.CFG.CAN_BUS);
    absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED

    isDriveMotorInverted = invertDrive;
    isSteerMotorInverted = invertSteer;

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    TalonFXConfiguration steerConfig = new TalonFXConfiguration();
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

    cancoderConfig.MagnetSensor.MagnetOffset = absOffset;
    cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    driveConfig.Slot0 =
        new Slot0Configs()
            .withKP(Drive.CFG.DRIVE_0_KP)
            .withKI(Drive.CFG.DRIVE_0_KI)
            .withKD(Drive.CFG.DRIVE_0_KD)
            .withKS(Drive.CFG.DRIVE_0_KS)
            .withKV(Drive.CFG.DRIVE_0_KV)
            .withKD(Drive.CFG.DRIVE_0_KA);

    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted =
        isDriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    // driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.02;

    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = Drive.CFG.DRIVE_STATOR_LIMIT_A;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -Drive.CFG.DRIVE_STATOR_LIMIT_A;
    driveConfig.CurrentLimits.StatorCurrentLimit = Drive.CFG.DRIVE_STATOR_LIMIT_A;
    driveConfig.CurrentLimits.SupplyCurrentLimit = Drive.CFG.DRIVE_SUPPLY_LIMIT_A;
    driveConfig.CurrentLimits.SupplyCurrentThreshold = Drive.CFG.DRIVE_SUPPLY_THRES_LIMIT_A;
    driveConfig.CurrentLimits.SupplyTimeThreshold = Drive.CFG.DRIVE_SUPPLY_THRES_TIME_S;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    steerConfig.Slot0 =
        new Slot0Configs()
            .withKP(Drive.CFG.STEER_0_KP)
            .withKI(Drive.CFG.STEER_0_KI)
            .withKD(Drive.CFG.STEER_0_KD)
            .withKS(Drive.CFG.STEER_0_KS)
            .withKV(Drive.CFG.STEER_0_KV)
            .withKA(Drive.CFG.STEER_0_KA);

    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfig.MotorOutput.Inverted =
        isSteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    steerConfig.Feedback.FeedbackRemoteSensorID = cancoderID;
    steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    steerConfig.Feedback.RotorToSensorRatio = Drive.CFG.STEER_GEAR_RATIO;
    steerConfig.MotionMagic.MotionMagicCruiseVelocity =
        Constants.maxVelTalonFX_rps / Drive.CFG.STEER_GEAR_RATIO;
    steerConfig.MotionMagic.MotionMagicAcceleration =
        steerConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    steerConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * Drive.CFG.STEER_GEAR_RATIO;
    steerConfig.MotionMagic.MotionMagicExpo_kA = 0.1;

    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

    StatusCode response = cancoder.getConfigurator().apply(cancoderConfig);
    if (response.isError()) {
      System.out.print(Drive.CFG.CAN_BUS + cancoderID + " CAN FAILURE: " + response.toString());
    }

    response = driveTalon.getConfigurator().apply(driveConfig);
    if (response.isError()) {
      System.out.print(Drive.CFG.CAN_BUS + driveID + " CAN FAILURE: " + response.toString());
    }

    response = steerTalon.getConfigurator().apply(steerConfig);
    if (response.isError()) {
      System.out.print(Drive.CFG.CAN_BUS + steerID + " CAN FAILURE: " + response.toString());
    }

    // setBrake(true);

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    drivePosition = driveTalon.getPosition().clone();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity().clone();
    driveAppliedVolts = driveTalon.getMotorVoltage().clone();
    driveCurrent = driveTalon.getSupplyCurrent().clone();

    steerAbsolutePosition = cancoder.getAbsolutePosition().clone();
    steerPosition = steerTalon.getPosition().clone();
    steerPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(steerTalon, steerTalon.getPosition());
    steerVelocity = steerTalon.getVelocity().clone();
    steerAppliedVolts = steerTalon.getMotorVoltage().clone();
    steerCurrent = steerTalon.getSupplyCurrent().clone();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Drive.CFG.ODOMETRY_FREQUENCY_Hz, drivePosition, steerPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.globalDelta_Hz,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        steerAbsolutePosition,
        steerVelocity,
        steerAppliedVolts,
        steerCurrent);
    driveTalon.optimizeBusUtilization();
    steerTalon.optimizeBusUtilization();

    double rotPerWheelRot = Drive.CFG.DRIVE_GEAR_RATIO;
    double metersPerWheelRot = 2.0 * Math.PI * Drive.CFG.WHEEL_RADIUS_m;
    driveFactor_rpmeter = rotPerWheelRot / metersPerWheelRot;

    // Make control requests synchronous
    driveDCOut.UpdateFreqHz = 0;
    driveVelOut.UpdateFreqHz = 0;
    steerVOut.UpdateFreqHz = 0;
    steerPosOut.UpdateFreqHz = 0;
    steerPosOut2.UpdateFreqHz = 0;
    steerPosOut3.UpdateFreqHz = 0;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        steerAbsolutePosition,
        steerPosition,
        steerVelocity,
        steerAppliedVolts,
        steerCurrent);

    double drive_r = BaseStatusSignal.getLatencyCompensatedValue(drivePosition, driveVelocity);
    double steer_r = BaseStatusSignal.getLatencyCompensatedValue(steerPosition, steerVelocity);

    /*
     * Back out the drive rotations based on angle rotations due to coupling between
     * azimuth and steer
     */
    drive_r -= steer_r * Drive.CFG.COUPLE_RATIO;

    inputs.drivePos_r = drive_r / Drive.CFG.DRIVE_GEAR_RATIO;
    inputs.driveVel_mps = driveVelocity.getValue() / driveFactor_rpmeter;
    inputs.driveVolts_V = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrent_A = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnAbsPos_Rot2d =
        Rotation2d.fromRotations(steerAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPos_Rot2d = Rotation2d.fromRotations(steer_r);
    inputs.turnVel_rps = steerVelocity.getValueAsDouble();
    inputs.turnVolts_V = steerAppliedVolts.getValueAsDouble();
    inputs.turnCurrent_A = new double[] {steerCurrent.getValueAsDouble()};

    inputs.odometryTimestamps_s =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePos_r =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> value / Drive.CFG.DRIVE_GEAR_RATIO)
            .toArray();
    inputs.odometryTurnPos_Rot2d =
        steerPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    steerPositionQueue.clear();
  }

  @Override
  public void setDriveDC(double vel_mps) {
    double vel_rps = vel_mps * driveFactor_rpmeter;
    /* Back out the expected shimmy the drive motor will see */
    /* Find the angular rate to determine what to back out */
    double azimuthTurnRps = steerVelocity.getValue();
    /* Azimuth turn rate multiplied by coupling ratio provides back-out rps */
    double driveRateBackOut = azimuthTurnRps * Drive.CFG.COUPLE_RATIO;
    vel_rps += driveRateBackOut;
    double newVel_mps = vel_rps / driveFactor_rpmeter;
    driveTalon.setControl(new DutyCycleOut(newVel_mps / Drive.CFG.MAX_LINEAR_VEL_mps));
  }

  @Override
  public void setDriveVel(double vel_mps) {
    double vel_rps = vel_mps * driveFactor_rpmeter;
    /* Back out the expected shimmy the drive motor will see */
    /* Find the angular rate to determine what to back out */
    double azimuthTurnRps = steerVelocity.getValue();
    /* Azimuth turn rate multiplied by coupling ratio provides back-out rps */
    double driveRateBackOut = azimuthTurnRps * Drive.CFG.COUPLE_RATIO;
    vel_rps += driveRateBackOut;

    driveTalon.setControl(driveVelOut.withVelocity(vel_rps));
  }

  @Override
  public void setTurnVoltage(double volts) {
    steerTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnPos(double measured_rad, double pos_rad) {
    steerTalon.setControl(steerPosOut2.withPosition(Units.radiansToRotations(pos_rad)));
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
        isSteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tconfig.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(dconfig);
    steerTalon.getConfigurator().apply(tconfig);
  }
}
