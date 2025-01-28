package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ElevatorIOReal implements ElevatorIO {

  private final TalonFX leftFalcon;
  private final TalonFX rightFalcon;

  private final DigitalInput hallEffect;

  // TODO CHANGE

  private static final double MIN_HEIGHT_R = 0.6477000000000004;
  private static final double MAX_HEIGHT_R = 85.11303203125;
  private static final double STROKE_R = MAX_HEIGHT_R - MIN_HEIGHT_R;
  private static final double METERS_TO_ROTATIONS = STROKE_R / Elevator.ELEV_STROKE_M;
  private static final double ROTATIONS_TO_METERS = 1.0 / METERS_TO_ROTATIONS;
  private final StatusSignal<Double> pos_m;
  private final StatusSignal<Double> vel_mps;
  private final StatusSignal<Double> acc_mps2;
  private final StatusSignal<Double> volts_V;
  private final StatusSignal<Double> leftCurrent_A, rightCurrent_A;

  private final boolean isLeftMotorInverted = false;
  private boolean isBraked = true;
  private final VoltageOut voltageControl;
  private final MotionMagicVelocityVoltage mmVelControl;
  private final PositionVoltage posControl;
  private final MotionMagicVoltage mmPosControl;

  private double posOffset_m = 0.0;

  public ElevatorIOReal() {
    var leaderConfig = new TalonFXConfiguration();
    var followerConfig = new TalonFXConfiguration();

    voltageControl = new VoltageOut(0);
    mmVelControl = new MotionMagicVelocityVoltage(0, 0, true, 0, 2, false, false, false);
    posControl =
        new PositionVoltage(Elevator.ELEV_MIN_HEIGHT_M, 0, true, 0, 0, false, false, false);
    mmPosControl =
        new MotionMagicVoltage(Elevator.ELEV_MIN_HEIGHT_M, true, 0, 1, false, false, false);

    leftFalcon = new TalonFX(1);
    rightFalcon = new TalonFX(4);
    hallEffect = new DigitalInput(5);

    pos_m = rightFalcon.getPosition();
    vel_mps = rightFalcon.getVelocity();
    acc_mps2 = rightFalcon.getAcceleration();
    leftCurrent_A = rightFalcon.getStatorCurrent();
    rightCurrent_A = rightFalcon.getStatorCurrent();
    volts_V = leftFalcon.getMotorVoltage();

    // TODO CHANGE VLAUES

    leaderConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leaderConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
    leaderConfig.Feedback.SensorToMechanismRatio = METERS_TO_ROTATIONS;
    leaderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    leaderConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0;
    leaderConfig.MotionMagic.MotionMagicAcceleration = 1.0;
    leaderConfig.MotionMagic.MotionMagicJerk = 0;
    leaderConfig.MotionMagic.MotionMagicExpo_kV = 0.5;
    leaderConfig.MotionMagic.MotionMagicExpo_kA = 0.5;

    // Position control gains
    leaderConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    leaderConfig.Slot0.kG = 0.07; //
    leaderConfig.Slot0.kP = 176;
    leaderConfig.Slot0.kI = 0;
    leaderConfig.Slot0.kD = 4; // 16;

    // MotionMagic Position gains
    leaderConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;
    leaderConfig.Slot1.kG = 0.07;
    leaderConfig.Slot1.kV = 38;
    leaderConfig.Slot1.kS = 0.0;
    leaderConfig.Slot1.kA = 0; // 1;
    leaderConfig.Slot1.kP = 23;
    leaderConfig.Slot1.kI = 0;
    leaderConfig.Slot1.kD = 0.0;

    // MotionMagic Velocity control
    leaderConfig.Slot2.GravityType = GravityTypeValue.Elevator_Static;
    leaderConfig.Slot2.kG = 0;
    leaderConfig.Slot2.kV = 0.0;
    leaderConfig.Slot2.kS = 0.0;
    leaderConfig.Slot2.kA = 0.0;
    leaderConfig.Slot2.kP = 0.0;
    leaderConfig.Slot2.kI = 0.0;
    leaderConfig.Slot2.kD = 0.0;

    leftFalcon.getConfigurator().apply(leaderConfig);
    rightFalcon.getConfigurator().apply(leaderConfig);
    // rightFalcon.setControl(new Follower(leftFalcon.getDeviceID(), false).withUpdateFreqHz(50));
    BaseStatusSignal.setUpdateFrequencyForAll(
        1.0 / Constants.globalDelta_s, pos_m, vel_mps, volts_V, leftCurrent_A, rightCurrent_A);
    leftFalcon.optimizeBusUtilization();
    rightFalcon.optimizeBusUtilization();

    resetPos(Elevator.ELEV_MIN_HEIGHT_M);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(pos_m, vel_mps, acc_mps2, volts_V, leftCurrent_A, rightCurrent_A);
    inputs.pos_m = pos_m.getValueAsDouble() + posOffset_m;
    inputs.volts_V = volts_V.getValueAsDouble();
    inputs.vel_mps = vel_mps.getValueAsDouble();
    inputs.currents_A =
        new double[] {leftCurrent_A.getValueAsDouble(), rightCurrent_A.getValueAsDouble()};
    inputs.hallEffect = !hallEffect.get();
  }

  @Override
  public void setVel(double vel_mps) {
    double v = MathUtil.clamp(vel_mps, -Elevator.ELEV_MAX_VEL_MPS, Elevator.ELEV_MAX_VEL_MPS);
    leftFalcon.setControl(mmVelControl.withVelocity(v));
    rightFalcon.setControl(mmVelControl.withVelocity(v));
  }

  @Override
  public void holdPos(double position) {
    leftFalcon.setControl(posControl.withPosition(position - posOffset_m));
    rightFalcon.setControl(posControl.withPosition(position - posOffset_m));
  }

  @Override
  public void travelToPos(double position) {
    leftFalcon.setControl(mmPosControl.withPosition(position - posOffset_m));
    rightFalcon.setControl(mmPosControl.withPosition(position - posOffset_m));
  }

  @Override
  public void setVoltage(double volts_V) {
    leftFalcon.setControl(voltageControl.withOutput(volts_V));
    rightFalcon.setControl(voltageControl.withOutput(volts_V));
  }

  @Override
  public void resetPos(double pos_m) {
    leftFalcon.setPosition(0);
  }

  @Override
  public void stop() {
    leftFalcon.stopMotor();
    rightFalcon.stopMotor();
  }

  public void toggleBrake() {
    var config = new MotorOutputConfigs();

    config.Inverted = InvertedValue.Clockwise_Positive;
    config.NeutralMode = isBraked ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    isBraked = !isBraked;
    leftFalcon.getConfigurator().apply(config);
    rightFalcon.getConfigurator().apply(config);
  }
}
