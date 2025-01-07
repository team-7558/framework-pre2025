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
import org.littletonrobotics.junction.Logger;

public class ElevatorIOTalonFX implements ElevatorIO {

  private final TalonFX motor;
  private final DigitalInput hallEffect;

  private boolean isBraked = true;
  private final StatusSignal<Double> pos_m;
  private final StatusSignal<Double> vel_mps;
  private final StatusSignal<Double> acc_mps2;
  private final StatusSignal<Double> volts_V;
  private final StatusSignal<Double> leftCurrent_A, rightCurrent_A;

  private final VoltageOut voltageControl;
  private final MotionMagicVelocityVoltage mmVelControl;
  private final PositionVoltage posControl;
  private final MotionMagicVoltage mmPosControl;

  private double posOffset_m = 0.0;

  public ElevatorIOTalonFX() {
    System.out.println("ELEVATOR CTOR");
    var motorConfig = new TalonFXConfiguration();

    voltageControl = new VoltageOut(0);
    mmVelControl = new MotionMagicVelocityVoltage(0, 0, false, 0, 2, false, false, false);
    posControl = new PositionVoltage(0, 0, false, 0, 0, false, false, false);
    mmPosControl = new MotionMagicVoltage(0, false, 0, 1, false, false, false);

    motor = new TalonFX(16);
    hallEffect = new DigitalInput(2);

    pos_m = motor.getPosition();
    vel_mps = motor.getVelocity();
    acc_mps2 = motor.getAcceleration();
    leftCurrent_A = motor.getStatorCurrent();
    rightCurrent_A = motor.getStatorCurrent();
    volts_V = motor.getMotorVoltage();

    motorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
    motorConfig.Feedback.SensorToMechanismRatio = 16.0;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 2.5;
    motorConfig.MotionMagic.MotionMagicAcceleration = 2.5;
    motorConfig.MotionMagic.MotionMagicJerk = 0;
    motorConfig.MotionMagic.MotionMagicExpo_kV = 0.9;
    motorConfig.MotionMagic.MotionMagicExpo_kA = 0.5;

    // Position control gains
    motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    motorConfig.Slot0.kG = 0;
    motorConfig.Slot0.kP = 0.1;
    motorConfig.Slot0.kI = 0;
    motorConfig.Slot0.kD = 0; // 16;

    // MotionMagic Position gains
    motorConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;
    motorConfig.Slot1.kG = 0.2;
    motorConfig.Slot1.kV = 1.6;
    motorConfig.Slot1.kS = 0.0;
    motorConfig.Slot1.kA = 0; // 1;
    motorConfig.Slot1.kP = 8;
    motorConfig.Slot1.kI = 0.0;
    motorConfig.Slot1.kD = 0.0;

    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motor.getConfigurator().apply(motorConfig);
    // motor.setControl(new Follower(motor.getDeviceID(), false).withUpdateFreqHz(50));
    BaseStatusSignal.setUpdateFrequencyForAll(
        1.0 / 0.015, pos_m, vel_mps, volts_V, leftCurrent_A, rightCurrent_A);
    motor.optimizeBusUtilization();

    resetPos(0);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(pos_m, vel_mps, acc_mps2, volts_V, leftCurrent_A, rightCurrent_A);
    inputs.position_m = pos_m.getValueAsDouble() + posOffset_m;
    inputs.volts_V = volts_V.getValueAsDouble();
    inputs.velocity_mps = vel_mps.getValueAsDouble();
    inputs.current_A =
        new double[] {leftCurrent_A.getValueAsDouble(), rightCurrent_A.getValueAsDouble()};
    inputs.hallEffectHit = !hallEffect.get();
  }

  @Override
  public void setVelocity(double vel_mps) {
    double v = MathUtil.clamp(vel_mps, -0.3, 0.3);
    motor.setControl(mmVelControl.withVelocity(v));
  }

  /*
  @Override
  public void holdPos(double position) {
    motor.setControl(posControl.withPosition(position - posOffset_m));
    motor.setControl(posControl.withPosition(position - posOffset_m));
  }

  */

  @Override
  public void travelToPos(double position) {
    motor.setControl(mmPosControl.withPosition(position));
  }

  @Override
  public void setVoltage(double volts_V) {
    motor.setControl(voltageControl.withOutput(volts_V));
  }

  @Override
  public void resetPos(double pos_m) {
    posOffset_m = -this.pos_m.getValueAsDouble() + pos_m;
    Logger.recordOutput("Elevator/posOffset", posOffset_m);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void toggleBrake() {
    var config = new MotorOutputConfigs();

    config.Inverted = InvertedValue.Clockwise_Positive;
    config.NeutralMode = isBraked ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    isBraked = !isBraked;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void zero() {
    motor.setPosition(0);
  }
}
