package frc.robot.subsystems.claw;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

public class ClawIOTalonFX implements ClawIO {
  private final TalonFX arm_motor;
  private final TalonFX claw_motor;
  private final CANcoder cancoder;

  private final StatusSignal<Double> armAbsolutePosition;
  private final VoltageOut armVoltageControl;
  private final VoltageOut claw_voltage_out;
  private final PositionVoltage armPosControl;

  private final StatusSignal<Double> pos_m;
  private final StatusSignal<Double> vel_mps;
  private final StatusSignal<Double> acc_mps2;
  private final StatusSignal<Double> volts_V;
  private final StatusSignal<Double> leftCurrent_A, rightCurrent_A;

  private final VoltageOut voltageControl;
  private final MotionMagicVelocityVoltage mmVelControl;
  private final PositionVoltage posControl;
  private final MotionMagicVoltage mmPosControl;

  private final double maxPositionDegrees = 90;
  private final double minPositionDegrees = 0; // Set your desired minimum position here

  public ClawIOTalonFX() {
    var armmotorConfig = new TalonFXConfiguration();
    var clawmotorConfig = new TalonFXConfiguration();

    armVoltageControl = new VoltageOut(0);
    armPosControl = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    claw_voltage_out = new VoltageOut(0.0);

    arm_motor = new TalonFX(11);
    claw_motor = new TalonFX(12);
    cancoder = new CANcoder(13);
    // Configure arm arm_motor

    voltageControl = new VoltageOut(0);
    mmVelControl = new MotionMagicVelocityVoltage(0, 0, true, 0, 2, false, false, false);
    posControl = new PositionVoltage(minPositionDegrees, 0, true, 0, 0, false, false, false);
    mmPosControl = new MotionMagicVoltage(minPositionDegrees, true, 0, 1, false, false, false);

    pos_m = arm_motor.getPosition();
    vel_mps = arm_motor.getVelocity();
    acc_mps2 = arm_motor.getAcceleration();
    leftCurrent_A = arm_motor.getStatorCurrent();
    rightCurrent_A = arm_motor.getStatorCurrent();
    volts_V = arm_motor.getMotorVoltage();

    armmotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    armmotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armmotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
    armmotorConfig.Feedback.SensorToMechanismRatio = 5;
    armmotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armmotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    armmotorConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0;
    armmotorConfig.MotionMagic.MotionMagicAcceleration = 1.0;
    armmotorConfig.MotionMagic.MotionMagicJerk = 0;
    armmotorConfig.MotionMagic.MotionMagicExpo_kV = 0.5;
    armmotorConfig.MotionMagic.MotionMagicExpo_kA = 0.5;

    // Position control gains
    armmotorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    armmotorConfig.Slot0.kG = 0.07;
    armmotorConfig.Slot0.kP = 176;
    armmotorConfig.Slot0.kI = 0;
    armmotorConfig.Slot0.kD = 4; // 16;

    // MotionMagic Position gains
    armmotorConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;
    armmotorConfig.Slot1.kG = 0.07;
    armmotorConfig.Slot1.kV = 38;
    armmotorConfig.Slot1.kS = 0.0;
    armmotorConfig.Slot1.kA = 0; // 1;
    armmotorConfig.Slot1.kP = 23;
    armmotorConfig.Slot1.kI = 0;
    armmotorConfig.Slot1.kD = 0.0;

    // MotionMagic Velocity control
    armmotorConfig.Slot2.GravityType = GravityTypeValue.Elevator_Static;
    armmotorConfig.Slot2.kG = 0;
    armmotorConfig.Slot2.kV = 0.0;
    armmotorConfig.Slot2.kS = 0.0;
    armmotorConfig.Slot2.kA = 0.0;
    armmotorConfig.Slot2.kP = 0.0;
    armmotorConfig.Slot2.kI = 0.0;
    armmotorConfig.Slot2.kD = 0.0;

    arm_motor.getConfigurator().apply(armmotorConfig);

    clawmotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    clawmotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    clawmotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
    clawmotorConfig.Feedback.SensorToMechanismRatio = 5;
    clawmotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    claw_motor.getConfigurator().apply(clawmotorConfig);

    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    armAbsolutePosition = cancoder.getAbsolutePosition();
  }

  public void updateInputs(ClawIOInputs inputs) {
    BaseStatusSignal.refreshAll();
    inputs.arm_currents_A = new double[] {arm_motor.getStatorCurrent().getValueAsDouble()};
    inputs.arm_pos_deg = Units.radiansToDegrees(arm_motor.getPosition().getValueAsDouble());
    inputs.arm_volts_V = arm_motor.getMotorVoltage().getValueAsDouble();
    inputs.arm_velocity_DegPS = arm_motor.getVelocity().getValueAsDouble();

    inputs.arm_absolute_pos_deg = armAbsolutePosition.getValueAsDouble();

    inputs.claw_currents_A = new double[] {arm_motor.getStatorCurrent().getValueAsDouble()};
    inputs.claw_volts_V = arm_motor.getMotorVoltage().getValueAsDouble();
    inputs.claw_velocity_degps = arm_motor.getVelocity().getValueAsDouble();
  }

  public void goToAngle(double position_deg, boolean first_time) {
    double rotations = position_deg / 360;
    arm_motor.setControl(
        armPosControl.withPosition(MathUtil.clamp(rotations, 35, maxPositionDegrees)));
  }

  public void setArmVoltage(double volts) {
    arm_motor.setControl(armVoltageControl.withOutput(volts));
  }

  public void ClawVoltage(double volts) {
    // Clamp voltage to valid range (-12V to 12V)
    volts = MathUtil.clamp(volts, -12.0, 12.0);

    // Set motor voltage
    claw_motor.setControl(claw_voltage_out.withOutput(volts));
  }

  @Override
  public void stop_arm() {
    setArmVoltage(0.0);
  }

  @Override
  public void stop_claw() {
    setClawVoltage(0.0);
  }
}
