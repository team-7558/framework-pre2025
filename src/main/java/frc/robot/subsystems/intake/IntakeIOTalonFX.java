package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOTalonFX implements IntakeIO {

  // Arm motor
  private final TalonFX ArmMotor;
  private final TalonFX IntakeMotor;
  private final VoltageOut armVoltageControl;
  private final VoltageOut Intake_voltage_out;
  private final PositionVoltage armPosControl;

  private final DigitalInput beambreak;

  private final StatusSignal<Double> pos_m;
  private final StatusSignal<Double> vel_mps;
  private final StatusSignal<Double> acc_mps2;
  private final StatusSignal<Double> volts_V;
  private final StatusSignal<Double> leftCurrent_A, rightCurrent_A;

  private final boolean isLeftMotorInverted = true;
  private final VoltageOut voltageControl;
  private final MotionMagicVelocityVoltage mmVelControl;
  private final PositionVoltage posControl;
  private final MotionMagicVoltage mmPosControl;

  private final double maxPositionDegrees = 90;
  private final double minPositionDegrees = 0; // Set your desired minimum position here

  public IntakeIOTalonFX() {
    var motorConfig = new TalonFXConfiguration();

    armVoltageControl = new VoltageOut(0);
    armPosControl = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    ArmMotor = new TalonFX(11);

    // Configure arm motor

    voltageControl = new VoltageOut(0);
    mmVelControl = new MotionMagicVelocityVoltage(0, 0, true, 0, 2, false, false, false);
    posControl = new PositionVoltage(minPositionDegrees, 0, true, 0, 0, false, false, false);
    mmPosControl = new MotionMagicVoltage(minPositionDegrees, true, 0, 1, false, false, false);

    pos_m = ArmMotor.getPosition();
    vel_mps = ArmMotor.getVelocity();
    acc_mps2 = ArmMotor.getAcceleration();
    leftCurrent_A = ArmMotor.getStatorCurrent();
    rightCurrent_A = ArmMotor.getStatorCurrent();
    volts_V = ArmMotor.getMotorVoltage();

    motorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
    motorConfig.Feedback.SensorToMechanismRatio = 5;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0;
    motorConfig.MotionMagic.MotionMagicAcceleration = 1.0;
    motorConfig.MotionMagic.MotionMagicJerk = 0;
    motorConfig.MotionMagic.MotionMagicExpo_kV = 0.5;
    motorConfig.MotionMagic.MotionMagicExpo_kA = 0.5;

    // Position control gains
    motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    motorConfig.Slot0.kG = 0.07;
    motorConfig.Slot0.kP = 176;
    motorConfig.Slot0.kI = 0;
    motorConfig.Slot0.kD = 4; // 16;

    // MotionMagic Position gains
    motorConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;
    motorConfig.Slot1.kG = 0.07;
    motorConfig.Slot1.kV = 38;
    motorConfig.Slot1.kS = 0.0;
    motorConfig.Slot1.kA = 0; // 1;
    motorConfig.Slot1.kP = 23;
    motorConfig.Slot1.kI = 0;
    motorConfig.Slot1.kD = 0.0;

    // MotionMagic Velocity control
    motorConfig.Slot2.GravityType = GravityTypeValue.Elevator_Static;
    motorConfig.Slot2.kG = 0;
    motorConfig.Slot2.kV = 0.0;
    motorConfig.Slot2.kS = 0.0;
    motorConfig.Slot2.kA = 0.0;
    motorConfig.Slot2.kP = 0.0;
    motorConfig.Slot2.kI = 0.0;
    motorConfig.Slot2.kD = 0.0;

    ArmMotor.getConfigurator().apply(motorConfig);
    ArmMotor.getConfigurator().apply(motorConfig);

    IntakeMotor = new TalonFX(13);
    var IntakemotorConfig = new TalonFXConfiguration();
    beambreak = new DigitalInput(0);

    IntakemotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    IntakemotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    IntakemotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
    IntakemotorConfig.Feedback.SensorToMechanismRatio = 5;
    IntakemotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    IntakeMotor.getConfigurator().apply(motorConfig);

    // Initialize voltage control
    Intake_voltage_out = new VoltageOut(0.0);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll();
    inputs.slap_currents = new double[] {ArmMotor.getStatorCurrent().getValueAsDouble()};
    inputs.slap_pos_deg = Units.radiansToDegrees(ArmMotor.getPosition().getValueAsDouble());
    inputs.slap_volts = ArmMotor.getMotorVoltage().getValueAsDouble();
    inputs.slap_velDegPS = ArmMotor.getVelocity().getValueAsDouble();
  }

  public void goToAngle(double position_deg, IntakeIOInputs inputs, boolean first_time) {
    double rotations = position_deg / 360;
    ArmMotor.setControl(
        armPosControl.withPosition(MathUtil.clamp(rotations, 35, maxPositionDegrees)));
  }

  public void setIntakeVoltage(double volts) {
    // Clamp voltage to valid range (-12V to 12V)
    volts = MathUtil.clamp(volts, -12.0, 12.0);

    // Set motor voltage
    IntakeMotor.setControl(Intake_voltage_out.withOutput(volts));
  }

  @Override
  public void stopIntake() {
    setIntakeVoltage(0.0);
  }

  public void setArmVoltage(double volts) {
    ArmMotor.setControl(armVoltageControl.withOutput(volts));
  }

  @Override
  public void stopArm() {
    setArmVoltage(0.0);
  }
}
