package frc.robot.subsystems.arm;

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

public class ArmIOTalonFX implements ArmIO {

  // Arm ElbowMotor
  private final TalonFX ElbowMotor;
  private final VoltageOut ElbowVoltageControl;
  private final PositionVoltage ElbowPosControl;

  private final StatusSignal<Double> ElbowPos_Deg;
  private final StatusSignal<Double> ElbowVel_degps;
  private final StatusSignal<Double> ElbowAcc_degps2;
  private final StatusSignal<Double> ElbowVolts_V;
  private final StatusSignal<Double> ElbowCurrent_A;

  private final MotionMagicVelocityVoltage ElbowMMVelControl;
  private final PositionVoltage posControlthing;
  private final MotionMagicVoltage ElbowMMPosControl;

  private final TalonFX ShoulderMotor;
  private final VoltageOut ShoulderVoltageControl;
  private final PositionVoltage ShoulderPosControl;

  private final StatusSignal<Double> ShoulderPos_Deg;
  private final StatusSignal<Double> ShoulderVel_degps;
  private final StatusSignal<Double> ShoulderAcc_degps2;
  private final StatusSignal<Double> ShoulderVolts_V;
  private final StatusSignal<Double> ShoulderCurrent_A;

  private final MotionMagicVelocityVoltage ShoulderMMVelControl;
  private final PositionVoltage ShoulderposControlthing;
  private final MotionMagicVoltage ShoulderMMPosControl;

  private final double maxPositionDegrees = 90;
  private final double minPositionDegrees = 0; // Set your desired minimum position here

  public ArmIOTalonFX() {
    var ElbowMotorConfig = new TalonFXConfiguration();

    ElbowVoltageControl = new VoltageOut(0);
    ElbowPosControl = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    ElbowMotor = new TalonFX(11);

    // Configure arm ElbowMotor

    ElbowMMVelControl = new MotionMagicVelocityVoltage(0, 0, true, 0, 2, false, false, false);
    posControlthing = new PositionVoltage(minPositionDegrees, 0, true, 0, 0, false, false, false);
    ElbowMMPosControl = new MotionMagicVoltage(minPositionDegrees, true, 0, 1, false, false, false);

    ElbowPos_Deg = ElbowMotor.getPosition();
    ElbowVel_degps = ElbowMotor.getVelocity();
    ElbowAcc_degps2 = ElbowMotor.getAcceleration();
    ElbowCurrent_A = ElbowMotor.getStatorCurrent();
    ElbowVolts_V = ElbowMotor.getMotorVoltage();

    ElbowMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    ElbowMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    ElbowMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
    ElbowMotorConfig.Feedback.SensorToMechanismRatio = 5;
    ElbowMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    ElbowMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    ElbowMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0;
    ElbowMotorConfig.MotionMagic.MotionMagicAcceleration = 1.0;
    ElbowMotorConfig.MotionMagic.MotionMagicJerk = 0;
    ElbowMotorConfig.MotionMagic.MotionMagicExpo_kV = 0.5;
    ElbowMotorConfig.MotionMagic.MotionMagicExpo_kA = 0.5;

    // Position control gains
    ElbowMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    ElbowMotorConfig.Slot0.kG = 0.07;
    ElbowMotorConfig.Slot0.kP = 176;
    ElbowMotorConfig.Slot0.kI = 0;
    ElbowMotorConfig.Slot0.kD = 4; // 16;

    // MotionMagic Position gains
    ElbowMotorConfig.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
    ElbowMotorConfig.Slot1.kG = 0.07;
    ElbowMotorConfig.Slot1.kV = 38;
    ElbowMotorConfig.Slot1.kS = 0.0;
    ElbowMotorConfig.Slot1.kA = 0; // 1;
    ElbowMotorConfig.Slot1.kP = 23;
    ElbowMotorConfig.Slot1.kI = 0;
    ElbowMotorConfig.Slot1.kD = 0.0;

    // MotionMagic Velocity control
    ElbowMotorConfig.Slot2.GravityType = GravityTypeValue.Arm_Cosine;
    ElbowMotorConfig.Slot2.kG = 0;
    ElbowMotorConfig.Slot2.kV = 0.0;
    ElbowMotorConfig.Slot2.kS = 0.0;
    ElbowMotorConfig.Slot2.kA = 0.0;
    ElbowMotorConfig.Slot2.kP = 0.0;
    ElbowMotorConfig.Slot2.kI = 0.0;
    ElbowMotorConfig.Slot2.kD = 0.0;

    ElbowMotor.getConfigurator().apply(ElbowMotorConfig);

    var ShoulderMotorConfig = new TalonFXConfiguration();

    ShoulderVoltageControl = new VoltageOut(0);
    ShoulderPosControl = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    ShoulderMotor = new TalonFX(12);

    // Configure arm ElbowMotor

    ShoulderMMVelControl = new MotionMagicVelocityVoltage(0, 0, true, 0, 2, false, false, false);
    ShoulderposControlthing =
        new PositionVoltage(minPositionDegrees, 0, true, 0, 0, false, false, false);
    ShoulderMMPosControl =
        new MotionMagicVoltage(minPositionDegrees, true, 0, 1, false, false, false);

    ShoulderPos_Deg = ElbowMotor.getPosition();
    ShoulderVel_degps = ElbowMotor.getVelocity();
    ShoulderAcc_degps2 = ElbowMotor.getAcceleration();
    ShoulderCurrent_A = ElbowMotor.getStatorCurrent();
    ShoulderVolts_V = ElbowMotor.getMotorVoltage();

    ShoulderMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    ShoulderMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    ShoulderMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
    ShoulderMotorConfig.Feedback.SensorToMechanismRatio = 5;
    ShoulderMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    ShoulderMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    ShoulderMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0;
    ShoulderMotorConfig.MotionMagic.MotionMagicAcceleration = 1.0;
    ShoulderMotorConfig.MotionMagic.MotionMagicJerk = 0;
    ShoulderMotorConfig.MotionMagic.MotionMagicExpo_kV = 0.5;
    ShoulderMotorConfig.MotionMagic.MotionMagicExpo_kA = 0.5;

    // Position control gains
    ShoulderMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    ShoulderMotorConfig.Slot0.kG = 0.07;
    ShoulderMotorConfig.Slot0.kP = 176;
    ShoulderMotorConfig.Slot0.kI = 0;
    ShoulderMotorConfig.Slot0.kD = 4; // 16;

    // MotionMagic Position gains
    ShoulderMotorConfig.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
    ShoulderMotorConfig.Slot1.kG = 0.07;
    ShoulderMotorConfig.Slot1.kV = 38;
    ShoulderMotorConfig.Slot1.kS = 0.0;
    ShoulderMotorConfig.Slot1.kA = 0; // 1;
    ShoulderMotorConfig.Slot1.kP = 23;
    ShoulderMotorConfig.Slot1.kI = 0;
    ShoulderMotorConfig.Slot1.kD = 0.0;

    // MotionMagic Velocity control
    ShoulderMotorConfig.Slot2.GravityType = GravityTypeValue.Arm_Cosine;
    ShoulderMotorConfig.Slot2.kG = 0;
    ShoulderMotorConfig.Slot2.kV = 0.0;
    ShoulderMotorConfig.Slot2.kS = 0.0;
    ShoulderMotorConfig.Slot2.kA = 0.0;
    ShoulderMotorConfig.Slot2.kP = 0.0;
    ShoulderMotorConfig.Slot2.kI = 0.0;
    ShoulderMotorConfig.Slot2.kD = 0.0;

    ShoulderMotor.getConfigurator().apply(ShoulderMotorConfig);
  }

  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll();
    inputs.elbow_current_A = new double[] {ElbowMotor.getStatorCurrent().getValueAsDouble()};
    inputs.elbow_pos_deg = Units.radiansToDegrees(ElbowMotor.getPosition().getValueAsDouble());
    inputs.elbow_volts_V = ElbowMotor.getMotorVoltage().getValueAsDouble();
    inputs.elbow_vel_degps = ElbowMotor.getVelocity().getValueAsDouble();

    inputs.shoulder_current_A = new double[] {ShoulderMotor.getStatorCurrent().getValueAsDouble()};
    inputs.shoulder_pos_deg =
        Units.radiansToDegrees(ShoulderMotor.getPosition().getValueAsDouble());
    inputs.shoulder_volts_V = ShoulderMotor.getMotorVoltage().getValueAsDouble();
    inputs.shoulder_vel_degps = ShoulderMotor.getVelocity().getValueAsDouble();
  }

  public void goToElbowAngle(double position_deg, ArmIOInputs inputs, boolean first_time) {
    double rotations = position_deg / 360;
    ElbowMotor.setControl(
        ElbowPosControl.withPosition(MathUtil.clamp(rotations, 35, maxPositionDegrees)));
  }

  public void setElbowVoltage(double volts) {
    ElbowMotor.setControl(ElbowVoltageControl.withOutput(volts));
  }

  @Override
  public void stopElbow() {
    setElbowVoltage(0.0);
  }

  public void goToShoulderAngle(
    double position_deg, ArmIOInputs inputs, boolean first_time, double volts) {
    double rotations = position_deg / 360;
    ShoulderMotor.setControl(
        ShoulderPosControl.withPosition(MathUtil.clamp(rotations, 35, maxPositionDegrees))
            .withFeedForward(volts));
  }

  public void setShoulderVoltage(double volts) {
    ShoulderMotor.setControl(ShoulderVoltageControl.withOutput(volts));
  }

  @Override
  public void stopShoulder() {
    setShoulderVoltage(0.0);
  }
}
