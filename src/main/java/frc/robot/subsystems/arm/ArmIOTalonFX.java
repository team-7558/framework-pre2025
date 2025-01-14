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
  private final PositionVoltage posControl;
  private final MotionMagicVoltage ElbowMMPosControl;

  private final double maxPositionDegrees = 90;
  private final double minPositionDegrees = 0; // Set your desired minimum position here

  public ArmIOTalonFX() {
    var ElbowMotorConfig = new TalonFXConfiguration();

    ElbowVoltageControl = new VoltageOut(0);
    ElbowPosControl = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    ElbowMotor = new TalonFX(11);

    // Configure arm ElbowMotor

    ElbowMMVelControl = new MotionMagicVelocityVoltage(0, 0, true, 0, 2, false, false, false);
    posControl = new PositionVoltage(minPositionDegrees, 0, true, 0, 0, false, false, false);
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
    ElbowMotorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    ElbowMotorConfig.Slot0.kG = 0.07;
    ElbowMotorConfig.Slot0.kP = 176;
    ElbowMotorConfig.Slot0.kI = 0;
    ElbowMotorConfig.Slot0.kD = 4; // 16;

    // MotionMagic Position gains
    ElbowMotorConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;
    ElbowMotorConfig.Slot1.kG = 0.07;
    ElbowMotorConfig.Slot1.kV = 38;
    ElbowMotorConfig.Slot1.kS = 0.0;
    ElbowMotorConfig.Slot1.kA = 0; // 1;
    ElbowMotorConfig.Slot1.kP = 23;
    ElbowMotorConfig.Slot1.kI = 0;
    ElbowMotorConfig.Slot1.kD = 0.0;

    // MotionMagic Velocity control
    ElbowMotorConfig.Slot2.GravityType = GravityTypeValue.Elevator_Static;
    ElbowMotorConfig.Slot2.kG = 0;
    ElbowMotorConfig.Slot2.kV = 0.0;
    ElbowMotorConfig.Slot2.kS = 0.0;
    ElbowMotorConfig.Slot2.kA = 0.0;
    ElbowMotorConfig.Slot2.kP = 0.0;
    ElbowMotorConfig.Slot2.kI = 0.0;
    ElbowMotorConfig.Slot2.kD = 0.0;

    ElbowMotor.getConfigurator().apply(ElbowMotorConfig);
  }

  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll();
    inputs.elbow_currents = new double[] {ElbowMotor.getStatorCurrent().getValueAsDouble()};
    inputs.elbow_pos_deg = Units.radiansToDegrees(ElbowMotor.getPosition().getValueAsDouble());
    inputs.elbow_volts = ElbowMotor.getMotorVoltage().getValueAsDouble();
    inputs.elbow_velDegPS = ElbowMotor.getVelocity().getValueAsDouble();
  }

  public void goToAngle(double position_deg, ArmIOInputs inputs, boolean first_time) {
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
}
