package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;
import frc.robot.util.Util;

public class ArmIOSparkMax {

  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Arm");

  // arm motor
  private final CANSparkMax armMotor = new CANSparkMax(11, MotorType.kBrushless);
  private final RelativeEncoder armEncoder = armMotor.getEncoder();
  private final DigitalInput m_hallEffect = new DigitalInput(1);
  private final SparkPIDController armPid = armMotor.getPIDController();

  private final CANSparkMax wheelsMotor = new CANSparkMax(12, MotorType.kBrushless);

  // private final CANSparkMax wristMotor = new CANSparkMax(13, MotorType.kBrushless);
  // private final RelativeEncoder wristEncoder = wristMotor.getEncoder();
  // private final SparkPIDController wristPid = wristMotor.getPIDController();

  double armZeroOffset = 0;

  double max = 0;

  public ArmIOSparkMax() {

    // set gains

    armMotor.setIdleMode(IdleMode.kBrake);

    armMotor.setInverted(true);
    armPid.setOutputRange(-0.5, 0.5);

    armEncoder.setPosition(0);
    // wristEncoder.setPosition(0);

    armPid.setP(0.05);
    armPid.setI(0);
    armPid.setD(0);

    // wristPid.setP(0.15);
    // wristPid.setI(0);
    // wristPid.setD(0);
    // wristPid.setOutputRange(-0.5, 0.5);
  }

  public void updateInputs(ArmIOInputs inputs) {

    // System.out.println(m_hallEffect.get());
    inputs.arm_current_A = new double[] {armMotor.getOutputCurrent()};
    inputs.arm_halleffect = m_hallEffect.get();
    inputs.arm_position_r = armEncoder.getPosition();
    inputs.arm_volts_V = armMotor.getBusVoltage();
    inputs.arm_velocity_rps = armEncoder.getVelocity();

    inputs.wheels_current_A = new double[] {wheelsMotor.getOutputCurrent()};
    inputs.wheels_velocity_rps = wheelsMotor.getEncoder().getVelocity();
    inputs.wheels_volts_V = wheelsMotor.getBusVoltage();

    // inputs.wrist_current_A = new double[] {wristMotor.getOutputCurrent()};
    // inputs.wrist_position_r = wristEncoder.getPosition();
    // inputs.wrist_velocity_rps = wristEncoder.getVelocity();
    // inputs.wrist_volts_V = wristMotor.getBusVoltage();
  }

  public void setArmPosition(double position) {

    double rotations = position / 4;
    armPid.setReference(Util.limit(rotations, 35, max), ControlType.kPosition);
  }

  public void setWristAngle(double angle) {
    // double rotations = angle / 5.8;
    // wristPid.setReference(Util.limit(rotations, 0, 180 / 5.8), ControlType.kPosition);
  }

  public void setArmVoltage(double volts) {
    armMotor.setVoltage(volts);
  }

  public void zero() {
    armEncoder.setPosition(0);
  }
}
