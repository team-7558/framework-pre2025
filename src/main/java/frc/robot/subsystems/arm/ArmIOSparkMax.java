package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;
import frc.robot.util.Util;

public class ArmIOSparkMax {

  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Arm");

  // arm motor
  // private final CANSparkMax armMotor = new CANSparkMax(11, MotorType.kBrushless);
  // private final RelativeEncoder armEncoder = armMotor.getEncoder();
  // private final DigitalInput m_hallEffect = new DigitalInput(1);
  private final PIDController armPid = new PIDController(0.1, 0, 0);

  // private final CANSparkMax wheelsMotor = new CANSparkMax(12, MotorType.kBrushless);

  // private PneumaticHub revPH = new PneumaticHub(24);
  // private Compressor compressor = new Compressor(24, PneumaticsModuleType.REVPH);

  // private Solenoid solenoid = new Solenoid(24, PneumaticsModuleType.REVPH, 1);

  // private final CANSparkMax wristMotor = new CANSparkMax(13, MotorType.kBrushless);
  // private final RelativeEncoder wristEncoder = wristMotor.getEncoder();
  // private final SparkPIDController wristPid = wristMotor.getPIDController();

  double armZeroOffset = 0;

  double max = 0;

  public ArmIOSparkMax() {
    // revPH.enableCompressorDigital();
    // revPH.enableCompressorAnalog(100, 120);

    // set gains

    // armMotor.setIdleMode(IdleMode.kBrake);

    // armMotor.setInverted(true);

    // armEncoder.setPosition(0);
    // wristEncoder.setPosition(0);

    armPid.setP(0.15);
    armPid.setI(0);
    armPid.setD(0);
    armPid.setTolerance(0.2);

    // wristPid.setP(0.15);
    // wristPid.setI(0);
    // wristPid.setD(0);
    // wristPid.setOutputRange(-0.5, 0.5);
  }

  public void updateInputs(ArmIOInputs inputs) {

    // System.out.println(m_hallEffect.get());
    // inputs.arm_current_A = new double[] {armMotor.getOutputCurrent()};
    // inputs.arm_halleffect = m_hallEffect.get();
    // inputs.arm_position_r = armEncoder.getPosition();
    // inputs.arm_volts_V = armMotor.getBusVoltage();
    // inputs.arm_velocity_rps = armEncoder.getVelocity();

    // inputs.wheels_current_A = new double[] {wheelsMotor.getOutputCurrent()};
    // inputs.wheels_velocity_rps = wheelsMotor.getEncoder().getVelocity();
    // inputs.wheels_volts_V = wheelsMotor.getBusVoltage();

    // inputs.wrist_current_A = new double[] {wristMotor.getOutputCurrent()};
    // inputs.wrist_position_r = wristEncoder.getPosition();
    // inputs.wrist_velocity_rps = wristEncoder.getVelocity();
    // inputs.wrist_volts_V = wristMotor.getBusVoltage();
    // System.out.println(inputs.arm_position_r);
  }

  public void setArmPosition(double position) {
    // double currentPosition = armEncoder.getPosition();

    // double output = armPid.calculate(currentPosition, position);
    // output = Math.max(-0.4, Math.min(0.4, output));

    // double rotations = position / 2;
    // System.out.println("output to motor: " + output);
    // armMotor.set(output);
  }

  public void setArmPosition(double position, double maxDutyCycle) {
    position = Util.limit(position, 30);
    // double currentPosition = armEncoder.getPosition();

    // double output = armPid.calculate(currentPosition, position);
    // output = Math.max(-maxDutyCycle, Math.min(maxDutyCycle, output));

    // double rotations = position / 2;
    // System.out.println("output to motor: " + output);
    // armMotor.set(output);
  }

  public void setWristAngle(double angle) {
    // double rotations = angle / 5.8;
    // wristPid.setReference(Util.limit(rotations, 0, 180 / 5.8), ControlType.kPosition);
  }

  public void setArmVoltage(double volts) {
    // armMotor.setVoltage(volts);
  }

  public void runWheels(double volts) {
    // wheelsMotor.set(volts);
  }

  public void zero() {
    // armEncoder.setPosition(0);
  }

  public void setSolenoid(boolean on) {
    // solenoid.set(on);
  }
}
