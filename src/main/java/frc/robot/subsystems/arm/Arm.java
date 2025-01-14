package frc.robot.subsystems.arm;

import frc.robot.subsystems.StateMachineSubsystemBase;

public class Arm extends StateMachineSubsystemBase<ArmState> {

  private ArmIOSparkMax io;
  private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private static Arm instance;

  private double armPosition = 0;
  private double wristPosition = 0;
  private double wheelsVelocity = 0;

  private double armMinimum = 15;
  private double armScoring = 30;
  private double armZeroing = 0;

  private double THRESHOLD = 30; // amps

  public double armIntaking = 10;

  private double maxDutyCycle = 0.5;

  private boolean zeroed = false;

  public Arm(String name) {
    super(name);

    io = new ArmIOSparkMax();
    queueState(ArmState.IDLE);
  }

  @Override
  public void inputPeriodic() {
    io.updateInputs(inputs);
  }

  public static Arm getInstance() {
    if (instance == null) {
      instance = new Arm("arm");
    }
    return instance;
  }

  public void zero() {
    io.zero();
  }

  public void setClaw(boolean a) {
    io.setSolenoid(a);
  }

  @Override
  public void handleStateMachine() {

    // System.out.println(state.name());

    switch (getState()) {
      case DISABLED:
        break;

      case IDLE:
        armPosition = armMinimum;
        io.setArmPosition(armPosition, maxDutyCycle);
        // io.setSolenoid(true);
        io.runWheels(wheelsVelocity);
        break;
      case HOLDING_PIECE:
        // io.runWheels(-1);
        // io.setSolenoid(true);
        io.setArmPosition(armPosition, maxDutyCycle);
        io.runWheels(wheelsVelocity);
        break;
      case SPITTING:
        // io.runWheels(-1);
        // io.setSolenoid(true);
        io.setArmPosition(armPosition, maxDutyCycle);
        io.runWheels(wheelsVelocity);
      case ZEROING:
        // System.out.println(inputs.arm_current_A[0]);
        // if (inputs.arm_current_A[0] < THRESHOLD) {
        //   zeroed = false;
        //   io.setArmVoltage(-1.5);
        // } else {
        io.zero();
        zeroed = true;
        queueState(ArmState.IDLE);
        // }

        break;
      case INTAKING:
        // io.runWheels(1);
        io.setArmPosition(armIntaking);
        // io.setSolenoid(false);
        break;

      case MANUAL:
        // io.setArmVoltage(OI.DR.getXButton() ? 1.5 : 0);
        // io.setArmVoltage(OI.DR.getYButton() ? -1.5 : 0);
        break;
      default:
        break;
    }
  }

  public void setArmTarget(double a) {
    this.armPosition = a;
    this.maxDutyCycle = 0.5;
  }

  public void setArmTarget(double a, double maxDutyCycle) {
    this.armPosition = a;
    this.maxDutyCycle = maxDutyCycle;
  }

  public double getArmPosition() {
    return inputs.arm_position_r;
  }

  public void setWheels(double wheels) {
    this.wheelsVelocity = wheels;
  }

  @Override
  public void outputPeriodic() {}

  public boolean getZeroed() {
    return this.zeroed;
  }
}
