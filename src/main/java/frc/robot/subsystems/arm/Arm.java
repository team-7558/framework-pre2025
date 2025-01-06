package frc.robot.subsystems.arm;

import frc.robot.subsystems.StateMachineSubsystemBase;

public class Arm extends StateMachineSubsystemBase<ArmState> {

  private ArmState state;
  private ArmIOSparkMax io;
  private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private static Arm instance;

  private double armPosition = 0;
  private double wristPosition = 0;
  private double wheelsVelocity = 0;

  private double armMinimum = 40;
  private double armMax = 150;
  private double armZeroing = 0;

  private boolean zeroed = false;

  public Arm(String name) {
    super(name);

    io = new ArmIOSparkMax();
    state = ArmState.ZEROING;
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

  @Override
  public void handleStateMachine() {

    switch (state) {
      case DISABLED:
        break;

      case HOLDING:
        // io.setArmPosition(armPosition);
        break;
      case IDLE:
        System.out.println("in idle setting arm pos");
        io.setArmPosition(45);
        // io.setArmPosition(armMinimum);
        break;
      case PICKUP:
        // io.setArmPosition(armMinimum);
      case ZEROING:
        io.zero();
        io.setArmVoltage(0);
        queueState(ArmState.IDLE);
        // System.out.println("zeroed? and queuing");
        break;
      case MANUAL:
        // io.setArmVoltage(OI.DR.getXButton() ? 1.5 : 0);
        // io.setArmVoltage(OI.DR.getYButton() ? -1.5 : 0);
        break;
      default:
        break;
    }
  }

  @Override
  public void outputPeriodic() {}

  public boolean getZeroed() {
    return this.zeroed;
  }
}
