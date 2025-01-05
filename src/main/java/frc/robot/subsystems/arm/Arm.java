package frc.robot.subsystems.arm;

import frc.robot.subsystems.StateMachineSubsystemBase;

public class Arm extends StateMachineSubsystemBase<ArmState> {


    private ArmState state;
    private ArmIOSparkMax io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

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

    @Override
    public void handleStateMachine() {


        switch(state) {
            case DISABLED:
                break;

            case HOLDING:
                io.setArmPosition(armPosition);
                break;
            case IDLE:
                io.setArmPosition(armMinimum);
                break;
            case PICKUP:
                io.setArmPosition(armMinimum);
            case ZEROING:
                if(!inputs.arm_halleffect) {
                    io.setArmVoltage(-0.2);
                } else {
                    io.zero();
                    io.setArmVoltage(0);
                    queueState(ArmState.IDLE);
                    zeroed = true;
                }
            default:
                break;

            

        }


    }

    @Override
    public void outputPeriodic() {
        
    }


    public boolean getZeroed() {
        return this.zeroed;
    }
    
}
