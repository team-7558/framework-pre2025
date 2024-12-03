package frc.robot.subsystems.vision;

import frc.robot.subsystems.StateMachineSubsystemBase;

public class Vision extends StateMachineSubsystemBase<VisionState> {

    VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();
    VisionIO vision;

    public Vision(String name) {
        super(name);
        vision = new VisionIOLimelight("limelightthreeg");
    }

    @Override
    public void handleStateMachine() {
        switch(getState()) {
            case DISABLED:
                // nothing
                break;

            case PERFORMANCE:
                // pose estimation but fast (maybe teleop driver assist, stuff that isnt crazy precise)
                break;

            case POSE_ESTIMATION:
                // full field tracking (eg if theres an auto race and collision is detected we need to be accurate to regain accurate pose)
                break;
            
            default:
                break;
        }

    }

    @Override
    protected void outputPeriodic() {
        
    }

    @Override
    protected void inputPeriodic() {
        vision.updateInputs(visionInputs);
    }
    
}
