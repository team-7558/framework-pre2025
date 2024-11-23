package frc.robot.subsystems.pinkarm;

import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.subsystems.drive.PathingMode;

// Renamed to follow Java naming conventions
public class pinkarm {
    private static pinkarm instance;
    private final pinkarmIO io;
    State currState;
    private final pinkarm2d mech = new pinkarm2d();

    private final pinkarmInputsAutoLogged inputs = new pinkarmInputsAutoLogged();
    private double targetHeight_m; // Added missing variable

    public enum State {
        TRAVELLING,
        HOLDING,
        IDLE,
        DISABLED
    }

    public static pinkarm getInstance() {
        if (instance == null) {
            System.out.println("pinkarm initialized");
            switch (Constants.currentMode) {
                case REAL:
                    // Real robot, instantiate hardware IO implementations
                    break;

                case SIM:
                    // Sim robot, instantiate physics sim IO implementations
                    instance = new pinkarm(new pinkarmIOSim());
                    break;

                default:
                    // Replayed robot, disable IO implementations
                    break;
            }
        }
        return instance;
    }

    private pinkarm(pinkarmIO io) {
        this.io = io;
        currState = State.IDLE;
    }


    public void setcurrentState(State state) {
        currState = state;
    }

    public void set(double meters) {
        if (io != null) {
            io.goToPos(meters);
        }
    }

    // Added a setter for targetHeight_m
    public void setTargetHeight(double targetHeight_m) {
        this.targetHeight_m = targetHeight_m;
    }
    
    public void inputPeriodic() {
        if (io != null) {
            io.updateInputs(inputs);
            Logger.processInputs("pinkarm", inputs);
        }
    }

    public void outputPeriodic() {
        if (io != null) {
            mech.setLength(inputs.elev_posMeters);
            mech.setLength(targetHeight_m);
            Logger.recordOutput("pinkarm/TargetHeight_m", targetHeight_m);
        }
    }
    
    public void periodic() {
        switch(currState) {
            case DISABLED:
                io.stop();
                break;
            case IDLE:
                io.stop();
                break;
            case TRAVELLING:
                io.goToPos(targetHeight_m);
                break;
            case HOLDING:
                break;
        }
    }

}
