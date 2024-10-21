package frc.robot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.Drive;

public class SS {

    public enum State {
        DISABLED,
        IDLE,
        BOOT
    }

    private static SS instance;

    public static SS getInstance() {
        return instance == null ? instance = new SS() : instance;
    }

    private Timer timer;

    private Drive drive;

    private State lastState;
    private State currState;
    private State nextState;

    private SS() {
        lastState = State.DISABLED;
        currState = State.DISABLED;
        nextState = State.DISABLED;

        timer = new Timer();
        
        // drive = drive.getInstance();
        // ^^ add later

    }

    public void queueState(State s) {
        if(currState != s) {
            nextState = s;
        }
    }

    private void logStates() {
        Logger.recordOutput("SS/lastState", lastState);
        Logger.recordOutput("SS/currState", lastState);
        Logger.recordOutput("SS/nextState", lastState);
        Logger.recordOutput("SS/stateTime", timer.get());
    }

    public void periodic() {
        boolean first = currState != lastState;
        lastState = currState;
        if(currState != nextState) {
            timer.restart();
            currState = nextState;
        }

        switch(currState) {
            case DISABLED:
                if(first) {
                    // set states to disabled once i move subsystems to statemachines
                }
        }
    }
    
}
