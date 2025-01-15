package frc.robot.subsystems.hangV2;

import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.util.Units;

public class Hang extends StateMachineSubsystemBase<HangStates> {
    private final HangIO io;
    private final HangIOInputsAutoLogged inputs = new HangIOInputsAutoLogged();
    private static Hang instance;
    private boolean running = false;

    public Hang(HangIO io) {
        super("Hang");
        this.io = io;
        queueState(HangStates.IDLE);
    }

    @Override
    public void inputPeriodic() {
        io.updateInputs(inputs);
        //Logger.processInput("hang", inputs);
    }

    public static Hang getInstance() {
        if (instance == null) {
            switch (Constants.currentMode) {
                case SIM:
                    instance = new Hang(new HangIOSim());
                    break;
                case REAL:
                    instance = new Hang(new HangIOTalonFX());
                    break;
                default:
                    break;
            }
        }
        return instance;
    }

    @Override
    public void handleStateMachine() {
        switch (getState()) {
            case DISABLED:
                break;
            case IDLE:
                running = false;
                break;
            case TRAVELING:
                
        }
    }

    @Override
    public void outputPeriodic() {
        
    }
}
