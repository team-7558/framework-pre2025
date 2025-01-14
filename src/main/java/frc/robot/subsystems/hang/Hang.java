package frc.robot.subsystems.hang;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.StateMachineSubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import org.littletonrobotics.junction.Logger;

public class Hang extends StateMachineSubsystemBase {
    public Hang(String name) {
        super(name);
        //TODO Auto-generated constructor stub
    }

    public static Hang instance;

    @Override
    public void handleStateMachine() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'handleStateMachine'");
    }

    @Override
    protected void outputPeriodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'outputPeriodic'");
    }


}
