package frc.robot.subsystems.elevatorWithArm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorWithArm extends SubsystemBase{
    private final ElevatorWithArmIO io;
    private final ElevatorWithArmIOInputsAutoLogged inputs = new ElevatorWithArmIOInputsAutoLogged();
    public ElevatorWithArm(ElevatorWithArmIO io) {
        this.io = io;
        switch(Constants.currentMode) {
            case REAL:
            case SIM:
            case REPLAY:
        }
    }

    public void setVoltageElev(double voltage) {
        io.setVoltageElev(voltage);
    }

    public void setVoltageArm(double voltage) {
        io.setVoltageArm(voltage);
    }

    public void setPosElev(double pos) {
        io.setPosElev(pos);
    }
    public void setPosArm(double armAngle) {
        io.setPosArm(armAngle);
    }

    public void stop(){
        io.stop();
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("ElevatorWithArm", inputs);
    }
}