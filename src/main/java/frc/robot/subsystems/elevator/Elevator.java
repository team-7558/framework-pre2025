// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.elevator;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends StateMachineSubsystemBase<ElevatorState> {

    private static Elevator instance;

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private double targetLengthMeters;

    public static final double ELEV_MIN_HEIGHT_M = 0.5;
    public static final double ELEV_MAX_HEIGHT_M = 1.5;
    public static final double ELEV_MIN_ANGLE_DEG = 0;
    public static final double ELEV_MAX_ANGLE_DEG = 180;

    private Elevator(ElevatorIO io) {
        super("Elevator");
        this.io = io;
        setTargetLength(0.7);
        queueState(ElevatorState.IDLE);
    }

    public static Elevator getInstance() {
        if (instance == null) {
            switch (Constants.currentMode) {
                case SIM:
                    break;
                case REAL:
                    instance = new Elevator(new ElevatorIOTalonFX());
                    break;
                default:
                    break;
            }
        }
        return instance;
    }

    @Override
    public void inputPeriodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    @Override
    public void handleStateMachine() {
        switch (getState()) {
            case DISABLED:
                break;
            case IDLE:
                break;
            case TRAVELLING:
                if (Math.abs(targetLengthMeters - inputs.position_m) < 0.5) {
                    queueState(ElevatorState.HOLDING);
                }
                break;
            case HOLDING:
                io.setVoltage(0);
                break;
            default:
                break;
        }
    }

    @Override
    public void outputPeriodic() {
        io.travelToPos(targetLengthMeters);
        Logger.recordOutput("Elevator/TargetLengthMeters", targetLengthMeters);
    }

    public void set(double meters) {
        meters = Math.max(ELEV_MIN_HEIGHT_M, Math.min(ELEV_MAX_HEIGHT_M, meters));
        setTargetLength(meters);
    }

    public void setTargetLength(double targetLengthMeters) {
        this.targetLengthMeters = targetLengthMeters;
    }
}
