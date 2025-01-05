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
import frc.robot.OI;
import frc.robot.subsystems.StateMachineSubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends StateMachineSubsystemBase<ElevatorState> {

    private static Elevator instance;

    private boolean zeroed = false;

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private double targetLengthMeters;

    public static final double ELEV_MIN_HEIGHT_M = 0;
    public static final double ELEV_MAX_HEIGHT_M = 10000;
    public static final double ELEV_MIN_ANGLE_DEG = 0;
    public static final double ELEV_MAX_ANGLE_DEG = 180;

    public Elevator() {
        super("Elevator");
        this.io = new ElevatorIOTalonFX();
        setTargetLength(0.7);
        queueState(ElevatorState.IDLE);
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
            case ZEROING:
                if(!inputs.hallEffectHit) {
                    io.setVoltage(-0.6);
                } else {
                    io.zero();
                    io.setVoltage(0);
                    zeroed = true;
                }
                break;
            case IDLE:
                break;
            case TRAVELLING:
                if (Math.abs(targetLengthMeters - inputs.position_m) < 0.2) {
                    queueState(ElevatorState.HOLDING);
                }
                break;
            case HOLDING:
                io.travelToPos(targetLengthMeters);
                break;

            case MANUAL:
                io.setVoltage(OI.DR.getAButton() ? 1.5 : 0);
                io.setVoltage(OI.DR.getBButton() ? -1.5 : 0);
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

    public boolean getZeroed() {
        return zeroed;
    }
}
