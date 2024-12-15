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

package frc.robot.subsystems.pinkarm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.util.AltTimer;
import org.littletonrobotics.junction.Logger;

public class Pinkarm extends StateMachineSubsystemBase<elevModes> {

    private static Pinkarm instance;

    private final PinkarmIO io;
    private final Pinkarm2d mech = new Pinkarm2d("ArmActual", new Color8Bit(100, 0, 0));
    private final Pinkarm2d targetMech = new Pinkarm2d("ArmTarget", new Color8Bit(0, 0, 100));

    private final PinkarmInputsAutoLogged inputs = new PinkarmInputsAutoLogged();
    private final AltTimer timer = new AltTimer();

    private double targetLengthMeters;
    private double targetAngleDegrees;

    public static final double ELEV_MIN_HEIGHT_M = 0.5;
    public static final double ELEV_MAX_HEIGHT_M = 1.5;
    public static final double ELEV_MIN_ANGLE_DEG = 0;
    public static final double ELEV_MAX_ANGLE_DEG = 180;

    private TrapezoidProfile.State elevSetpoint;
    private TrapezoidProfile.State elevStartPoint;
    private TrapezoidProfile.State elevGoal;

    private TrapezoidProfile.State armSetpoint;
    private TrapezoidProfile.State armStartPoint;
    private TrapezoidProfile.State armGoal;

    private final TrapezoidProfile.Constraints elevConstraints = new TrapezoidProfile.Constraints(1, 0.5);
    private final TrapezoidProfile.Constraints armConstraints = new TrapezoidProfile.Constraints(90, 45);

    private final TrapezoidProfile elevProfile = new TrapezoidProfile(elevConstraints);
    private final TrapezoidProfile armProfile = new TrapezoidProfile(armConstraints);

    private Pinkarm(PinkarmIO io) {
        super("pinkarm");
        this.io = io;
        setTargetLength(0.7);
        queueState(elevModes.IDLE);
    }

    public static Pinkarm getInstance() {
        if (instance == null) {
            switch (Constants.currentMode) {
                case SIM:
                    instance = new Pinkarm(new PinkarmIOSim());
                    break;
                case REAL:
                default:
                    break;
            }
        }
        return instance;
    }

    @Override
    public void inputPeriodic() {
        io.updateInputs(inputs);
        Logger.processInputs("pinkarm", inputs);
    }

    @Override
    public void handleStateMachine() {
        switch (getState()) {
            case DISABLED:
                break;
            case IDLE:
                break;
            case TRAVELLING:
                if (Math.abs(targetLengthMeters - inputs.elev_posMeters) < 0.5 &&
                    Math.abs(inputs.arm_posDegrees - targetAngleDegrees) < 5) {
                    queueState(elevModes.HOLDING);
                } else {
                    if (stateInit()) {
                        timer.reset();

                        armGoal = new TrapezoidProfile.State(targetAngleDegrees, 0);
                        elevGoal = new TrapezoidProfile.State(targetLengthMeters, 0);

                        elevStartPoint = new TrapezoidProfile.State(inputs.elev_posMeters, inputs.elev_velMPS);
                        armStartPoint = new TrapezoidProfile.State(inputs.arm_posDegrees, inputs.arm_velDegPS);
                    }

                    elevSetpoint = elevProfile.calculate(timer.time(), elevStartPoint, elevGoal);
                    armSetpoint = armProfile.calculate(timer.time(), armStartPoint, armGoal);
                }
                break;
            case HOLDING:
                io.setArmVoltage(0);
                io.setelevVoltage(0);
                break;
            default:
                break;
        }
    }

    @Override
    public void outputPeriodic() {
        io.goToPos(elevSetpoint.position);
        mech.setLength(inputs.elev_posMeters);
        targetMech.setLength(elevSetpoint.position);

        io.goToAngle(armSetpoint.position);
        mech.setAngle(inputs.arm_posDegrees);
        targetMech.setAngle(armSetpoint.position);

        mech.periodic();
        targetMech.periodic();

        Logger.recordOutput("pinkarm/TargetLengthMeters", targetLengthMeters);
        Logger.recordOutput("pinkarm/TargetAngleDegrees", targetAngleDegrees);
        Logger.recordOutput("pinkarm/ElevSetpoint", elevSetpoint.position);
        Logger.recordOutput("pinkarm/ArmSetpoint", armSetpoint.position);
    }

    public void placeEndEffector(double x, double y) {
        double magnitude = Math.sqrt(0.1 * x * 0.1 * x + 0.1 * y * 0.1 * y);
        double angle = Units.radiansToDegrees(Math.atan2(0.1 * y, 0.1 * x));

        set(magnitude, angle);
    }

    public void set(double meters, double angle) {
        meters = Math.max(ELEV_MIN_HEIGHT_M, Math.min(ELEV_MAX_HEIGHT_M, meters));
        angle = Math.max(ELEV_MIN_ANGLE_DEG, Math.min(ELEV_MAX_ANGLE_DEG, angle));

        setTargetLength(meters);
        setTargetAngle(angle);
    }

    public void setTargetLength(double targetLengthMeters) {
        this.targetLengthMeters = targetLengthMeters;
    }

    public void setTargetAngle(double targetAngleDegrees) {
        this.targetAngleDegrees = targetAngleDegrees;
    }
}
