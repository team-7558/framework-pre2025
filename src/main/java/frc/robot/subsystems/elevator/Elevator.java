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

import frc.robot.subsystems.StateMachineSubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends StateMachineSubsystemBase<ElevatorState> {

  private boolean zeroed;

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private static Elevator instance;

  private double targetLengthMeters;

  public static final double ELEV_MIN_HEIGHT_M = 0;
  public static final double ELEV_MAX_HEIGHT_M = 3.55;
  public static final double ELEV_MIN_ANGLE_DEG = 0;
  public static final double ELEV_MAX_ANGLE_DEG = 180;

  public Elevator() {
    super("Elevator");
    this.io = new ElevatorIOTalonFX();
    setTargetLength(0);
    queueState(ElevatorState.IDLE);
    zeroed = false;
  }

  @Override
  public void inputPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();
    }
    return instance;
  }

  @Override
  public void handleStateMachine() {
    // System.out.println(getState().name());
    switch (getState()) {
      case DISABLED:
        break;
      case ZEROING:
        if (!inputs.hallEffectHit) {
          io.setVoltage(-0.4);
        } else {
          io.zero();
          io.setVoltage(0);
          zeroed = true;
          queueState(ElevatorState.IDLE);
        }
        break;
      case IDLE:
        break;
      case HOLDING:
        io.travelToPos(targetLengthMeters);
        break;

      case MANUAL:
        // io.setVoltage(OI.DR.getAButton() ? 2.5 : 0);
        // io.setVoltage(OI.DR.getBButton() ? 2 : 0);
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

  public boolean getZeroed() {
    return zeroed;
  }
}
