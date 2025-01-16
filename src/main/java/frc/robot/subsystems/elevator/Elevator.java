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

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

public class Elevator extends StateMachineSubsystemBase<ElevatorState> {

  private boolean zeroed;

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private static Elevator instance;

  private double targetLengthMeters;

  public static final double ELEV_MIN_HEIGHT_M = 0.0;
  public static final double ELEV_MAX_HEIGHT_M = 0.0;
  public static final double ELEV_STROKE_M = ELEV_MAX_HEIGHT_M - ELEV_MIN_HEIGHT_M;

  public static final double ELEV_MIN_ANGLE_DEG = 0.0;
  public static final double ELEV_MAX_ANGLE_DEG = 0.0;

  public static final double ELEV_SCORING_TOP = 0.0;
  public static final double ELEV_SCORING_DOWN = 0.0;

  public static final double ELEV_MAX_VEL_MPS = 0.0;

  private double targetHeight_m = ELEV_MIN_HEIGHT_M;

  public Elevator(ElevatorIO io) {
    super("Elevator");
    this.io = new ElevatorIOReal();
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
      System.out.println("Elevator initialized.");
      switch (Constants.currentMode) {
        case REAL:
          instance = new Elevator(new ElevatorIOReal());
          break;

        case SIM:
          instance = new Elevator(new ElevatorIOSim());
          break;

        default:
          instance = new Elevator(new ElevatorIO() {});
          break;
      }
    }
    return instance;
  }

  @Override
  public void handleStateMachine() {
    // System.out.println(getState().name());
    switch (getState()) {
      case DISABLED:
        break;
      case HOMING:
        if (!inputs.hallEffect) {
          io.setVoltage(0.0);
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

  public double getHeight() {
    return inputs.pos_m;
  }

  public boolean atTargetHeight() {
    return atTargetHeight(0.0);
  }

  public boolean atTargetHeight(double tol) {
    return atHeight(targetHeight_m, tol);
  }

  public boolean atHeight(double height_m, double tol) {
    return Util.inRange(height_m - inputs.pos_m, tol);
  }

  public void setTargetHeight(double height_m) {
    this.targetHeight_m = MathUtil.clamp(height_m, ELEV_MIN_HEIGHT_M, ELEV_MAX_HEIGHT_M);
  }
}
