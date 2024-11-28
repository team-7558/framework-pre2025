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

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Pinkarm extends StateMachineSubsystemBase<elevModes> {
  private final Pinkarm2d mech = new Pinkarm2d();
  private static Pinkarm instance;
  private final PinkarmIO io;

  public static Pinkarm getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
          break;
        case SIM:
          instance = new Pinkarm(new PinkarmIOSim());
          break;
        default:
          break;
      }
    }
    return instance;
  }

  private final PinkarmInputsAutoLogged inputs = new PinkarmInputsAutoLogged();
  private double targetlength_m; // Added missing variable
  private double targetangle_deg;
  private double magnitude;
  private double angle;

  public static final double ELEV_MIN_HEIGHT_M = 0.5;
  public static final double ELEV_MAX_HEIGHT_M = 1.5;
  public static final double ELEV_MIN_ANGLE_DEG = 0;
  public static final double ELEV_MAX_ANGLE_DEG = 180;

  private Pinkarm(PinkarmIO io) {
    super("pinkarm");
    this.io = io;
    setTargetLength(0.7);
    queueState(elevModes.IDLE);
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
        setTargetLength(ELEV_MIN_HEIGHT_M + 0.115);
        break;
      case HOLDING:
        break;
      default:
        break;
    }
  }

  @Override
  public void outputPeriodic() {
    io.goToPos(targetlength_m);
    mech.setLength(inputs.elev_posMeters);

    io.goToAngle(targetangle_deg);
    mech.setAngle(inputs.arm_posDegrees);

    mech.periodic();
    Logger.recordOutput("pinkarm/Targetlength_m", targetlength_m);
    Logger.recordOutput("pinkarm/Targetangle_deg", targetangle_deg);
  }

  public void set(double meters, double angle) {

    if (meters > ELEV_MAX_HEIGHT_M) {
      meters = ELEV_MAX_HEIGHT_M;
    } else if (meters < ELEV_MIN_HEIGHT_M) {
      meters = ELEV_MIN_HEIGHT_M;
    }

    if (angle > ELEV_MAX_ANGLE_DEG) {
      angle = ELEV_MAX_ANGLE_DEG;
    } else if (angle < ELEV_MIN_ANGLE_DEG) {
      angle = ELEV_MIN_HEIGHT_M;
    }

    setTargetLength(meters);
    setTargetAngle(angle);
  }

  public void PlaceEndEffector(double x, double y) {
    magnitude = Math.sqrt(0.1 * x * 0.1 * x + 0.1 * y * 0.1 * y);
    angle = Units.radiansToDegrees(Math.atan2(0.1 * y, 0.1 * x));

    set(magnitude, angle);
  }

  // Added a setter for targetHeight_m
  public void setTargetLength(double targetlength_m) {
    this.targetlength_m = targetlength_m;
  }

  public void setTargetAngle(double targetangle_deg) {
    this.targetangle_deg = targetangle_deg;
  }
}
