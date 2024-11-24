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
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import org.littletonrobotics.junction.Logger;

public class pinkarm extends StateMachineSubsystemBase<elevModes> {


  private static pinkarm instance;
  private final pinkarmIO io;
  private final pinkarm2d mech = new pinkarm2d();

  public static pinkarm getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
            break;
        case SIM:

          instance =
              new pinkarm(new pinkarmIOSim());
          break;
        default:
            break;
      }
    }
    return instance;
  }


  private final pinkarmInputsAutoLogged inputs = new pinkarmInputsAutoLogged();
  private double targetlength_m; // Added missing variable

  
  public pinkarm(pinkarmIO io) {
    super("pinkarm");
    this.io = io;
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
        setTargetLength(0.5 + 0.115);
        break;
      case HOLDING:
        break;
      default:
    }
  }

  @Override
  public void outputPeriodic() {
    mech.setLength(inputs.elev_posMeters);
    mech.setLength(targetlength_m);
    Logger.recordOutput("pinkarm/Targetlength_m", targetlength_m);
  }

  public void set(double meters) {
    if (io != null) {
        io.goToPos(meters);
    }
  }

    // Added a setter for targetHeight_m
    public void setTargetLength(double targetlength_m) {
        this.targetlength_m = targetlength_m;
    }

}