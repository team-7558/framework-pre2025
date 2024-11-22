package frc.robot.subsystems.pinkarm;

import frc.robot.Constants;

public class pinkarm {
  private static pinkarm instance;
  private final pinkarmIO io;
  private final pinkarm2d mech = new pinkarm2d();


  public static pinkarm getInstance() {

    if (instance == null) {
      System.out.println("pinkarm initialized");
      switch (Constants.currentMode) {
        case REAL:
          // Real robot, instantiate hardware IO implementations
          break;

        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          instance = new pinkarm(new pinkarmIOSim());
          break;

        default:
          // Replayed robot, disable IO implementations
          break;
      }
    }
    return instance;
  }

  private pinkarm(pinkarmIO io) {
    this.io = io;
  }

  public void set(double meters) {
    io.setLength(meters);
  }

  public void periodic() {
    mech.periodic();
  }
}
