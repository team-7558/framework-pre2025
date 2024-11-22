package frc.robot.subsystems.pinkarm;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;

public class pinkarm {
    private static pinkarm instance;

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

    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
    }
}
