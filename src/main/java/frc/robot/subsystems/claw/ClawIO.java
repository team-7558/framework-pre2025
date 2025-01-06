package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public interface ClawIO {
    public static class ClawIOInputs {

    public double elev_posMeters = 0.5;
    public double elev_velMPS = 0.0;
    public double elev_volts = 0.0;
    public double[] elev_currents = new double[] {};

    public double arm_posDegrees = 0.0;
    public double arm_velDegPS = 0.0;
    public double arm_volts = 0.0;
    public double[] arm_currents = new double[] {};
  }
}
