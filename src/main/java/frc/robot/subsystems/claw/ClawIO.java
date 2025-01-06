package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.AutoLog;


public interface ClawIO {
  @AutoLog
  public static class ClawIOInputs {
    public boolean open_or_not = true;
    public double velocityMPS = 0.0;
    public double volts_V = 0.0;
    public double[] currents_A = new double[] {};
  }

  public default updateInputs(ClawIOInputs inputs) {}

  public default setVelocity(double velocity) {} 

  public default setVoltage(double volts) {}

  public default open(boolean open_or_not) {}
}
