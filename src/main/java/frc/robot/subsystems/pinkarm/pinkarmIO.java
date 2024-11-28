package frc.robot.subsystems.pinkarm;

import org.littletonrobotics.junction.AutoLog;

public interface PinkarmIO {
  @AutoLog
  public static class PinkarmInputs {

    public double elev_posMeters = 0.5;
    public double elev_velMPS = 0.0;
    public double elev_volts = 0.0;
    public double[] elev_currents = new double[] {};

    public double arm_posDegrees = 0.0;
    public double arm_velDegPS = 0.0;
    public double arm_volts = 0.0;
    public double[] arm_currents = new double[] {};
  }

  public default void updateInputs(PinkarmInputs inputs) {}

  public default void setelevVoltage(double volts) {}

  public default void setArmVoltage(double volts) {}

  public default void goToAngle(double degrees) {}

  public default void setAngle(double degrees) {}

  public default void goToPos(double meters) {}

  public default void stop() {}
}
