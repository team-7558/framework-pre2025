package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double arm_position_r = 0;
    public double arm_velocity_rps = 0;
    public double arm_volts_V = 0;
    public double[] arm_current_A = new double[] {};
    public boolean arm_halleffect = false;

    public double wrist_position_r = 0;
    public double wrist_velocity_rps = 0;
    public double wrist_volts_V = 0;
    public double[] wrist_current_A = new double[] {};

    public double wheels_velocity_rps = 0;
    public double wheels_volts_V = 0;
    public double[] wheels_current_A = new double[] {};
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void setWristAngle(double a) {}

  public default void setWheels(double velocity) {}

  public default void zero() {}

  public default void setSolenoid(boolean solenoid) {}
}
