package frc.robot.subsystems.elevatorWithArm;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorWithArmIO {
    @AutoLog
    public static class ElevatorWithArmIOInputs {
        public double elevPositionM = 0.0;
        public double armPositionM = 0.0;
        public double velocityMPerSecond = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
    }

    public default void updateInputs(ElevatorWithArmIOInputs inputs) {}
    public default void setVoltageElev(double voltage_V) {}
    public default void setVoltageArm(double voltage_V) {}
    public default void setPosElev(double elevPositionM) {}
    public default void setPosArm(double armPositionM) {}
}
