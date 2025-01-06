package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIO {
    @AutoLog
    public static class CoralIOInputs {
        public double intakeVelocityMPS = 0.0;
        public double intakeAppliedVolts = 0.0;
        public double[] intakecurrentAmps = new double[] {};


        public double slapDownVelocityMPS = 0.0;
        public double slapDownAppliedVolts = 0.0;
        public double[] slapDowncurrentAmps = new double[] {}; 


        public boolean beamBreakActivated = false;
    }

    public default void updateInputs(CoralIOInputs inputs) {}

    public default void setWheelVelocity(double velocity) {}

    public default void setWheelVolts(double volts) {}

    public default void stopWheel() {}

    public default void setSlapVelocity(double velocity) {}

    public default void setSlapVolts(double volts) {}

    public default void stopSlap() {}

    public default void toggleBrake() {}
}
