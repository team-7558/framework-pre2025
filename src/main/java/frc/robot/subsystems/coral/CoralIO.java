package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIO {
    @AutoLog
    public static class CoralIOInputs {
        public double intakeVelocityMPS = 0.0;
        public double intakeAppliedVolts = 0.0;
        public double[] intakecurrentAmps = new double[] {};
        public boolean beamBreakActivated = false;
    }

    public default void updateInputs(CoralIOInputs inputs) {}

    public default void setVelocity(double velocity) {}

    public default void setVolts(double volts) {}

    public default void stop() {}

    public default void toggleBrake() {}
}
