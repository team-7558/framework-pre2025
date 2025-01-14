package frc.robot.subsystems.hang;

import org.littletonrobotics.junction.AutoLog;

public interface HangIO {
    @AutoLog
    public static class HangIOInputs {
        public double[] currentAmps = new double[] {};
        public double pos_deg = 0.0;
        public double volts = 0.0;
        public double velDegPS = 0.0;
    }

    public default void updateInputs(HangIOInputs inputs) {}

    public default void setVolts(double volts) {}

    public default void setVelocity(double velocityRadPerSec, double ffVolts) {}

    public default void setAngle(double posDeg, HangIOInputs inputs) {}

    public default void stop() {}

    public default void setBrake() {}
}
