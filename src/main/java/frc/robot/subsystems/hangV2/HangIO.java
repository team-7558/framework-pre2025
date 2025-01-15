package frc.robot.subsystems.hangV2;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.PIDController;

public interface HangIO {
    @AutoLog
    public static class HangIOInputs {
        double positionDeg = 0.0;
        double targetPos = 0.0;
        public double[] currentAmps = new double[] {};      // why do we make this an array? There is only one value??
        PIDController pidController = new PIDController(positionDeg, targetPos, positionDeg);
    }

    // update the inputs
    public default void updateInputs(HangIOInputs inputs) {};

    // set the mech at a specific height
    public default void setPos(double positionDeg, double targetPos) {}

    public default void configurePID(double kP, double kI, double kD) {}

    public default void stop() {}

    public default void toggleBrake(boolean brake) {}
}
