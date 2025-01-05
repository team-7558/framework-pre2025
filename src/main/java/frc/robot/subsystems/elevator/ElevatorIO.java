package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;


public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double position_r = 0;
        public double velocity_rps = 0;

        public boolean hallEffectHit = false;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    public default void setPosition() {

    }

    public default void setVelocity() {

    }

    public default void zero() {

    }
}
