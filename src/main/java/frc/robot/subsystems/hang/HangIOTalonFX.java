package frc.robot.subsystems.hang;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class HangIOTalonFX implements HangIO {
    // motor
    private final TalonFX motor;
    private final VoltageOut armVoltageControl;
    private final PositionVoltage armPosControl;

    private final StatusSignal<Double> pos_deg;
    private final StatusSignal<Double> vel_mps;
    private final StatusSignal<Double> acc_mps;
    private final StatusSignal<Double> volts_V;
    private final StatusSignal<Double> current_A;
    
    private final VoltageOut voltageControl;
    // motionMagic so that it doesn't put stress on the robot
    private final MotionMagicVelocityVoltage mmVelControl;
    private final PositionVoltage posControl;
    private final MotionMagicVoltage mmPosControl;

    private final double maxPositionDegrees = 360.0;
    private final double minPositionDegrees = 0.0;

    public HangIOTalonFX() {
        var motorConfig = new TalonFXConfiguration();

        armVoltageControl = new VoltageOut(0);
        armPosControl = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

        motor = new TalonFX(15);

        voltageControl = new VoltageOut(0);
        mmVelControl = new MotionMagicVelocityVoltage(0, 0, true, 0, 0, false, false, false);
        posControl = new PositionVoltage(minPositionDegrees, 0, true, 0, 0, false, false, false);
        mmPosControl = new MotionMagicVoltage(minPositionDegrees, true, 0, 1, false, false, false);

        pos_deg = motor.getPosition();
        vel_mps = motor.getVelocity();
        acc_mps = motor.getAcceleration();
        volts_V = motor.getMotorVoltage();
        current_A = motor.getStatorCurrent();

    }
}
