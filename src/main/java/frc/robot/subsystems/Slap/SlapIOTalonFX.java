package frc.robot.subsystems.Slap;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

public class SlapIOTalonFX implements SlapIO {

    // Arm motor
    private final TalonFX  motor;
    private final VoltageOut armVoltageControl;
    private final PositionVoltage armPosControl;

    private final StatusSignal<Double> pos_m;
    private final StatusSignal<Double> vel_mps;
    private final StatusSignal<Double> acc_mps2;
    private final StatusSignal<Double> volts_V;
    private final StatusSignal<Double> leftCurrent_A, rightCurrent_A;

    private final boolean isLeftMotorInverted = true;
    private final VoltageOut voltageControl;
    private final MotionMagicVelocityVoltage mmVelControl;
    private final PositionVoltage posControl;
    private final MotionMagicVoltage mmPosControl;

    private final double maxPositionDegrees = 90;
    private final double minPositionDegrees = 0; // Set your desired minimum position here


    public SlapIOTalonFX() {
        var motorConfig = new TalonFXConfiguration();

        armVoltageControl = new VoltageOut(0);
        armPosControl = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

        motor = new TalonFX(11);

        // Configure arm motor

        voltageControl = new VoltageOut(0);
        mmVelControl = new MotionMagicVelocityVoltage(0, 0, true, 0, 2, false, false, false);
        posControl = new PositionVoltage(minPositionDegrees, 0, true, 0, 0, false, false, false);
        mmPosControl = new MotionMagicVoltage(minPositionDegrees, true, 0, 1, false, false, false);

        pos_m = motor.getPosition();
        vel_mps = motor.getVelocity();
        acc_mps2 = motor.getAcceleration();
        leftCurrent_A = motor.getStatorCurrent();
        rightCurrent_A = motor.getStatorCurrent();
        volts_V = motor.getMotorVoltage();

        motorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
        motorConfig.Feedback.SensorToMechanismRatio = 5;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0;
        motorConfig.MotionMagic.MotionMagicAcceleration = 1.0;
        motorConfig.MotionMagic.MotionMagicJerk = 0;
        motorConfig.MotionMagic.MotionMagicExpo_kV = 0.5;
        motorConfig.MotionMagic.MotionMagicExpo_kA = 0.5;

        // Position control gains
        motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        motorConfig.Slot0.kG = 0.07;
        motorConfig.Slot0.kP = 176;
        motorConfig.Slot0.kI = 0;
        motorConfig.Slot0.kD = 4; // 16;

        // MotionMagic Position gains
        motorConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;
        motorConfig.Slot1.kG = 0.07;
        motorConfig.Slot1.kV = 38;
        motorConfig.Slot1.kS = 0.0;
        motorConfig.Slot1.kA = 0; // 1;
        motorConfig.Slot1.kP = 23;
        motorConfig.Slot1.kI = 0;
        motorConfig.Slot1.kD = 0.0;

        // MotionMagic Velocity control
        motorConfig.Slot2.GravityType = GravityTypeValue.Elevator_Static;
        motorConfig.Slot2.kG = 0;
        motorConfig.Slot2.kV = 0.0;
        motorConfig.Slot2.kS = 0.0;
        motorConfig.Slot2.kA = 0.0;
        motorConfig.Slot2.kP = 0.0;
        motorConfig.Slot2.kI = 0.0;
        motorConfig.Slot2.kD = 0.0;

        motor.getConfigurator().apply(motorConfig);
        motor.getConfigurator().apply(motorConfig);
    }

    public void updateInputs(SlapIOInputs inputs) {
        BaseStatusSignal.refreshAll();
        inputs.left_currents = new double[] { motor.getStatorCurrent().getValueAsDouble() };
        inputs.pos_deg  = Units.radiansToDegrees(motor.getPosition().getValueAsDouble());
        inputs.left_volts = motor.getMotorVoltage().getValueAsDouble();
        inputs.left_velDegPS = motor.getVelocity().getValueAsDouble();
    }

    public void goToAngle(double position_deg) {
        double rotations = position_deg / 360;
        motor.setControl(armPosControl.withPosition(MathUtil.clamp(rotations, 35, maxPositionDegrees)));
    }

    public void setVoltage(double volts) {
        motor.setControl(armVoltageControl.withOutput(volts));
    }

    @Override
    public void stop() {
      setVoltage(0.0);
    }

}
