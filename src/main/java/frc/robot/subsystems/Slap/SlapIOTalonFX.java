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
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
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
        var leaderConfig = new TalonFXConfiguration();
        var followerConfig = new TalonFXConfiguration();

        armVoltageControl = new VoltageOut(0);
        armPosControl = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

        leftMotor = new TalonFX(11);
        rightMotor = new TalonFX(12);

        // Configure arm motor

        voltageControl = new VoltageOut(0);
        mmVelControl = new MotionMagicVelocityVoltage(0, 0, true, 0, 2, false, false, false);
        posControl = new PositionVoltage(minPositionDegrees, 0, true, 0, 0, false, false, false);
        mmPosControl = new MotionMagicVoltage(minPositionDegrees, true, 0, 1, false, false, false);

        pos_m = rightMotor.getPosition();
        vel_mps = rightMotor.getVelocity();
        acc_mps2 = rightMotor.getAcceleration();
        leftCurrent_A = rightMotor.getStatorCurrent();
        rightCurrent_A = rightMotor.getStatorCurrent();
        volts_V = leftMotor.getMotorVoltage();

        leaderConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leaderConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
        leaderConfig.Feedback.SensorToMechanismRatio = 5;
        leaderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        leaderConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0;
        leaderConfig.MotionMagic.MotionMagicAcceleration = 1.0;
        leaderConfig.MotionMagic.MotionMagicJerk = 0;
        leaderConfig.MotionMagic.MotionMagicExpo_kV = 0.5;
        leaderConfig.MotionMagic.MotionMagicExpo_kA = 0.5;

        // Position control gains
        leaderConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        leaderConfig.Slot0.kG = 0.07;
        leaderConfig.Slot0.kP = 176;
        leaderConfig.Slot0.kI = 0;
        leaderConfig.Slot0.kD = 4; // 16;

        // MotionMagic Position gains
        leaderConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;
        leaderConfig.Slot1.kG = 0.07;
        leaderConfig.Slot1.kV = 38;
        leaderConfig.Slot1.kS = 0.0;
        leaderConfig.Slot1.kA = 0; // 1;
        leaderConfig.Slot1.kP = 23;
        leaderConfig.Slot1.kI = 0;
        leaderConfig.Slot1.kD = 0.0;

        // MotionMagic Velocity control
        leaderConfig.Slot2.GravityType = GravityTypeValue.Elevator_Static;
        leaderConfig.Slot2.kG = 0;
        leaderConfig.Slot2.kV = 0.0;
        leaderConfig.Slot2.kS = 0.0;
        leaderConfig.Slot2.kA = 0.0;
        leaderConfig.Slot2.kP = 0.0;
        leaderConfig.Slot2.kI = 0.0;
        leaderConfig.Slot2.kD = 0.0;

        leftMotor.getConfigurator().apply(leaderConfig);
        rightMotor.getConfigurator().apply(leaderConfig);
    }

    public void updateInputs(SlapIOInputs inputs) {
        BaseStatusSignal.refreshAll();
        inputs.left_currents = new double[] { leftMotor.getStatorCurrent().getValueAsDouble() };
        inputs.pos_deg  = Units.radiansToDegrees(leftMotor.getPosition().getValueAsDouble());
        inputs.left_volts = leftMotor.getMotorVoltage().getValueAsDouble();
        inputs.left_velDegPS = leftMotor.getVelocity().getValueAsDouble();

        inputs.right_currents = new double[] { rightMotor.getStatorCurrent().getValueAsDouble() };
        inputs.right_velDegPS = rightMotor.getVelocity().getValueAsDouble();
        inputs.right_volts = rightMotor.getMotorVoltage().getValueAsDouble();
    }

    public void goToAngle(double position_deg) {
        double rotations = position_deg / 360;
        leftMotor.setControl(armPosControl.withPosition(MathUtil.clamp(rotations, 35, maxPositionDegrees)));
    }

    public void setVoltage(double volts) {
        leftMotor.setControl(armVoltageControl.withOutput(volts));
    }

    @Override
    public void stop() {
      setVoltage(0.0);
    }

}
