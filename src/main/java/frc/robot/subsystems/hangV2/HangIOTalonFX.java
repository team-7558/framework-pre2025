package frc.robot.subsystems.hangV2;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class HangIOTalonFX implements HangIO {
    private static final double GEAR_RATIO = 0;
    private final TalonFX leader = new TalonFX(0);;
    private final TalonFX follower = new TalonFX(1);
    private final double targetPos;

    private final StatusSignal<Double> leaderPosition = leader.getPosition();
    private final StatusSignal<Double> leaderCurrent = leader.getSupplyCurrent();
    private final StatusSignal<Double> followerCurrent = follower.getSupplyCurrent();

    public HangIOTalonFX() {
        var motorConfig = new TalonFXConfiguration();

        motorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leader.getConfigurator().apply(motorConfig);
        follower.getConfigurator().apply(motorConfig);
        follower.setControl(new Follower(leader.getDeviceID(), false));

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, leaderPosition, leaderCurrent, followerCurrent);
        leader.optimizeBusUtilization();
        follower.optimizeBusUtilization();

        targetPos = 1;
    }

    public void updateInputs(HangIOInputs inputs) {
        BaseStatusSignal.refreshAll(leaderPosition, leaderCurrent, followerCurrent);

        inputs.positionDeg = Units.rotationsToDegrees(leaderPosition.getValueAsDouble()) / GEAR_RATIO;
    }

    public void setPos(StatusSignal<Double> leaderPosition, double targetPos) {
        targetPos = this.targetPos;
        leaderPosition = this.leaderPosition;

        leader.setPosition(targetPos);
    }

    public void configurePID(double kP, double kI, double kD) {
        var config = new Slot0Configs();

        config.kP = kP;
        config.kI = kI;
        config.kD = kD;

        leader.getConfigurator().apply(config);
    }

    public void stop() {
        leader.set(0.0);
    }

    public void toggleBrake(boolean brake) {
        brake = true;
    }
}
