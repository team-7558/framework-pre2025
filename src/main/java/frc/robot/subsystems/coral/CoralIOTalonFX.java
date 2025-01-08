package frc.robot.subsystems.coral;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;

public class CoralIOTalonFX implements CoralIO {

  private final TalonFX motor;
  private final VoltageOut voltage_out;
  private final DigitalInput beambreak;

  public CoralIOTalonFX() {
    motor = new TalonFX(13);
    var motorConfig = new TalonFXConfiguration();
    beambreak = new DigitalInput(0);

    motorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
    motorConfig.Feedback.SensorToMechanismRatio = 5;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor.getConfigurator().apply(motorConfig);

    // Initialize voltage control
    voltage_out = new VoltageOut(0.0);
  }

  public void updateInputs(CoralIOInputs inputs) {
    // Refresh all signals to get updated data
    BaseStatusSignal.refreshAll();

    // Read motor data and update inputs
    inputs.current_Amps = new double[] {motor.getStatorCurrent().getValueAsDouble()};
    inputs.AppliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.VelocityDegPS =
        motor.getVelocity().getValueAsDouble(); // Assuming getVelocity() is properly configured
  }

  public void setVoltage(double volts) {
    // Clamp voltage to valid range (-12V to 12V)
    volts = MathUtil.clamp(volts, -12.0, 12.0);

    // Set motor voltage
    motor.setControl(voltage_out.withOutput(volts));
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
