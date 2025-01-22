package frc.robot.subsystems.hand;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;

public class HandIOTalonFX implements HandIO {

  private final TalonFX intakeMotor;
  private final TalonFX scoringMotor;
  private final VoltageOut voltage_out;
  private final DigitalInput beambreak;

  public HandIOTalonFX() {
    intakeMotor = new TalonFX(13);
    scoringMotor = new TalonFX(14);
    var motorConfig = new TalonFXConfiguration();
    beambreak = new DigitalInput(0);

    motorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
    motorConfig.Feedback.SensorToMechanismRatio = 5;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeMotor.getConfigurator().apply(motorConfig);
    scoringMotor.getConfigurator().apply(motorConfig);

    // Initialize voltage control
    voltage_out = new VoltageOut(0.0);
  }

  public void updateInputs(HandIOInputs inputs) {
    // Refresh all signals to get updated data
    BaseStatusSignal.refreshAll();

    // Read motor data and update inputs
    inputs.intake_current_Amps = new double[] {intakeMotor.getStatorCurrent().getValueAsDouble()};
    inputs.intakeAppliedVolts = intakeMotor.getMotorVoltage().getValueAsDouble();
    inputs.intakeVelocityDegPS =
        intakeMotor
            .getVelocity()
            .getValueAsDouble(); // Assuming getVelocity() is properly configured
    inputs.scoring_current_Amps = new double[] {intakeMotor.getStatorCurrent().getValueAsDouble()};
    inputs.scoringAppliedVolts = intakeMotor.getMotorVoltage().getValueAsDouble();
    inputs.scoringVelocityDegPS =
        scoringMotor
            .getVelocity()
            .getValueAsDouble(); // Assuming getVelocity() is properly configured
  }

  @Override
  public void intakeSetVoltage(double volts) {
    // Clamp voltage to valid range (-12V to 12V)
    volts = MathUtil.clamp(volts, -12.0, 12.0);

    // Set motor voltage
    intakeMotor.setControl(voltage_out.withOutput(volts));
  }

  @Override
  public void intakeStop() {
    intakeSetVoltage(0.0);
  }

  @Override
  public void scoringSetVoltage(double volts) {
    // Clamp voltage to valid range (-12V to 12V)
    volts = MathUtil.clamp(volts, -12.0, 12.0);

    // Set motor voltage
    scoringMotor.setControl(voltage_out.withOutput(volts));
  }

  @Override
  public void scoringStop() {
    scoringSetVoltage(0.0);
  }
}
