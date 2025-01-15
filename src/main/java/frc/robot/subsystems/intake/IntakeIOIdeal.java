package frc.robot.subsystems.intake;

public class IntakeIOIdeal implements IntakeIO {
  public double slap_pos_deg = 0.0;
  public double slap_velDegPS = 0.0;
  public double slap_volts = 0.0;
  public double[] slap_currents = new double[] {};

  public double VelocityDegPS = 0.0;
  public double AppliedVolts = 0.0;
  public double[] current_Amps = new double[] {};
  public boolean beamBreakActivated = false;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.AppliedVolts = AppliedVolts;
    inputs.VelocityDegPS = VelocityDegPS;
    inputs.beamBreakActivated = beamBreakActivated;
    inputs.current_Amps = current_Amps;

    inputs.slap_currents = slap_currents;
    inputs.slap_pos_deg = slap_pos_deg;
    inputs.slap_velDegPS = slap_velDegPS;
    inputs.slap_volts = slap_volts;
  }

  public void setSlap_pos_deg(double slap_pos_deg) {
    this.slap_pos_deg = slap_pos_deg;
  }

  public void setSlap_velDegPS(double slap_velDegPS) {
    this.slap_velDegPS = slap_velDegPS;
  }

  public void setSlap_volts(double slap_volts) {
    this.slap_volts = slap_volts;
  }

  public void setSlap_currents(double[] slap_currents) {
    this.slap_currents = slap_currents;
  }

  public void setVelocityDegPS(double velocityDegPS) {
    VelocityDegPS = velocityDegPS;
  }

  public void setAppliedVolts(double appliedVolts) {
    AppliedVolts = appliedVolts;
  }

  public void setCurrent_Amps(double[] current_Amps) {
    this.current_Amps = current_Amps;
  }

  public void setBeamBreakActivated(boolean beamBreakActivated) {
    this.beamBreakActivated = beamBreakActivated;
  }
}
