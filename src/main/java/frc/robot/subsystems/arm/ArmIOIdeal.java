package frc.robot.subsystems.arm;

public class ArmIOIdeal implements ArmIO {
  private double ElbowVel_degps = 0.0;
  private double ElbowVolts_V = 0.0;
  private double ElbowPos_deg = 0.0;

  private double ShoulderVel_degps = 0.0;
  private double ShoulderVolts_V = 0.0;
  private double ShoulderPos_deg = 0.0;

  public ArmIOIdeal() {}

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.elbow_volts_V = ElbowVolts_V;
    inputs.elbow_vel_degps = ElbowVel_degps;
    inputs.elbow_pos_deg = ElbowPos_deg;
    inputs.elbow_current_A = new double[] {40, 40};

    inputs.shoulder_volts_V = ShoulderVolts_V;
    inputs.shoulder_vel_degps = ShoulderVel_degps;
    inputs.shoulder_pos_deg = ShoulderPos_deg;
    inputs.shoulder_current_A = new double[] {40, 40};
  }

  @Override
  public void goToElbowAngle(double angle, ArmIOInputs inputs, boolean first_time, double volts) {
    this.ElbowPos_deg = angle;
  }

  @Override
  public void stopElbow() {
    this.ElbowVolts_V = 0.0;
  }

  @Override
  public void goToShoulderAngle(
      double angle, ArmIOInputs inputs, boolean first_time, double volts) {
    this.ShoulderPos_deg = angle;
  }

  public void stopShoulder() {
    this.ElbowVolts_V = 0.0;
  }
}
