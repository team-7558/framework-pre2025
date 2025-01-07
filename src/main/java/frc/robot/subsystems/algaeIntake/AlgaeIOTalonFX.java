package frc.robot.subsystems.algaeIntake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;

public class AlgaeIOTalonFX implements AlgaeIO {
    private final TalonFX motor = new TalonFX(1);
    private final DigitalInput bb = new DigitalInput(2);
}

