package frc.robot.subsystems.pinkarm;

import edu.wpi.first.wpilibj.DigitalInput;

public class ClawIOSim implements ClawIO {
    DigitalInput beambreak = new DigitalInput(1);

    public void updateInputs(ClawInputs inputs) {
        inputs.gamepiece = beambreak.get();
    }

    public void open(boolean open_or_not) {
        
    }

    public void stop() {}
}
