package frc.robot.auto.autos;

import frc.robot.auto.AltAuto;
import frc.robot.auto.AutoState;

public class DoNothing extends AltAuto {

    public DoNothing() {
        super("DoNothing", true);
    }

    @Override
    public void onInit() {
        queueState(AutoState.DO_NOTHING); // let's not do anything for the whole auto
    }

    @Override
    public void onExecute() {
    }


    
}
