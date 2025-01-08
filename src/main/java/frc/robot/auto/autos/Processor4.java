package frc.robot.auto.autos;

import frc.robot.auto.AltAuto;
import frc.robot.auto.AutoState;

public class Processor4 extends AltAuto {

  public Processor4() {
    super("Processor4", true);
    trajstack.appendChain().append("toscore", false);
    trajstack.setActiveIdx(0);
  }

  @Override
  public void onInit() {
    queueState(AutoState.DO_NOTHING); // let's not do anything for the whole auto
  }

  @Override
  public void onExecute() {}
}
