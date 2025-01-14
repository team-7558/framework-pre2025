package frc.robot.auto.autos;

import frc.robot.auto.AltAuto;
import frc.robot.auto.AutoState;

public class Processor4 extends AltAuto {

  public Processor4() {
    super("Processor4", true);
    trajstack.appendChain().append("straightline", false);
    trajstack
        .appendChain()
        .append("toscore", false)
        .append("scoretopickup1", false)
        .append("pickuptoscore2", false)
        .append("score2topickup3", false)
        .append("pickup3toscore3", false)
        .append("score3topickup4", false)
        .append("pickup4toscore4", false);
    trajstack.setActiveIdx(0);
  }

  @Override
  public void onInit() {
    queueState(AutoState.DO_NOTHING); // let's not do anything for the whole auto
  }

  @Override
  public void onExecute() {}
}
