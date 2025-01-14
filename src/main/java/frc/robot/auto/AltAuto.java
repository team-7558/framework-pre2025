package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PathingMode;
import frc.robot.superstructure.SS;
import frc.robot.util.AltTimer;
import frc.robot.util.IStateMachine;
import frc.robot.util.Util;

public abstract class AltAuto implements IStateMachine<AutoState> {

  private String name;

  public final Trajstack trajstack;

  protected final Drive drive;
  protected final SS ss;

  private Timer swerveTimer;
  private SwerveFollower follower;

  private AutoState state;

  private boolean first;

  private AltTimer t;
  private boolean forcePoseReset;

  private boolean generated = false;

  public AltAuto(String s, boolean forcePoseReset) {
    name = s;
    drive = Drive.getInstance();
    ss = SS.getInstance();
    trajstack = new Trajstack();
    this.forcePoseReset = forcePoseReset;
    this.t = new AltTimer();
    this.state = AutoState.DO_NOTHING;
    this.swerveTimer = new Timer();
    this.follower = new SwerveFollower(trajstack);
  }

  public final void init() {
    swerveTimer.reset();
    swerveTimer.start();
    follower.start();
    System.out.println("Starting " + name);
    if (!generated) {
      generate();
    }

    if (forcePoseReset) {
      drive.setPose(trajstack.getInitState().pose);
    }

    drive.queueState(PathingMode.PATH_FOLLOWING);

    t = new AltTimer();
    t.reset();
  }

  @Override
  public String toString() {
    return name;
  }

  public void generate() {
    trajstack.generate();
    generated = true;
  }

  public final void execute() {
    drive.queueState(PathingMode.PATH_FOLLOWING);
    follower.step(swerveTimer.get());
    handleStateMachine();
    onExecute();
    // executeCallbacks(); figure this out
    double time = t.time();
    // if (time < 15.0) led.drawPreciseNumber(time, 16, 16, 16);
    // else led.drawNumber(time, 48, 0, 0);
    // TODO: ADD LED BACK
  }

  protected boolean before(double time_s) {
    return t.time() < time_s;
  }

  protected boolean after(double time_s) {
    return t.time() > time_s;
  }

  protected boolean between(double time0_s, double time1_s) {
    return after(time0_s) && before(time1_s);
  }

  protected double alpha(double time0_s, double time1_s) {
    return Util.unlerp(time0_s, time1_s, t.time());
  }

  protected boolean near(double x, double y, double tol) {
    double dx = x - drive.getPose().getX();
    double dy = y - drive.getPose().getY();
    return dx * dx + dy * dy < tol * tol;
  }

  // elipses

  protected boolean near(double x, double y, double tolX, double tolY) {
    double dx = x - drive.getPose().getX();
    double dy = y - drive.getPose().getY();
    return (dx * dx) / (tolX * tolX) + (dy * dy) / (tolY * tolY) < 1;
  }

  protected boolean near(Pose2d pose, double tolX, double tolY) {
    double dx = pose.getX() - drive.getPose().getX();
    double dy = pose.getY() - drive.getPose().getY();
    return (dx * dx) / (tolX * tolX) + (dy * dy) / (tolY * tolY) < 1;
  }

  protected boolean near(Pose2d pose, double tol) {
    double dx = pose.getX() - drive.getPose().getX();
    double dy = pose.getY() - drive.getPose().getY();
    return dx * dx + dy * dy < tol * tol;
  }

  protected boolean velUnder(double mag) {
    // return drive.velUnder(mag);
    // todo: add
    return true;
  }

  protected double segEnd(int i) {
    return trajstack.segEnd(i);
  }

  protected double angleDifference(double targetAngle) {
    double currentAngle = drive.getPose().getRotation().getDegrees();
    return targetAngle - currentAngle;
  }

  protected boolean withinRectangle(double minX, double maxX, double minY, double maxY) {
    double x = drive.getPose().getX();
    double y = drive.getPose().getY();
    return x >= minX && x <= maxX && y >= minY && y <= maxY;
  }

  protected boolean headingTowards(double x, double y, double angleTol) {
    double targetAngle =
        Math.atan2(y - drive.getPose().getY(), x - drive.getPose().getX()) * 180 / Math.PI;
    double angleDiff = angleDifference(targetAngle);
    return Math.abs(angleDiff) <= angleTol;
  }

  @Override
  public AutoState getState() {
    return state;
  }

  @Override
  public void queueState(AutoState nextState) {
    if (!state.equals(nextState)) {
      state = nextState;
      t.reset();
      first = true;
    } else {
      first = false;
    }
  }

  @Override
  public boolean stateInit() {
    if (first) {
      first = false;
      return true;
    }
    return false;
  }

  @Override
  public boolean isState(AutoState state) {
    return this.state.equals(state);
  }

  public abstract void onExecute();

  public abstract void onInit();

  @Override
  public void handleStateMachine() {
    switch (state) {
      case DO_NOTHING:
        // ss.intend(Intention.IDLE);
        break;
      case INTAKING:
        break;
      case SCORING:
        break;
      default:
        break;
    }
  }
}
