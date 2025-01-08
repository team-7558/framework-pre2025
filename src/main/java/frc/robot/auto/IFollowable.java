package frc.robot.auto;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

public interface IFollowable {

  public void generate();

  public boolean isGenerated();

  public PathPlannerTrajectoryState sample(double time_s);

  public double endTime();

  public PathPlannerTrajectoryState getInitState();

  public PathPlannerTrajectoryState getEndState();

  public double segEnd(int i);
}
