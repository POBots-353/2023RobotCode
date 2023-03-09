// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.PathPlannerUtil;

public class PathPlannerCommand extends FollowPathWithEvents {
  private DriveSubsystem driveSubsystem;

  private PathPlannerTrajectory trajectory;

  public static PathPlannerTrajectory loadPathPlannerTrajectory(String path) {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(path, new PathConstraints(
        AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    return trajectory;
  }

  /** Creates a new PathPlannerCommand. */
  public PathPlannerCommand(PathPlannerTrajectory trajectory,
      DriveSubsystem driveSubsystem) {
    super(PathPlannerUtil.createPathFollowCommand(trajectory, driveSubsystem),
        trajectory.getMarkers(), PathPlannerUtil.eventMap);

    this.driveSubsystem = driveSubsystem;

    this.trajectory = trajectory;

    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(driveSubsystem);
  }

  public PathPlannerCommand(String pathName, DriveSubsystem driveSubsystem) {
    this(PathPlannerUtil.loadPathPlannerTrajectory(pathName), driveSubsystem);
  }

  @Override
  public void initialize() {
    // driveSubsystem.resetOdometry(trajectory.getInitialPose());
    driveSubsystem.resetOdometry(trajectory.getInitialPose(), trajectory);

    super.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.tankDrive(0, 0);
    super.end(interrupted);
  }
}
