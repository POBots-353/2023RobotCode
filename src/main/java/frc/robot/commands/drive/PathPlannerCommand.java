// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.HashMap;

import javax.lang.model.element.Element;
import javax.swing.text.AbstractDocument.ElementEdit;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.manipulator.SetElevatorPositionCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElementTransitSubsystem;
import frc.robot.util.PathPlannerUtil;

public class PathPlannerCommand extends FollowPathWithEvents {
  private DriveSubsystem driveSubsystem;
  private ElementTransitSubsystem elementTransit;

  private PathPlannerTrajectory trajectory;

  public static PathPlannerTrajectory loadPathPlannerTrajectory(String path) {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(path, new PathConstraints(
        AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    return trajectory;
  }

  private static PPRamseteCommand loadPathFollowCommand(String pathName, ElementTransitSubsystem elementTransit,
      DriveSubsystem driveSubsystem) {
    return new PPRamseteCommand(loadPathPlannerTrajectory(pathName),
        driveSubsystem::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSecondsPerMeter,
            AutoConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.driveKinematics,
        driveSubsystem::getWheelSpeeds,
        new PIDController(AutoConstants.kPDriveVel, 0, 0),
        new PIDController(AutoConstants.kPDriveVel, 0, 0),
        driveSubsystem::tankDriveVolts,
        driveSubsystem, elementTransit);
  }

  private static PPRamseteCommand createPathFollowCommand(PathPlannerTrajectory trajectory,
      ElementTransitSubsystem elementTransit, DriveSubsystem driveSubsystem) {
    return new PPRamseteCommand(trajectory,
        driveSubsystem::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSecondsPerMeter,
            AutoConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.driveKinematics,
        driveSubsystem::getWheelSpeeds,
        new PIDController(AutoConstants.kPDriveVel, 0, 0),
        new PIDController(AutoConstants.kPDriveVel, 0, 0),
        driveSubsystem::tankDriveVolts,
        driveSubsystem);
  }

  /** Creates a new PathPlannerCommand. */
  public PathPlannerCommand(PathPlannerTrajectory trajectory, ElementTransitSubsystem elementTransit,
      DriveSubsystem driveSubsystem) {
    super(createPathFollowCommand(trajectory, elementTransit, driveSubsystem),
        trajectory.getMarkers(), PathPlannerUtil.eventMap);

    this.elementTransit = elementTransit;
    this.driveSubsystem = driveSubsystem;

    this.trajectory = trajectory;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elementTransit, driveSubsystem);
  }

  public PathPlannerCommand(String pathName, ElementTransitSubsystem elementTransit, DriveSubsystem driveSubsystem) {
    this(PathPlannerUtil.loadPathPlannerTrajectory(pathName), elementTransit, driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.resetOdometry(trajectory.getInitialPose());

    super.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.tankDriveVolts(0, 0);
    super.end(interrupted);
  }
}
