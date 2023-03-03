// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.manipulator.SetElevatorPositionCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElementTransitSubsystem;

/**
 * A utility class for creating path planner commands and loading path planner
 * trajectories
 */
public class PathPlannerUtil {
  public static HashMap<String, Command> eventMap = new HashMap<String, Command>();

  public static void initializeCommands(DriveSubsystem driveSubsystem, ElementTransitSubsystem elementTransit) {
    eventMap.put("intakeCone", Commands.run(elementTransit::intakeCone, elementTransit));
    eventMap.put("outtakeCone", Commands.run(elementTransit::outTakeCone, elementTransit));
    eventMap.put("intakeCube", Commands.run(elementTransit::intakeCube, elementTransit));
    eventMap.put("outtakeCube", Commands.run(elementTransit::outTakeCube, elementTransit));
    eventMap.put("stopIntake", Commands.runOnce(elementTransit::stopIntakeMotor, elementTransit));

    eventMap.put("elevatorConeHigh",
        new SetElevatorPositionCommand(IntakeConstants.elevatorConeTopSetPoint, elementTransit));
    eventMap.put("elevatorConeMid",
        new SetElevatorPositionCommand(IntakeConstants.elevatorConeMidSetPoint, elementTransit));
    eventMap.put("elevatorConeLow",
        new SetElevatorPositionCommand(IntakeConstants.elevatorConeLowSetPoint, elementTransit));

    // eventMap.put("marker1", new PrintCommand("Passed Marker 1"));
    // eventMap.put("marker2", new PrintCommand("Passed Marker 2"));
    // eventMap.put("marker3", new PrintCommand("Passed Marker 3"));
  }

  public static PathPlannerTrajectory loadPathPlannerTrajectory(String path) {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(path, new PathConstraints(
        AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    return trajectory;
  }

  public static PPRamseteCommand createPathFollowCommand(PathPlannerTrajectory trajectory,
      DriveSubsystem driveSubsystem) {
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
}
