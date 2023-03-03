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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.manipulator.SetElevatorPositionCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * A utility class for creating path planner commands and loading path planner
 * trajectories
 */
public class PathPlannerUtil {
  public static HashMap<String, Command> eventMap = new HashMap<String, Command>();

  public static void initializeCommands(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSystem, IntakeSubsystem intakeSystem) {
    eventMap.put("intakeCone", Commands.race(Commands.run(intakeSystem::intakeCone, intakeSystem), new WaitCommand(IntakeConstants.autoIntakeTime)));
    eventMap.put("outtakeCone", Commands.race(Commands.run(intakeSystem::outTakeCone, intakeSystem), new WaitCommand(IntakeConstants.autoIntakeTime)));

    eventMap.put("intakeCube", Commands.race(Commands.run(intakeSystem::intakeCube, intakeSystem), new WaitCommand(IntakeConstants.autoIntakeTime)));
    eventMap.put("outtakeCube", Commands.race(Commands.run(intakeSystem::outTakeCube, intakeSystem), new WaitCommand(IntakeConstants.autoIntakeTime)));

    eventMap.put("stopIntake", Commands.runOnce(intakeSystem::stopIntakeMotor, intakeSystem));

    eventMap.put("elevatorConeHigh",
        new SetElevatorPositionCommand(IntakeConstants.elevatorConeTopSetPoint, elevatorSystem));
    eventMap.put("elevatorConeMid",
        new SetElevatorPositionCommand(IntakeConstants.elevatorConeMidSetPoint, elevatorSystem));
    eventMap.put("elevatorConeLow",
        new SetElevatorPositionCommand(IntakeConstants.elevatorConeLowSetPoint, elevatorSystem));

    eventMap.put("elevatorCubeHigh", new SetElevatorPositionCommand(IntakeConstants.elevatorCubeTopSetPoint, elevatorSystem));
    eventMap.put("elevatorCubeMid", new SetElevatorPositionCommand(IntakeConstants.elevatorCubeMidSetPoint, elevatorSystem));
    eventMap.put("elevatorCubeLow", new SetElevatorPositionCommand(IntakeConstants.elevatorCubeLowSetPoint, elevatorSystem));

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
