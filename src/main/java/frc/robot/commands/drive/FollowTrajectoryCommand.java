// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.io.IOException;
import java.lang.reflect.Field;
import java.nio.file.Path;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class FollowTrajectoryCommand extends RamseteCommand {
  private Trajectory trajectory;
  private DriveSubsystem driveSubsystem;

  /** Creates a new PathWeaverCommand. */
  public FollowTrajectoryCommand(Trajectory trajectory, DriveSubsystem driveSubsystem) {
    super(trajectory,
        driveSubsystem::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            AutoConstants.ksVolts,
            AutoConstants.kvVoltSecondsPerMeter,
            AutoConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.driveKinematics,
        driveSubsystem::getWheelSpeeds,
        new PIDController(AutoConstants.kPDriveVel, 0, 0),
        new PIDController(AutoConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        driveSubsystem::tankDriveVolts,
        driveSubsystem);

    this.trajectory = trajectory;
    this.driveSubsystem = driveSubsystem;
  }

  public FollowTrajectoryCommand(String path, DriveSubsystem driveSubsystem) {
    this(loadTrajectory(path), driveSubsystem);
  }

  public static Trajectory loadTrajectory(String path) {
    Trajectory trajectory = new Trajectory();

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
    }

    return trajectory;
  }

  @Override
  public void initialize() {
    driveSubsystem.resetOdometry(trajectory.getInitialPose());

    super.initialize();
  }
}
