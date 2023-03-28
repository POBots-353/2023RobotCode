// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.apriltag;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.drive.AutoTurnToAngleCommand;
import frc.robot.commands.drive.DriveToPoseCommand;
import frc.robot.subsystems.Drive;
import frc.robot.util.LimelightHelpers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignToAprilTagCommand extends SequentialCommandGroup {
  private Trajectory trajectory = new Trajectory();

  /** Creates a new AlignToAprilTagCommand. */
  public AlignToAprilTagCommand(Drive driveSubsystem) {
    addCommands(
        // Trajectory generation
        Commands.runOnce(() -> {
          Pose3d cameraPose = LimelightHelpers.getCameraPose3d_TargetSpace(DriveConstants.limelightName);

          double zTranslation = cameraPose.getZ();
          double xTranslation = cameraPose.getX();

          double zTranslationError = -zTranslation - 1.5;
          double xTranslationError = xTranslation;

          var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
              new SimpleMotorFeedforward(
                  AutoConstants.ksVolts,
                  AutoConstants.kvVoltSecondsPerMeter,
                  AutoConstants.kaVoltSecondsSquaredPerMeter),
              DriveConstants.driveKinematics,
              10);

          TrajectoryConfig config = new TrajectoryConfig(
              AutoConstants.kMaxSpeedMetersPerSecond,
              AutoConstants.kMaxAccelerationMetersPerSecondSquared)
              // Add kinematics to ensure max speed is actually obeyed
              .setKinematics(DriveConstants.driveKinematics)
              // Apply the voltage constraint
              .addConstraint(autoVoltageConstraint);

          Pose2d currentPose = driveSubsystem.getPose();
          Translation2d translation = new Translation2d(zTranslationError, xTranslationError);

          Pose2d finalPose = new Pose2d(currentPose.getTranslation().plus(translation), driveSubsystem.getRotation());

          trajectory = TrajectoryGenerator.generateTrajectory(driveSubsystem.getPose(), List.of(),
              finalPose, config);
        }, driveSubsystem),
        new DriveToPoseCommand(() -> trajectory, driveSubsystem),
        new AutoTurnToAngleCommand(0, driveSubsystem));
  }
}
