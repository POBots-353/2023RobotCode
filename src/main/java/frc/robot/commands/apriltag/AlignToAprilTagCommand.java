// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.apriltag;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Limelight;
import frc.robot.commands.AlignToTapeCommand;
import frc.robot.commands.autonomous.AutoDriveCommand;
import frc.robot.commands.autonomous.AutoTurnToAngleCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignToAprilTagCommand extends SequentialCommandGroup {
  private double neededDistance;
  private double neededAngle;

  private double relativeAngle;

  private Limelight camera;

  /** Creates a new AlignToAprilTagCommand. */
  public AlignToAprilTagCommand(DriveSubsystem driveSubsystem) {
    camera = driveSubsystem.getCamera();

    addCommands(
        Commands.runOnce(() -> {
          Double[] cameraPose = camera.getTable().getEntry("campose").getDoubleArray(new Double[0]);

          if (cameraPose.length == 0) {
            return;
          }

          double zTranslation = -cameraPose[2];
          double xTranslation = cameraPose[0];

          double zTranslationError = zTranslation - 1.5;
          double xTranslationError = xTranslation;

          double robotSkew = Math.toDegrees(Math.atan2(xTranslation, zTranslation));
          relativeAngle = driveSubsystem.getGyroYaw() + robotSkew;

          relativeAngle = driveSubsystem.getAngleError(cameraPose[4]);

          SmartDashboard.putNumber("Target Yaw", robotSkew);

          SmartDashboard.putNumber("Relative Angle", relativeAngle);

          neededAngle = 90 + Math.toDegrees(Math.atan2(zTranslationError, xTranslationError));

          neededDistance = -Math.sqrt((xTranslationError * xTranslationError) +
              (zTranslationError * zTranslationError));
          // neededDistance = Math.hypot(zTranslationError, xTranslationError);

          // if (zTranslationError < 0 && xTranslationError > 0) {
          // neededDistance *= -1;
          // }

          if (neededAngle > 90) {
            neededAngle -= 180;
            neededDistance *= -1;
          }

          if (neededAngle < -90) {
            neededAngle += 180;
            neededDistance *= -1;
          }

        }, driveSubsystem),
        new AutoTurnToAngleCommand(() -> neededAngle, driveSubsystem),
        new WaitCommand(0.15),
        new AutoDriveCommand(() -> neededDistance, driveSubsystem),
        new WaitCommand(0.15),
        new AutoTurnToAngleCommand(() -> 0, driveSubsystem),
        new WaitCommand(0.15),
        new AlignToTapeCommand(driveSubsystem));
  }
}
