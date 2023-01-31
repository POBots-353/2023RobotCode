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
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.autonomous.AutoTurnToAngleCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignToAprilTagCommand extends SequentialCommandGroup {
  private double neededDistance;
  private double neededAngle;

  private Limelight camera;

  /** Creates a new AlignToAprilTagCommand. */
  public AlignToAprilTagCommand(DriveSubsystem driveSubsystem) {
    camera = driveSubsystem.getCamera();

    // Double[] cameraPose =
    // camera.getTable().getEntry("campose").getDoubleArray(new Double[0]);

    // if (cameraPose.length == 0) {
    // return;
    // }

    // double zTranslationError = -cameraPose[2] - 1.5;
    // double xTranslationError = cameraPose[0];

    // neededAngle = 90 + Math.toDegrees(Math.atan(zTranslationError /
    // xTranslationError));

    // neededDistance = Math.sqrt((xTranslationError * xTranslationError) +
    // (zTranslationError * zTranslationError));

    // if (neededAngle > 90) {
    // neededAngle -= 180;
    // neededDistance *= -1;
    // }

    // if (neededAngle < -90) {
    // neededAngle += 180;
    // neededDistance *= -1;
    // }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(() -> {
          Double[] cameraPose = camera.getTable().getEntry("campose").getDoubleArray(new Double[0]);

          if (cameraPose.length == 0) {
            return;
          }

          double zTranslationError = -cameraPose[2] - 1.5;
          double xTranslationError = cameraPose[0];

          neededAngle = 90 + Math.toDegrees(Math.atan(zTranslationError / xTranslationError));

          neededDistance = Math.sqrt((xTranslationError * xTranslationError) + (zTranslationError * zTranslationError));

          if (zTranslationError < 0 && xTranslationError > 0) {
            neededDistance *= -1;
          }

          SmartDashboard.putNumber("Needed Distance", neededDistance);

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
