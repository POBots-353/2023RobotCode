// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.apriltag;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToSquaredCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private Limelight camera;

  private double neededDistance;

  private double xTranslationError;
  private double zTranslationError;

  private double desiredAngle;

  private boolean angleReached = false;

  private PIDController pidController = new PIDController(0.0035, 0.0015, 0);

  /** Creates a new DriveToSquaredCommand. */
  public DriveToSquaredCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    camera = driveSubsystem.getCamera();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleReached = false;
    desiredAngle = 0;
    driveSubsystem.resetEncoders();
    if (camera.hasTarget()) {
      NetworkTableEntry campose = camera.getTable().getEntry("campose");
      Double[] pose = campose.getDoubleArray(new Double[0]);

      if (pose.length > 0) {
        xTranslationError = pose[0];
        zTranslationError = -pose[2] - 1.5; // 1.5 meters in front of tag

        desiredAngle = 90 + Math.toDegrees(Math.atan(zTranslationError / xTranslationError));

        neededDistance = Math.sqrt((xTranslationError * xTranslationError) + (zTranslationError * zTranslationError));

        if (desiredAngle > 90) {
          desiredAngle -= 180;
          neededDistance *= -1;
        }

        if (desiredAngle < -90) {
          desiredAngle += 180;
          neededDistance *= -1;
        }
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!angleReached) {
      double angleError = driveSubsystem.getAngleError(desiredAngle);

      double turnSpeed = -pidController.calculate(angleError, 0);

      driveSubsystem.arcadeDrive(0, MathUtil.clamp(turnSpeed, -0.35, 0.35));

      angleReached = Math.abs(angleError) < 1.00;
    } else {
      driveSubsystem.autoDrive(neededDistance);

      if (driveSubsystem.distanceReached(neededDistance)) {
        angleReached = false;
        desiredAngle = 0;
      }
    }
    // driveSubsystem.autoDrive(neededDistance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return angleReached && desiredAngle == 0;
  }
}
