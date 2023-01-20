// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToTapeCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private PhotonCamera camera;

  double yaw = 0;

  /** Creates a new AlignToTapeCommand. */
  public AlignToTapeCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    camera = driveSubsystem.getCamera();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // driveSubsystem.alignToTape();
    PhotonPipelineResult result = camera.getLatestResult();

    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();

      yaw = target.getYaw();

      double turnSpeed = yaw / 160;

      if (Math.abs(turnSpeed) >= 0.254) {
        turnSpeed = (turnSpeed > 0) ? 0.254 : -0.254;
      }

      SmartDashboard.putNumber("Turn Speed", turnSpeed);

      driveSubsystem.arcadeDrive(0, turnSpeed);
    } else if (Math.abs(yaw) >= 5) {

      double turnSpeed = yaw / 160;

      if (Math.abs(turnSpeed) >= 0.254) {
        turnSpeed = (turnSpeed > 0) ? 0.254 : -0.254;
      }

      driveSubsystem.arcadeDrive(0, turnSpeed);
    } else {
      driveSubsystem.arcadeDrive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveSubsystem.alignedToTape();
  }
}
