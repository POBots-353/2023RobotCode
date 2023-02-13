// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.LimelightHelpers;

public class AlignToTapeCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;

  private PIDController turnController = new PIDController(0.0050, 0.00155, 0.00025);

  private double yaw = 0;

  private double timeAligned = 0;

  /** Creates a new AlignToTapeCommand. */
  public AlignToTapeCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeAligned = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getTV(DriveConstants.limelightName)) {
      yaw = LimelightHelpers.getTX(DriveConstants.limelightName);

      double turnSpeed = -turnController.calculate(yaw, 0);

      driveSubsystem.arcadeDrive(0, MathUtil.clamp(turnSpeed, -0.15, 0.15));
    } else {
      driveSubsystem.arcadeDrive(0, 0);
    }

    if (driveSubsystem.alignedToTapeYaw()) {
      timeAligned++;
    } else if (timeAligned > 0) {
      timeAligned--;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveSubsystem.alignedToTapeYaw() && timeAligned >= 10;
  }
}
