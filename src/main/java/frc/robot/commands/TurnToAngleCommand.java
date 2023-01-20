// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngleCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;

  private double angle;

  /** Creates a new TurnToAngleCommand. */
  // Angle must be 0 - 360
  public TurnToAngleCommand(double angle, DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    if (angle > 180) {
      angle -= 360;
    }

    this.angle = angle;
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
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
