// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class ArcadeDrive extends CommandBase {
  private DoubleSupplier forwardSupplier;
  private DoubleSupplier turnSupplier;

  private Drive driveSubsystem;

  /** Creates a new ArcadeDriveCommand. */
  public ArcadeDrive(DoubleSupplier forwardSupplier, DoubleSupplier turnSupplier,
      Drive driveSubsystem) {
    this.forwardSupplier = forwardSupplier;
    this.turnSupplier = turnSupplier;

    this.driveSubsystem = driveSubsystem;
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
    driveSubsystem.arcadeDrive(-forwardSupplier.getAsDouble() * 0.45, turnSupplier.getAsDouble() * 0.45); 
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
