// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public class TankDriveCommand extends CommandBase {
  private DoubleSupplier leftSupplier;
  private DoubleSupplier rightSupplier;

  private Drive driveSubsystem;

  /** Creates a new TankDriveCommand. */
  public TankDriveCommand(DoubleSupplier leftSupplier, DoubleSupplier rightSupplier, Drive driveSubsystem) {
    this.leftSupplier = leftSupplier;
    this.rightSupplier = rightSupplier;

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
    driveSubsystem.tankDrive(-leftSupplier.getAsDouble() * DriveConstants.defaultSpeed,
        -rightSupplier.getAsDouble() * DriveConstants.defaultSpeed);
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
