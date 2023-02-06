// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalanceCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;

  private PIDController balancePIDController = new PIDController(0.010, 0, 0.00125);

  /** Creates a new AutoBalanceCommand. */
  public AutoBalanceCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    balancePIDController.setP(0.010);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double gyroPitch = driveSubsystem.getGyroPitch();

    if (Math.abs(gyroPitch) <= 0.5) {
      driveSubsystem.arcadeDrive(0, 0);
      SmartDashboard.putBoolean("Balanced", true);
      return;
    }

    if (Math.abs(gyroPitch) < 5.5) {
      if (Math.abs(gyroPitch) < 2.5) {
        balancePIDController.setP(0.0085);
        SmartDashboard.putBoolean("Balanced", false);
      } else {
        balancePIDController.setP(0.0061);
        SmartDashboard.putBoolean("Balanced", false);
      }
    }

    double forwardSpeed = balancePIDController.calculate(gyroPitch, 0);

    driveSubsystem.arcadeDrive(forwardSpeed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.arcadeDrive(0, 0);

    SmartDashboard.putBoolean("Balanced", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
