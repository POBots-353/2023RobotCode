// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class AutoBalanceCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private LEDSubsystem ledSubsystem;

  private PIDController balancePIDController = new PIDController(0.012, 0, 0.00115); // kD 0.00125

  /** Creates a new AutoBalanceCommand. */
  public AutoBalanceCommand(LEDSubsystem ledSubsystem, DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.ledSubsystem = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem, ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    balancePIDController.setP(0.012);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double gyroPitch = -driveSubsystem.getGyroPitch();

    if (Math.abs(gyroPitch) <= 2.0) {
      driveSubsystem.arcadeDrive(0, 0);
      SmartDashboard.putBoolean("Balanced", true);
      ledSubsystem.setGreen();
      return;
    }

    if (Math.abs(gyroPitch) < 8.0) {
      // if (Math.abs(gyroPitch) < 2.5) {
      // balancePIDController.setP(0.0085);
      // SmartDashboard.putBoolean("Balanced", false);
      // ledSubsystem.initializeAllianceColor();
      // } else {
      balancePIDController.setP(0.0074);
      SmartDashboard.putBoolean("Balanced", false);
      ledSubsystem.initializeAllianceColor();
      // }
    }

    double forwardSpeed = balancePIDController.calculate(gyroPitch, 0);

    driveSubsystem.arcadeDrive(forwardSpeed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.arcadeDrive(0, 0);

    SmartDashboard.putBoolean("Balanced", false);
    ledSubsystem.initializeAllianceColor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (DriverStation.isAutonomous() && DriverStation.getMatchTime() <= 0.353 && DriverStation.getMatchTime() >= 0) {
      driveSubsystem.turnBrakesOn();

      return true;
    }

    return false;
  }
}
