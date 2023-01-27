// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngleCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;

  private DoubleSupplier angleSupplier;

  private PIDController pidController = new PIDController(0.0027, 0, 0);

  /** Creates a new TurnToAngleCommand. */
  // Angle must be between -180 and 180
  public TurnToAngleCommand(DoubleSupplier angleSupplier, DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    this.angleSupplier = angleSupplier;

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
    double expectedAngle = angleSupplier.getAsDouble();
    double angleError = driveSubsystem.getAngleError(expectedAngle);

    // double turnSpeed = angleError * 0.0027;

    // pidController.calculate(angleError, 0);

    double turnSpeed = -pidController.calculate(angleError, 0);

    driveSubsystem.arcadeDrive(0, MathUtil.clamp(turnSpeed, -0.35, 0.35));
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
