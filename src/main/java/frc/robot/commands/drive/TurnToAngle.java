// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class TurnToAngle extends CommandBase {
  private Drive driveSubsystem;

  private DoubleSupplier angleSupplier;
  private double angle;

  private PIDController pidController = new PIDController(0.0125, 0, 0.00055); // p: 0.0047, d: 0.00115

  private double minSpeed = 0.050;

  private SlewRateLimiter turnLimiter = new SlewRateLimiter(1.50);

  /** Creates a new TurnToAngleCommand. */
  // Angle must be between -180 and 180
  public TurnToAngle(DoubleSupplier angleSupplier, Drive driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    this.angleSupplier = angleSupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  public TurnToAngle(double angle, Drive drive) {
    this(() -> angle, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnLimiter.reset(0);

    angle = angleSupplier.getAsDouble();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double expectedAngle = angleSupplier.getAsDouble();
    double angleError = driveSubsystem.getAngleError(angle);

    // double angleErrorAbs = Math.abs(angleError);

    // if (angleErrorAbs < 15) {
    // if (angleErrorAbs < 2.5) {
    // pidController.setP(0.0145); // 0.0135
    // pidController.setI(0.00375); // 0.00375
    // } else {
    // pidController.setP(0.0115); // 0.0115
    // pidController.setI(0.00275); // 0.00275
    // }
    // } else {
    // pidController.setP(0.0047);
    // pidController.setI(0);
    // }

    // double turnSpeed = angleError * 0.0027;

    // pidController.calculate(angleError, 0);

    double turnSpeed = -pidController.calculate(angleError, 0);

    if (Math.abs(turnSpeed) < minSpeed) {
      turnSpeed = Math.copySign(minSpeed, turnSpeed);
    }

    // MathUtil.clamp(turnSpeed, -0.35, 0.35)

    driveSubsystem.arcadeDrive(0, MathUtil.clamp(turnSpeed, -0.450, 0.450));
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
