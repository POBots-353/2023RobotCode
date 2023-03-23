// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoTurnToAngleCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;

  private DoubleSupplier angleSupplier;

  private PIDController pidController = new PIDController(0.0115, 0, 0.00185); // p: 0.0145 d: 0.00115, 0.00125

  private SlewRateLimiter turnLimiter = new SlewRateLimiter(3.75);

  private int timeAligned = 0;

  /** Creates a new AutoTurnToAngleCommand. */
  public AutoTurnToAngleCommand(DoubleSupplier angleSupplier, DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    this.angleSupplier = angleSupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  public AutoTurnToAngleCommand(double angle, DriveSubsystem driveSubsystem) {
    this(() -> angle, driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnLimiter.reset(0);
    timeAligned = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double expectedAngle = angleSupplier.getAsDouble();
    double angleError = driveSubsystem.getAngleError(expectedAngle);

    double angleErrorAbs = Math.abs(angleError);

    if (angleErrorAbs < 15) {
      // if (angleErrorAbs < 4.5) {
      //   pidController.setP(0.0125); // 0.0135
      //   pidController.setI(0.00375); // 0.00375
      // } else {
        pidController.setP(0.0115); // 0.0115
        pidController.setI(0); // 0.00275
      // }
    } else {
      pidController.setP(0.0115);
      pidController.setI(0);
    }

    double turnSpeed = MathUtil.clamp(-pidController.calculate(angleError, 0), -0.35, 0.35);

    double turnSpeedRateLimited = turnLimiter.calculate(turnSpeed);

    // MathUtil.clamp(turnSpeed, -0.35, 0.35)

    driveSubsystem.arcadeDrive(0, turnSpeedRateLimited);

    if (angleErrorAbs <= 2.50) {
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
    return Math.abs(driveSubsystem.getAngleError(angleSupplier.getAsDouble())) <= 2.50 && timeAligned >= 4;
  }
}
