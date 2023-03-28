// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public class DriveToPoseCommand extends CommandBase {
  private Drive driveSubsystem;

  private RamseteController ramseteController = new RamseteController(AutoConstants.kRamseteB,
      AutoConstants.kRamseteZeta);

  private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(AutoConstants.ksVolts,
      AutoConstants.kvVoltSecondsPerMeter,
      AutoConstants.kaVoltSecondsSquaredPerMeter);

  private PIDController pidController = new PIDController(AutoConstants.kPDriveVel, 0, 0);

  private Supplier<Trajectory> trajectorySupplier;

  private Trajectory trajectory;

  private DifferentialDriveWheelSpeeds previousSpeeds;

  private Timer timer = new Timer();

  private double previousTime;

  Pose2d goal;

  /** Creates a new DriveToPoseCommand. */
  public DriveToPoseCommand(Supplier<Trajectory> trajectory, Drive driveSubsystem) {
    trajectorySupplier = trajectory;
    this.driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    previousTime = -1;

    trajectory = trajectorySupplier.get();

    // driveSubsystem.resetOdometry(trajectory.getInitialPose());

    var initialState = trajectory.sample(0);
    previousSpeeds = DriveConstants.driveKinematics.toWheelSpeeds(
        new ChassisSpeeds(
            initialState.velocityMetersPerSecond,
            0,
            initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));

    pidController.reset();

    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double time = timer.get();
    double dt = time - previousTime;

    ChassisSpeeds chassiSpeed = ramseteController.calculate(driveSubsystem.getPose(), trajectory.sample(time));
    DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.driveKinematics.toWheelSpeeds(chassiSpeed);

    double leftSpeedSetpoint = wheelSpeeds.leftMetersPerSecond;
    double rightSpeedSetpoint = wheelSpeeds.rightMetersPerSecond;

    double leftFeedforward = feedForward.calculate(
        leftSpeedSetpoint, (leftSpeedSetpoint - previousSpeeds.leftMetersPerSecond) / dt);

    double rightFeedforward = feedForward.calculate(
        rightSpeedSetpoint, (rightSpeedSetpoint - previousSpeeds.rightMetersPerSecond) / dt);

    DifferentialDriveWheelSpeeds currentSpeeds = driveSubsystem.getWheelSpeeds();

    double leftOutput = leftFeedforward + pidController.calculate(currentSpeeds.leftMetersPerSecond, leftSpeedSetpoint);

    double rightOutput = rightFeedforward
        + pidController.calculate(currentSpeeds.rightMetersPerSecond, rightSpeedSetpoint);

    driveSubsystem.tankDriveVolts(leftOutput, rightOutput);

    previousSpeeds = wheelSpeeds;
    previousTime = time;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }
}
