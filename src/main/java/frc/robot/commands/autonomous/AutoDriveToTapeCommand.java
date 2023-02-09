// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveToTapeCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private Limelight camera;

  private double yaw = 0;
  private double pitch = 0;

  private PIDController forwardController = new PIDController(0.0135, 0, 0);
  private PIDController turnController = new PIDController(0.0050, 0, 0);

  private int timeAligned = 0;

  /** Creates a new AutoDriveToTape. */
  public AutoDriveToTapeCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    camera = driveSubsystem.getCamera();

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
    // PhotonPipelineResult result = camera.getLatestResult();

    if (camera.hasTarget()) {
      // PhotonTrackedTarget target = result.getBestTarget();

      yaw = camera.getTargetX();
      pitch = camera.getTargetY();

      double turnSpeed = -turnController.calculate(yaw, 0);
      double forwardSpeed = forwardController.calculate(pitch, DriveConstants.tapeAlignmentPitch);

      driveSubsystem.arcadeDrive(MathUtil.clamp(forwardSpeed, -0.15, 0.15), MathUtil.clamp(turnSpeed, -0.15, 0.15));
    } else if (Math.abs(yaw) >= 5 || Math.abs(pitch - DriveConstants.tapeAlignmentPitch) >= 5) {

      double turnSpeed = -turnController.calculate(yaw, 0);
      double forwardSpeed = forwardController.calculate(pitch, DriveConstants.tapeAlignmentPitch);

      driveSubsystem.arcadeDrive(MathUtil.clamp(forwardSpeed, -0.15, 0.15), turnSpeed);
    } else {
      driveSubsystem.arcadeDrive(0, 0);
    }

    // Vibrate the controller if the robot is aligned
    if (driveSubsystem.alignedToTapeYaw() && driveSubsystem.alignedToTapePitch()) {
      RobotContainer.driverControllerHID.setRumble(RumbleType.kRightRumble, 1.00);
      SmartDashboard.putBoolean("Target Aligned", true);
      timeAligned++;
    } else {
      RobotContainer.driverControllerHID.setRumble(RumbleType.kRightRumble, 0);
      SmartDashboard.putBoolean("Target Aligned", false);

      if (timeAligned > 0) {
        timeAligned--;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driverControllerHID.setRumble(RumbleType.kRightRumble, 0);
    SmartDashboard.putBoolean("Target Aligned", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveSubsystem.alignedToTapeYaw() && driveSubsystem.alignedToTapePitch() && timeAligned >= 5;
  }
}
