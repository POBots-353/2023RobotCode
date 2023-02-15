// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.LimelightHelpers;

public class DriveToTapeCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;

  private double yaw = 0;
  private double pitch = 0;

  private PIDController forwardController = new PIDController(0.0135, 0, 0);
  private PIDController turnController = new PIDController(0.0050, 0, 0);

  /** Creates a new AlignToTapeCommand. */
  public DriveToTapeCommand(DriveSubsystem driveSubsystem) {
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
    if (LimelightHelpers.getTV(DriveConstants.limelightName)) {
      double degreesOffset = driveSubsystem.getConeDistanceFromCenter() / DriveConstants.mmDegreesOffsetRatio;

      yaw = LimelightHelpers.getTX(DriveConstants.limelightName) - degreesOffset;

      pitch = LimelightHelpers.getTY(DriveConstants.limelightName);

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
    } else {
      RobotContainer.driverControllerHID.setRumble(RumbleType.kRightRumble, 0);
      SmartDashboard.putBoolean("Target Aligned", false);
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
    return false;
  }
}
