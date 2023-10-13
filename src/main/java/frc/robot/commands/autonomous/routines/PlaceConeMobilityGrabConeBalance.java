// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.drive.AutoDrive;
import frc.robot.commands.drive.AutoTurnToAngle;
import frc.robot.commands.manipulator.SetElevatorPosition;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceConeMobilityGrabConeBalance extends SequentialCommandGroup {
  // private boolean cancelDriveback = true;
  private int timeAbove10Degrees = 0;

  /** Creates a new PlaceConeMobilityGrabConeBalance. */
  public PlaceConeMobilityGrabConeBalance(Elevator elevator, Intake intake, LEDs leds, Drive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(elevator::elevatorTiltOut, elevator),

        new WaitCommand(1.25),

        // new SetElevatorPositionCommand(IntakeConstants.elevatorConeTopSetPoint,
        // elevatorSystem),

        // Robot will outtake the game piece it started with
        intake.autoOuttakeCone(),

        Commands.runOnce(() -> drive.setMaxOutput(0.28), drive),

        // Robot will be facing the node, and will drive backward the calculated
        // distance to go onto the station and balance
        new AutoDrive(-4.75, drive),

        // Commands.runOnce(() -> cancelDriveback = false),

        // Commands.runOnce(elevatorSystem::elevatorTiltIn, elevatorSystem),

        // Commands.runOnce(intakeSystem::toggleWristIn, intakeSystem),

        Commands.parallel(
            new AutoTurnToAngle(0, drive).withTimeout(1.75), // -10,
            new SetElevatorPosition(ElevatorConstants.elevatorConeLowSetPoint, elevator)),

        Commands.runOnce(() -> {
          // cancelDriveback = true;
          drive.setMaxOutput(0.40);
        }, drive),

        Commands.race(
            Commands.run(intake::intakeCone, intake),
            new AutoDrive(0.65, drive)),

        Commands.waitSeconds(0.50),

        Commands.parallel(
            Commands.runOnce(intake::stopIntakeMotor, intake),

            Commands.runOnce(() -> drive.setMaxOutput(0.50), drive),

            Commands.runOnce(elevator::elevatorTiltIn, elevator),

            Commands.runOnce(() -> timeAbove10Degrees = 0)),

        Commands.parallel(
            new SetElevatorPosition(ElevatorConstants.elevatorConeMidSetPoint, elevator),
            new AutoDrive(-3.5, drive)
                .until(() -> {
                  if (Math.abs(drive.getGyroPitch()) > 10) {
                    timeAbove10Degrees++;
                  } else if (timeAbove10Degrees > 0) {
                    timeAbove10Degrees--;
                  }
                  return timeAbove10Degrees >= 18;
                })),

        new AutoBalance(leds, drive));

    // Trigger emergencyDriveBack = new Trigger(
    // () -> DriverStation.isAutonomous() && cancelDriveback == false &&
    // DriverStation.getMatchTime() < 5
    // && DriverStation.getMatchTime() > 0
    // && Math.abs(driveSubsystem.getGyroYaw()) < 20);

    // emergencyDriveBack.whileTrue(
    // Commands.sequence(
    // Commands.parallel(
    // Commands.runOnce(() -> driveSubsystem.setMaxOutput(0.55), driveSubsystem),

    // Commands.runOnce(elevatorSystem::elevatorTiltIn, elevatorSystem)),

    // Commands.parallel(
    // new SetElevatorPositionCommand(IntakeConstants.elevatorConeMidSetPoint,
    // elevatorSystem),
    // new AutoDriveCommand(-2.5, driveSubsystem)
    // .until(() -> Math.abs(driveSubsystem.getGyroPitch()) > 10)),

    // new AutoBalanceCommand(ledSubsystem, driveSubsystem)));
  }
}
