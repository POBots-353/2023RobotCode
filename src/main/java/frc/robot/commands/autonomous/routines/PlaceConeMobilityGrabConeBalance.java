// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.drive.AutoBalanceCommand;
import frc.robot.commands.drive.AutoDriveCommand;
import frc.robot.commands.drive.AutoTurnToAngleCommand;
import frc.robot.commands.manipulator.SetElevatorPositionCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceConeMobilityGrabConeBalance extends SequentialCommandGroup {
  /** Creates a new PlaceConeMobilityGrabConeBalance. */
  public PlaceConeMobilityGrabConeBalance(ElevatorSubsystem elevatorSystem, IntakeSubsystem intakeSystem,
      LEDSubsystem ledSubsystem, DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(elevatorSystem::elevatorTiltOut, elevatorSystem),

        new WaitCommand(1.25),

        // new SetElevatorPositionCommand(IntakeConstants.elevatorConeTopSetPoint,
        // elevatorSystem),

        // Robot will outtake the game piece it started with
        intakeSystem.autoOuttakeCone(),

        // Robot will be facing the node, and will drive backward the calculated
        // distance to go onto the station and balance
        new AutoDriveCommand(-4.25, driveSubsystem),

        // Commands.runOnce(elevatorSystem::elevatorTiltIn, elevatorSystem),

        // Commands.runOnce(intakeSystem::toggleWristIn, intakeSystem),

        Commands.parallel(
            new AutoTurnToAngleCommand(0, driveSubsystem), // -10,
            new SetElevatorPositionCommand(IntakeConstants.elevatorConeLowSetPoint, elevatorSystem)),

        Commands.runOnce(() -> driveSubsystem.setMaxVelocity(500), driveSubsystem),

        Commands.race(Commands.run(intakeSystem::intakeCone, intakeSystem),
            new AutoDriveCommand(0.90, driveSubsystem)),

        Commands.waitSeconds(0.50),

        Commands.parallel(
            Commands.runOnce(intakeSystem::stopIntakeMotor, intakeSystem),

            Commands.runOnce(() -> driveSubsystem.setMaxVelocity(600), driveSubsystem),

            Commands.runOnce(elevatorSystem::elevatorTiltIn, elevatorSystem)),

        Commands.parallel(
            new SetElevatorPositionCommand(IntakeConstants.elevatorConeMidSetPoint, elevatorSystem),
            new AutoDriveCommand(-2.5, driveSubsystem)
                .until(() -> Math.abs(driveSubsystem.getGyroPitch()) > 8.5)),

        new AutoBalanceCommand(ledSubsystem, driveSubsystem));
  }
}
