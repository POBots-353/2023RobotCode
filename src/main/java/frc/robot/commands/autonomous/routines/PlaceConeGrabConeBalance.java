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
import frc.robot.commands.drive.AutoDriveCommand;
import frc.robot.commands.drive.AutoTurnToAngleCommand;
import frc.robot.commands.manipulator.SetElevatorPositionCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceConeGrabConeBalance extends SequentialCommandGroup {
  /** Creates a new PlaceConeGrabConeBalance. */
  public PlaceConeGrabConeBalance(ElevatorSubsystem elevatorSystem, IntakeSubsystem intakeSystem,
      DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(elevatorSystem::elevatorTiltOut, elevatorSystem),

        new WaitCommand(1.50),

        new SetElevatorPositionCommand(IntakeConstants.elevatorConeTopSetPoint, elevatorSystem),

        // Robot will outtake the game piece it started with
        intakeSystem.autoOuttakeCone(),

        new AutoDriveCommand(-4.50, driveSubsystem),

        new AutoTurnToAngleCommand(() -> (DriverStation.getAlliance() == Alliance.Blue) ? 19.25 : -19.25,
            driveSubsystem),

        Commands.parallel(intakeSystem.autoIntakeCone(), new AutoDriveCommand(0.25, driveSubsystem)),

        new AutoTurnToAngleCommand(() -> (DriverStation.getAlliance() == Alliance.Blue) ? 19.25 + 90 : -19.25 - 90,
            driveSubsystem));
  }
}
