// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.routines;

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
import frc.robot.subsystems.LEDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceConePlaceCube extends SequentialCommandGroup {
  /** Creates a new PlaceConePlaceCube. */
  public PlaceConePlaceCube(ElevatorSubsystem elevatorSystem, IntakeSubsystem intakeSystem,
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

        Commands.runOnce(() -> driveSubsystem.setMaxOutput(0.60), driveSubsystem),

        new AutoDriveCommand(-4.50, driveSubsystem),

        Commands.parallel(
            new AutoTurnToAngleCommand(0, driveSubsystem).withTimeout(2.00), // -10,
            new SetElevatorPositionCommand(IntakeConstants.elevatorCubeLowSetPoint, elevatorSystem)),

        Commands.runOnce(() -> {
          driveSubsystem.setMaxOutput(0.40);
        }, driveSubsystem),

        Commands.race(
            Commands.run(intakeSystem::intakeCube, intakeSystem),
            new AutoDriveCommand(0.65, driveSubsystem)),

        Commands.parallel(Commands.runOnce(
            () -> driveSubsystem.setMaxOutput(0.60)),
            Commands.runOnce(intakeSystem::stopIntakeMotor, intakeSystem)),

        new AutoTurnToAngleCommand(180, driveSubsystem).withTimeout(2.00),

        new AutoDriveCommand(5.00, driveSubsystem),

        intakeSystem.autoOuttakeCube(),

        new AutoDriveCommand(-4.75, driveSubsystem));
  }
}
