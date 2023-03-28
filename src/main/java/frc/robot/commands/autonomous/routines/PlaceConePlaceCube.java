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
import frc.robot.commands.drive.AutoDrive;
import frc.robot.commands.drive.AutoTurnToAngle;
import frc.robot.commands.manipulator.SetElevatorPositionCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceConePlaceCube extends SequentialCommandGroup {
  /** Creates a new PlaceConePlaceCube. */
  public PlaceConePlaceCube(Elevator elevator, Intake intake, LEDs leds, Drive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(elevator::elevatorTiltOut, elevator),

        new WaitCommand(1.25),

        // new SetElevatorPositionCommand(IntakeConstants.elevatorConeTopSetPoint,
        // elevatorSystem),

        // Robot will outtake the game piece it started with
        intake.autoOuttakeCone(),

        Commands.runOnce(() -> drive.setMaxOutput(0.45), drive),

        new AutoDrive(-4.25, drive),

        Commands.parallel(
            new AutoTurnToAngle(() -> (DriverStation.getAlliance() == Alliance.Blue) ? 18.5 : -18.5,
                drive)
                .withTimeout(2.00),
            new SetElevatorPositionCommand(IntakeConstants.elevatorCubeLowSetPoint, elevator)),

        Commands.runOnce(() -> {
          drive.setMaxOutput(0.40);
        }, drive),

        Commands.race(
            Commands.run(intake::intakeCube, intake),
            new AutoDrive(1.00, drive)),

        Commands.parallel(Commands.runOnce(
            () -> drive.setMaxOutput(0.25)),
            Commands.runOnce(intake::stopIntakeMotor, intake)),

        new AutoTurnToAngle(180, drive).withTimeout(3.00)

    // Commands.parallel(
    // new AutoDriveCommand(4.75, driveSubsystem),
    // new SetElevatorPositionCommand(-15.0, elevatorSystem)),

    // intakeSystem.autoOuttakeCube(),

    // new AutoDriveCommand(-4.25, driveSubsystem)
    );
  }
}
