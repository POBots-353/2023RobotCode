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
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceConeGrabConeBalance extends SequentialCommandGroup {
  /** Creates a new PlaceConeGrabConeBalance. */
  public PlaceConeGrabConeBalance(Elevator elevator, Intake intake, Drive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(elevator::elevatorTiltOut, elevator),

        new WaitCommand(1.50),

        new SetElevatorPositionCommand(IntakeConstants.elevatorConeTopSetPoint, elevator),

        // Robot will outtake the game piece it started with
        intake.autoOuttakeCone(),

        new AutoDriveCommand(-4.50, drive),

        new AutoTurnToAngleCommand(() -> (DriverStation.getAlliance() == Alliance.Blue) ? 19.25 : -19.25,
            drive),

        Commands.parallel(intake.autoIntakeCone(), new AutoDriveCommand(0.25, drive)),

        new AutoTurnToAngleCommand(() -> (DriverStation.getAlliance() == Alliance.Blue) ? 19.25 + 90 : -19.25 - 90,
            drive));
  }
}
