// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.drive.AutoDrive;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceConeMobilityBalance extends SequentialCommandGroup {
  /** Creates a new PlaceConeMobilityBalance. */
  public PlaceConeMobilityBalance(Elevator elevator, Intake intake, LEDs leds, Drive drive) {
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
        new AutoDrive(-4.25, drive),

        Commands.waitSeconds(0.10),

        Commands.parallel(
            Commands.runOnce(elevator::elevatorTiltIn, elevator),
            Commands.runOnce(() -> drive.setMaxOutput(0.45), drive)),

        // Commands.runOnce(intakeSystem::toggleWristIn, intakeSystem),

        new AutoDrive(2.25, drive).until(() -> Math.abs(drive.getGyroPitch()) > 10),

        new AutoBalance(leds, drive));
  }
}
