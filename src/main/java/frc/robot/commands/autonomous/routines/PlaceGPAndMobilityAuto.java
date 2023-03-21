package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.drive.AlignToTapeCommand;
import frc.robot.commands.drive.AutoDriveCommand;
import frc.robot.commands.drive.AutoTurnToAngleCommand;
import frc.robot.commands.manipulator.SetElevatorPositionCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceGPAndMobilityAuto extends SequentialCommandGroup {

  /** Creates a new Auto Command. */
  public PlaceGPAndMobilityAuto(ElevatorSubsystem elevatorSystem, IntakeSubsystem intakeSystem,
      DriveSubsystem driveSubsystem) {

    addCommands(
        /*
         * When starting at position 1 or 3
         * Robot will be facing the node
         */
        Commands.runOnce(elevatorSystem::elevatorTiltOut, elevatorSystem),

        new WaitCommand(1.25),

        // Robot will outtake the game piece it started with
        intakeSystem.autoOuttakeCone(),

        // Robot will be facing the node, and will drive backward the calculated
        // distance to cross mobility line
        new AutoDriveCommand(-4.0, driveSubsystem),

        Commands.parallel(new SetElevatorPositionCommand(IntakeConstants.elevatorConeLowSetPoint, elevatorSystem),
            new AutoTurnToAngleCommand(() -> (DriverStation.getAlliance() == Alliance.Blue) ? 90 : -90,
                driveSubsystem)));
  }
}
