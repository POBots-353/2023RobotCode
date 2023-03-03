package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.drive.AlignToTapeCommand;
import frc.robot.commands.drive.AutoDriveCommand;
import frc.robot.commands.drive.AutoTurnToAngleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElementTransitSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceGPAndMobilityAuto extends SequentialCommandGroup {

  /** Creates a new Auto Command. */
  public PlaceGPAndMobilityAuto(ElementTransitSubsystem transitSystem, DriveSubsystem driveSubsystem) {

    addCommands(
        /*
         * When starting at position 1 or 3
         * Robot will be facing the node
         */
        // Robot will drive forward a slight calculated distance to get closer to the
        // node
        new AutoDriveCommand(0.2143125, driveSubsystem),

        // Robot will align to the node
        new AlignToTapeCommand(driveSubsystem),

        // Robot will outtake the game piece it started with
        transitSystem.autoOuttakeCone(),

        // Robot will be facing the node, and will drive backward the calculated
        // distance to cross mobility line
        new AutoDriveCommand(-1.3573125, driveSubsystem));
  }
}
