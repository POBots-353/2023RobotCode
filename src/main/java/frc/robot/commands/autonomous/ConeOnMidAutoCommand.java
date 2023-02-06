package frc.robot.commands.autonomous;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignToTapeCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConeOnMidAutoCommand extends SequentialCommandGroup {
  

  /** Creates a new Auto Command. */
  public ConeOnMidAutoCommand(DriveSubsystem driveSubsystem) {

    addCommands(
      /* When starting at position 1, 2, or 3
       * Robot will be facing the center of the field*/

      // Robot will turn around 180 degrees to face the node
      new AutoTurnToAngleCommand(180, driveSubsystem),

      // Robot will drive forward a slight calculated distance to get closer to the node
      new AutoDriveCommand(0.2143125, driveSubsystem),

      // Robot will align to the node
      new AlignToTapeCommand(driveSubsystem)

      // Robot will outtake the game piece it started with
    );
  }
}
