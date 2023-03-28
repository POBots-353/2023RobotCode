package frc.robot.commands.autonomous.routines;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AutoDrive;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MobilityAuto extends SequentialCommandGroup {
  

  /** Creates a new Auto Command. */
  public MobilityAuto(Drive drive) {

    addCommands(
      /* When starting at position 1 or 3
       * Robot will be facing the center of the field */

      // Robot will drive forward the calculated distance to cross mobility line
      new AutoDrive(-3.53, drive));
  }
}
