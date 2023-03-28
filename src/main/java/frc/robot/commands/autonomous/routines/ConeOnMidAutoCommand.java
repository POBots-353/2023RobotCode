package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AutoDriveCommand;
import frc.robot.commands.drive.AutoDriveToTapeCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConeOnMidAutoCommand extends SequentialCommandGroup {
  /** Creates a new Auto Command. */
  public ConeOnMidAutoCommand(Intake intake, LEDs leds, Drive drive) {

    addCommands(
        /*
         * When starting at position 1, 2, or 3
         * Robot will be facing the center of the field
         */
        // Robot will drive forward a slight calculated distance to get closer to the
        // node
        new AutoDriveCommand(0.2143125, drive),

        // Robot will align to the node
        new AutoDriveToTapeCommand(leds, drive),

        // Robot will outtake the game piece it started with
        intake.autoOuttakeCone());
  }
}
