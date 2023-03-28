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
public class PlaceConeBalance extends SequentialCommandGroup {

  /** Creates a new Auto Command. */
  public PlaceConeBalance(Elevator elevator, Intake intake, LEDs leds, Drive drive) {
    addCommands(
        /*
         * When starting at position 2
         * Robot will be facing the node
         */
        // Robot will drive forward a slight calculated distance to get closer to the
        // node
        // new AutoDriveCommand(0.2143125, driveSubsystem),

        // Robot will align to the node
        // new DriveToTapeCommand(driveSubsystem),
        Commands.runOnce(elevator::elevatorTiltOut, elevator),

        new WaitCommand(1.25),

        // Robot will outtake the game piece it started with
        intake.autoOuttakeCone(),

        Commands.runOnce(elevator::elevatorTiltIn, elevator),

        // Robot will be facing the node, and will drive backward the calculated
        // distance to go onto the station and balance
        new AutoDrive(-2.0, drive),

        // Robot will balance on the charge station
        new AutoBalance(leds, drive));
  }
}
