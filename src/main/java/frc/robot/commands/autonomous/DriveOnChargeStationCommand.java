package frc.robot.commands.autonomous;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveOnChargeStationCommand extends SequentialCommandGroup {
  

  /** Creates a new Auto Command. */
  public DriveOnChargeStationCommand(DriveSubsystem driveSubsystem) {

    addCommands(
      // drives on station
      new AutoDriveCommand(.8, driveSubsystem));

      // here will go balancing code
  }
}
