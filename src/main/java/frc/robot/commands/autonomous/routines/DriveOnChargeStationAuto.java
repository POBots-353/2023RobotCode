package frc.robot.commands.autonomous.routines;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.AutoBalanceCommand;
import frc.robot.commands.autonomous.AutoDriveCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveOnChargeStationAuto extends SequentialCommandGroup {
  /** Creates a new Auto Command. */
  public DriveOnChargeStationAuto(DriveSubsystem driveSubsystem) {

    addCommands(
      // drives on station
      new AutoDriveCommand(1.00, driveSubsystem),
      // balance
      new AutoBalanceCommand(driveSubsystem)
      );
  }
}
