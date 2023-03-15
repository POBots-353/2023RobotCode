package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AutoBalanceCommand;
import frc.robot.commands.drive.AutoDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveOnChargeStationAuto extends SequentialCommandGroup {
  /** Creates a new Auto Command. */
  public DriveOnChargeStationAuto(ElevatorSubsystem elevatorSystem, IntakeSubsystem intakeSystem,
      LEDSubsystem ledSubsystem, DriveSubsystem driveSubsystem) {

    addCommands(
        // drives on station
        new AutoDriveCommand(1.50, driveSubsystem),

        Commands.runOnce(elevatorSystem::elevatorTiltIn),

        Commands.runOnce(intakeSystem::toggleWristIn),
        // balance
        new AutoBalanceCommand(ledSubsystem, driveSubsystem));
  }
}
