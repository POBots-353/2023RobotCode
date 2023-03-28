package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.drive.AutoDrive;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BalanceAuto extends SequentialCommandGroup {
  /** Creates a new Auto Command. */
  public BalanceAuto(Elevator elevator, Intake intake,
      LEDs leds, Drive drive) {

    addCommands(
        // drives on station
        new AutoDrive(1.50, drive),

        Commands.runOnce(elevator::elevatorTiltIn),

        Commands.runOnce(intake::toggleWristIn),
        // balance
        new AutoBalance(leds, drive));
  }
}
