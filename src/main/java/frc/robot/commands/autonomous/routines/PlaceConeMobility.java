package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.drive.AutoDrive;
import frc.robot.commands.drive.AutoTurnToAngle;
import frc.robot.commands.manipulator.SetElevatorPosition;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceConeMobility extends SequentialCommandGroup {

  /** Creates a new Auto Command. */
  public PlaceConeMobility(Elevator elevator, Intake intake, Drive drive) {

    addCommands(
        /*
         * When starting at position 1 or 3
         * Robot will be facing the node
         */
        Commands.runOnce(elevator::elevatorTiltOut, elevator),

        new WaitCommand(1.25),

        // Robot will outtake the game piece it started with
        intake.autoOuttakeCone(),

        // Robot will be facing the node, and will drive backward the calculated
        // distance to cross mobility line
        new AutoDrive(-4.0, drive),

        Commands.parallel(new SetElevatorPosition(IntakeConstants.elevatorConeLowSetPoint, elevator),
            new AutoTurnToAngle(() -> (DriverStation.getAlliance() == Alliance.Blue) ? 90 : -90,
                drive)));
  }
}
