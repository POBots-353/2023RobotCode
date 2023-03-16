package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.drive.AutoBalanceCommand;
import frc.robot.commands.drive.AutoDriveCommand;
import frc.robot.commands.drive.DriveToTapeCommand;
import frc.robot.commands.manipulator.SetElevatorPositionCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceGPBalanceAuto extends SequentialCommandGroup {

  /** Creates a new Auto Command. */
  public PlaceGPBalanceAuto(ElevatorSubsystem elevatorSystem, IntakeSubsystem intakeSystem, LEDSubsystem ledSubsystem,
      DriveSubsystem driveSubsystem) {
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
        new SetElevatorPositionCommand(IntakeConstants.elevatorConeTopSetPoint, elevatorSystem),

        Commands.runOnce(elevatorSystem::elevatorTiltOut, elevatorSystem),

        new WaitCommand(2.00),

        // Robot will outtake the game piece it started with
        intakeSystem.autoOuttakeCone(),

        Commands.runOnce(elevatorSystem::elevatorTiltIn, elevatorSystem),

        Commands.runOnce(intakeSystem::toggleWristIn, intakeSystem),

        // Robot will be facing the node, and will drive backward the calculated
        // distance to go onto the station and balance
        new AutoDriveCommand(-2.5, driveSubsystem).until(() -> Math.abs(driveSubsystem.getGyroPitch()) > 7.5),

        // Robot will balance on the charge station
        new AutoBalanceCommand(ledSubsystem, driveSubsystem));
  }
}
