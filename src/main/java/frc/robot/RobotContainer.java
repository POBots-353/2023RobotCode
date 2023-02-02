// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Buttons;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveToTapeCommand;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SetElevatorPositionCommand;
import frc.robot.commands.TankDriveCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.apriltag.AlignToAprilTagCommand;
import frc.robot.commands.apriltag.DriveToSquaredCommand;
import frc.robot.commands.autonomous.AutoTurnToAngleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElementTransitSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ElementTransitSubsystem transitSubsystem = new ElementTransitSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  public static final XboxController driverControllerHID = driverController.getHID();
  public final static Joystick operatorStick = new Joystick(OperatorConstants.operatorStickPort);

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    autoChooser.setDefaultOption("Drive Backwards", new AutoDriveCommand(1.00, driveSubsystem));

    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the trigger bindings
    configureBindings();

    // driveSubsystem.setDefaultCommand(
    // new ArcadeDriveCommand(driverController::getLeftY,
    // driverController::getRightX, driveSubsystem));

    driveSubsystem.setDefaultCommand(
        new TankDriveCommand(driverController::getLeftY, driverController::getRightY, driveSubsystem));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    configureDriveButtons();

    configureElevatorButtons();

    configureIntakeButtons();

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  public void configureDriveButtons() {
    Trigger slowDrive = driverController.leftTrigger();

    Trigger turnToAngle = new Trigger(() -> driverControllerHID.getPOV() != -1);

    Trigger autoBalance = driverController.leftBumper();

    Trigger alignToTape = driverController.rightBumper();

    Trigger alignToAprilTag = driverController.y();

    Trigger setPipelineTape = driverController.start();
    Trigger setPipelineAprilTag = driverController.back();

    slowDrive.whileTrue(
        Commands.run(() -> driveSubsystem.tankDrive(-driverController.getLeftY() * DriveConstants.slowSpeed,
            -driverController.getRightY() * DriveConstants.slowSpeed), driveSubsystem));

    // Uses IEEEremainder to get the angle between -180 and 180
    turnToAngle
        .whileTrue(new TurnToAngleCommand(() -> Math.IEEEremainder(driverControllerHID.getPOV(), 360), driveSubsystem));

    autoBalance.whileTrue(new AutoBalanceCommand(driveSubsystem));

    alignToTape.whileTrue(new DriveToTapeCommand(driveSubsystem));

    alignToAprilTag.whileTrue(new AlignToAprilTagCommand(driveSubsystem));

    setPipelineTape
        .toggleOnTrue(Commands.runOnce(() -> driveSubsystem.getCamera().setPipelineIndex(0), driveSubsystem));

    setPipelineAprilTag
        .toggleOnTrue(Commands.runOnce(() -> driveSubsystem.getCamera().setPipelineIndex(1), driveSubsystem));
  }

  public void configureElevatorButtons() {
    JoystickButton elevatorTilt = new JoystickButton(operatorStick, Buttons.toggleElevatorPistonsButton);
    JoystickButton elevatorHigh = new JoystickButton(operatorStick, Buttons.elevatorHighButton);
    JoystickButton elevatorMid = new JoystickButton(operatorStick, Buttons.elevatorMidButton);
    JoystickButton elevatorLow = new JoystickButton(operatorStick, Buttons.elevatorMidButton);

    elevatorTilt.toggleOnTrue(Commands.runOnce(transitSubsystem::toggleElevatorTilt, transitSubsystem));

    elevatorHigh.whileTrue(new SetElevatorPositionCommand(IntakeConstants.elevatorTopSetPoint, transitSubsystem));
    elevatorMid.whileTrue(new SetElevatorPositionCommand(IntakeConstants.elevatorMidSetPoint, transitSubsystem));
    elevatorLow.whileTrue(new SetElevatorPositionCommand(IntakeConstants.elevatorLowSetPoint, transitSubsystem));
  }

  public void configureIntakeButtons() {
    JoystickButton inTake = new JoystickButton(operatorStick, Buttons.intakeInButton);
    JoystickButton outTake = new JoystickButton(operatorStick, Buttons.intakeOutButton);
    JoystickButton openCloseIntake = new JoystickButton(operatorStick, Buttons.intakeOpenClose);

    inTake.whileTrue(Commands.run(transitSubsystem::inTake, transitSubsystem))
        .toggleOnFalse(Commands.runOnce(transitSubsystem::stopClawMotors, transitSubsystem));

    outTake.whileTrue(Commands.run(transitSubsystem::outTake, transitSubsystem))
        .toggleOnFalse(Commands.runOnce(transitSubsystem::stopClawMotors, transitSubsystem));

    openCloseIntake.toggleOnTrue(Commands.runOnce(transitSubsystem::openCloseClaw, transitSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return new AutoDriveCommand(5.00, driveSubsystem);
    return autoChooser.getSelected();
    // return Autos.exampleAuto(m_exampleSubsystem);
  }
}
