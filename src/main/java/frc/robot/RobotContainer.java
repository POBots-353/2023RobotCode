// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignToTapeCommand;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TankDriveCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElementTransitSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // driveSubsystem.setDefaultCommand(
    // new ArcadeDriveCommand(driverController::getLeftY,
    // driverController::getRightX, driveSubsystem));

    driveSubsystem.setDefaultCommand(
        new TankDriveCommand(driverController::getLeftY, driverController::getRightY,
            driveSubsystem));
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    Trigger motorIntake = new JoystickButton(operatorStick, 6);
    Trigger shortClaws = new JoystickButton(operatorStick, 5);
    Trigger longClaws = new JoystickButton(operatorStick, 8);
    Trigger elevatorTilt = new JoystickButton(operatorStick, 7);
    Trigger topSetpoint = new JoystickButton(operatorStick, 9);
    Trigger midSetpoint = new JoystickButton(operatorStick, 10);
    Trigger lowSetpoint = new JoystickButton(operatorStick, 11);

    Trigger autoBalance = driverController.leftBumper();
    autoBalance.whileTrue(new AutoBalanceCommand(driveSubsystem));

    // Set the camera pipeline to reflective tape
    Trigger setPipelineTape = driverController.start();
    setPipelineTape
        .toggleOnTrue(Commands.runOnce(() -> driveSubsystem.getCamera().setPipelineIndex(0), driveSubsystem));

    Trigger setPipelineAprilTag = driverController.back();
    setPipelineAprilTag
        .toggleOnTrue(Commands.runOnce(() -> driveSubsystem.getCamera().setPipelineIndex(1), driveSubsystem));

    // Turn to angle
    // Uses IEEEremainder to get the angle between -180 and 180
    new Trigger(() -> driverControllerHID.getPOV() != -1)
        .whileTrue(new TurnToAngleCommand(() -> Math.IEEEremainder(driverControllerHID.getPOV(), 360),
            driveSubsystem));

    // Slow drive
    Trigger leftTrigger = driverController.leftTrigger();
    leftTrigger.whileTrue(
        Commands.run(
            () -> driveSubsystem.tankDrive(-driverController.getLeftY() * DriveConstants.slowSpeed,
                -driverController.getRightY() * DriveConstants.slowSpeed),
            driveSubsystem));

    // Align to tape
    Trigger alignToTape = driverController.rightBumper();
    alignToTape.whileTrue(new AlignToTapeCommand(driveSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    motorIntake.onTrue(Commands.run(transitSubsystem::runClawMotors, transitSubsystem));
    motorIntake.onFalse(Commands.run(transitSubsystem::stopClawMotors, transitSubsystem));// trigger claw motors on/off
    shortClaws.onTrue(Commands.run(transitSubsystem::toggleShort, transitSubsystem));
    longClaws.onTrue(Commands.run(transitSubsystem::toggleLong, transitSubsystem));// toggles claw half or full
    elevatorTilt.toggleOnTrue(Commands.run(transitSubsystem::elevatorTiltOn));
    topSetpoint.toggleOnTrue(Commands.run(transitSubsystem::elevatorHigh, transitSubsystem));
    midSetpoint.toggleOnTrue(Commands.run(transitSubsystem::elevatorMid, transitSubsystem));
    lowSetpoint.toggleOnTrue(Commands.run(transitSubsystem::elevatorLow, transitSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return new AutoDriveCommand(5.00, driveSubsystem);
    return Commands.sequence(new AutoDriveCommand(1.00, driveSubsystem), new AutoBalanceCommand(driveSubsystem));
    // return Autos.exampleAuto(m_exampleSubsystem);
  }
}
