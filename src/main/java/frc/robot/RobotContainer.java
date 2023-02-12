// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Buttons;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.autonomous.AutoBalanceCommand;
import frc.robot.commands.autonomous.AutoDriveCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.apriltag.AlignToAprilTagCommand;
import frc.robot.commands.autonomous.AutoTurnToAngleCommand;
import frc.robot.commands.autonomous.FollowTrajectoryCommand;
import frc.robot.commands.autonomous.routines.ConeOnMidAutoCommand;
import frc.robot.commands.autonomous.routines.DriveOnChargeStationAuto;
import frc.robot.commands.autonomous.routines.MobilityAutoCommand;
import frc.robot.commands.autonomous.routines.PlaceGPAndMobilityAuto;
import frc.robot.commands.autonomous.routines.PlaceGPBalanceAuto;
import frc.robot.commands.drive.ArcadeDriveCommand;
import frc.robot.commands.drive.DriveToTapeCommand;
import frc.robot.commands.drive.TankDriveCommand;
import frc.robot.commands.drive.TurnToAngleCommand;
import frc.robot.commands.manipulator.ManualMoveElevatorCommand;
import frc.robot.commands.manipulator.SetElevatorPositionCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElementTransitSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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

  private SendableChooser<Integer> startingFieldPosition = new SendableChooser<Integer>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    autoChooser.setDefaultOption("Drive Backwards", new MobilityAutoCommand(driveSubsystem));
    autoChooser.addOption("Place Cone", new ConeOnMidAutoCommand(transitSubsystem, driveSubsystem));
    autoChooser.addOption("Drive Back and Balance", new DriveOnChargeStationAuto(driveSubsystem));
    autoChooser.addOption("Place Cone and Drive Back", new PlaceGPAndMobilityAuto(transitSubsystem, driveSubsystem));
    autoChooser.addOption("Place Cone and Balance", new PlaceGPBalanceAuto(transitSubsystem, driveSubsystem));

    SmartDashboard.putData("Auto Chooser", autoChooser);

    startingFieldPosition.setDefaultOption("Aligned w/ Charge Station", 2);
    startingFieldPosition.addOption("Substation Side", 1);
    startingFieldPosition.addOption("Perimeter Side", 3);

    SmartDashboard.putData("Starting Field Position", startingFieldPosition);

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

  /**
   * Configures all drive triggers and buttons
   */
  public void configureDriveButtons() {
    Trigger slowDrive = driverController.leftTrigger();
    // charlie was here
    Trigger turnToAngle = new Trigger(() -> driverControllerHID.getPOV() != -1);

    Trigger autoBalance = driverController.leftBumper();

    Trigger toggleBrake = new JoystickButton(operatorStick, Buttons.toggleBrakesButton);

    Trigger alignToTape = driverController.rightBumper();

    Trigger alignToAprilTag = driverController.leftBumper();

    Trigger setPipelineTape = driverController.start();
    Trigger setPipelineAprilTag = driverController.back();

    slowDrive.whileTrue(
        Commands.run(() -> driveSubsystem.tankDrive(-driverController.getLeftY() * DriveConstants.slowSpeed,
            -driverController.getRightY() * DriveConstants.slowSpeed), driveSubsystem));

    // Uses IEEEremainder to get the angle between -180 and 180
    turnToAngle
        .whileTrue(
            new AutoTurnToAngleCommand(() -> Math.IEEEremainder(driverControllerHID.getPOV(), 360), driveSubsystem));

    autoBalance.whileTrue(new AutoBalanceCommand(driveSubsystem));

    toggleBrake.toggleOnTrue(Commands.runOnce(driveSubsystem::toggleBrakes, driveSubsystem));

    alignToTape.whileTrue(new DriveToTapeCommand(driveSubsystem));

    alignToAprilTag.whileTrue(new AlignToAprilTagCommand(driveSubsystem));

    setPipelineTape
        .toggleOnTrue(
            Commands.runOnce(() -> LimelightHelpers.setPipelineIndex(DriveConstants.limelightName, 0), driveSubsystem));

    setPipelineAprilTag
        .toggleOnTrue(
            Commands.runOnce(() -> LimelightHelpers.setPipelineIndex(DriveConstants.limelightName, 1), driveSubsystem));
  }

  /**
   * Configures all the buttons and triggers for the elevator
   */
  public void configureElevatorButtons() {
    JoystickButton elevatorTilt = new JoystickButton(operatorStick, Buttons.toggleElevatorPistonsButton);

    JoystickButton elevatorHigh = new JoystickButton(operatorStick, Buttons.elevatorHighButton);
    JoystickButton elevatorMid = new JoystickButton(operatorStick, Buttons.elevatorMidButton);
    JoystickButton elevatorLow = new JoystickButton(operatorStick, Buttons.elevatorLowButton);

    JoystickButton elevatorUp = new JoystickButton(operatorStick, Buttons.elevatorManualUpButton);
    JoystickButton elevatorDown = new JoystickButton(operatorStick, Buttons.elevatorManualUpButton);

    elevatorTilt.toggleOnTrue(Commands.runOnce(transitSubsystem::toggleElevatorTilt, transitSubsystem));

    elevatorHigh.whileTrue(new SetElevatorPositionCommand(IntakeConstants.elevatorTopSetPoint, transitSubsystem));
    elevatorMid.whileTrue(new SetElevatorPositionCommand(IntakeConstants.elevatorMidSetPoint, transitSubsystem));
    elevatorLow.whileTrue(new SetElevatorPositionCommand(IntakeConstants.elevatorLowSetPoint, transitSubsystem));

    elevatorUp.whileTrue(new ManualMoveElevatorCommand(-IntakeConstants.elevatorSpeed, transitSubsystem));

    elevatorDown.whileTrue(new ManualMoveElevatorCommand(IntakeConstants.elevatorSpeed, transitSubsystem));
  }

  /**
   * Configures all the buttons for the intake
   */
  public void configureIntakeButtons() {
    JoystickButton intakeCube = new JoystickButton(operatorStick, Buttons.intakeConeButton);
    JoystickButton intakeCone = new JoystickButton(operatorStick, Buttons.intakeCubeButton);
    JoystickButton intakePiston = new JoystickButton(operatorStick, Buttons.intakeOpenClose);

    intakeCube.whileTrue(Commands.run(transitSubsystem::intakeCube, transitSubsystem))
        .toggleOnFalse(Commands.runOnce(transitSubsystem::stopIntakeMotor, transitSubsystem));

    intakeCone.whileTrue(Commands.run(transitSubsystem::intakeCone, transitSubsystem))
        .toggleOnFalse(Commands.runOnce(transitSubsystem::stopIntakeMotor, transitSubsystem));

    intakePiston.toggleOnTrue(Commands.runOnce(transitSubsystem::toggleIntakePiston, transitSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String trajectoryJSON = "output/Testing.wpilib.json";

    return new FollowTrajectoryCommand(trajectoryJSON, driveSubsystem);
    // return Commands.run(() -> {});

    // An example command will be run in autonomous
    // return autoChooser.getSelected();
    // return Autos.exampleAuto(m_exampleSubsystem);
  }
}
