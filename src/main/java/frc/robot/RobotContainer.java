// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Buttons;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.apriltag.AlignToAprilTagCommand;
import frc.robot.commands.autonomous.routines.ConeOnMidAutoCommand;
import frc.robot.commands.autonomous.routines.DriveOnChargeStationAuto;
import frc.robot.commands.autonomous.routines.MobilityAutoCommand;
import frc.robot.commands.autonomous.routines.PlaceGPAndMobilityAuto;
import frc.robot.commands.autonomous.routines.PlaceGPBalanceAuto;
import frc.robot.commands.drive.AutoBalanceCommand;
import frc.robot.commands.drive.DriveToTapeCommand;
import frc.robot.commands.drive.PathPlannerCommand;
import frc.robot.commands.drive.TankDriveCommand;
import frc.robot.commands.drive.TurnToAngleCommand;
import frc.robot.commands.manipulator.ManualMoveElevatorCommand;
import frc.robot.commands.manipulator.SetElevatorPositionCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.PathPlannerUtil;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

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
    PathPlannerUtil.initializeCommands(driveSubsystem, elevatorSubsystem, intakeSubsystem);

    // autoChooser.setDefaultOption("Drive Backwards", new
    // MobilityAutoCommand(driveSubsystem));
    // autoChooser.addOption("Place Cone", new ConeOnMidAutoCommand(intakeSubsystem,
    // driveSubsystem));
    // autoChooser.addOption("Drive Back and Balance", new
    // DriveOnChargeStationAuto(driveSubsystem));
    // autoChooser.addOption("Place Cone and Drive Back", new
    // PlaceGPAndMobilityAuto(intakeSubsystem, driveSubsystem));
    // autoChooser.addOption("Place Cone and Balance", new
    // PlaceGPBalanceAuto(intakeSubsystem, driveSubsystem));

    autoChooser.addOption("(Substation Side) Place Cone, Drive Backwards",
        new PathPlannerCommand("Substation Place Cone Drive Backwards", driveSubsystem));

    autoChooser.addOption("(Substation Side) Place Cube, Grab Cone",
        new PathPlannerCommand("Substation Place Cube Grab Cone", driveSubsystem));

    autoChooser.addOption("(Substation Side) Place Cube, Grab Cone, Balance",
        new PathPlannerCommand("Substation Place Cube Grab Cone and Balance", driveSubsystem));

    autoChooser.addOption("(Charge Station) Place Cone, Balance",
        new PathPlannerCommand("Charge Station Place Cone Balance", driveSubsystem));

    autoChooser.addOption("(Field Edge) Place Cone, Drive Backwards",
        new PathPlannerCommand("Field Edge Place Cone Drive Backwards", driveSubsystem));

    autoChooser.addOption("(Field Edge) Place Cube, Grab Cone",
        new PathPlannerCommand("Field Edge Place Cube Grab Cone", driveSubsystem));

    autoChooser.addOption("(Field Edge) Place Cube, Grab Cone, Balance",
        new PathPlannerCommand("Field Edge Place Cube Grab Cone and Balance", driveSubsystem));

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
  }

  /**
   * Configures all drive triggers and buttons
   */
  public void configureDriveButtons() {
    Trigger slowDrive = driverController.leftTrigger();
    // charlie was here
    Trigger turnToAngle = new Trigger(() -> driverControllerHID.getPOV() != -1);

    Trigger autoBalance = driverController.rightTrigger();

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
            new TurnToAngleCommand(() -> Math.IEEEremainder(driverControllerHID.getPOV(), 360),
                driveSubsystem));

    autoBalance.whileTrue(new AutoBalanceCommand(driveSubsystem));

    toggleBrake.toggleOnTrue(Commands.runOnce(driveSubsystem::toggleBrakes, driveSubsystem));

    alignToTape.whileTrue(new DriveToTapeCommand(driveSubsystem));

    alignToAprilTag.whileTrue(new AlignToAprilTagCommand(driveSubsystem));

    setPipelineTape
        .toggleOnTrue(
            Commands.runOnce(() -> LimelightHelpers.setPipelineIndex(DriveConstants.limelightName, 0),
                driveSubsystem));

    setPipelineAprilTag
        .toggleOnTrue(
            Commands.runOnce(() -> LimelightHelpers.setPipelineIndex(DriveConstants.limelightName, 1),
                driveSubsystem));
  }

  /**
   * Configures all the buttons and triggers for the elevator
   */
  public void configureElevatorButtons() {
    Trigger cubeMode = new JoystickButton(operatorStick, Buttons.cubeModeButton);

    Trigger elevatorTilt = new JoystickButton(operatorStick, Buttons.toggleElevatorPistonsButton);

    Trigger coneElevatorHigh = new JoystickButton(operatorStick, Buttons.elevatorHighButton).and(cubeMode.negate());
    Trigger coneElevatorMid = new JoystickButton(operatorStick, Buttons.elevatorMidButton).and(cubeMode.negate());
    Trigger coneElevatorLow = new JoystickButton(operatorStick, Buttons.elevatorLowButton).and(cubeMode.negate());

    Trigger cubeElevatorHigh = new JoystickButton(operatorStick, Buttons.elevatorHighButton).and(cubeMode);
    Trigger cubeElevatorMid = new JoystickButton(operatorStick, Buttons.elevatorMidButton).and(cubeMode);
    Trigger cubeElevatorLow = new JoystickButton(operatorStick, Buttons.elevatorLowButton).and(cubeMode);

    Trigger elevatorUp = new JoystickButton(operatorStick, Buttons.elevatorManualUpButton);
    Trigger elevatorDown = new JoystickButton(operatorStick, Buttons.elevatorManualDownButton);

    Trigger startingConfiguration = new JoystickButton(operatorStick, 14);

    startingConfiguration.whileTrue(Commands.sequence(
        Commands.runOnce(elevatorSubsystem::elevatorTiltIn, elevatorSubsystem), new WaitCommand(1.00),
        new SetElevatorPositionCommand(IntakeConstants.startingConfigurationHeight, elevatorSubsystem)));

    elevatorTilt.toggleOnTrue(Commands.runOnce(elevatorSubsystem::toggleElevatorTilt, elevatorSubsystem));

    coneElevatorHigh
        .whileTrue(new SetElevatorPositionCommand(IntakeConstants.elevatorConeTopSetPoint, elevatorSubsystem));
    coneElevatorMid
        .whileTrue(new SetElevatorPositionCommand(IntakeConstants.elevatorConeMidSetPoint, elevatorSubsystem));
    coneElevatorLow
        .whileTrue(new SetElevatorPositionCommand(IntakeConstants.elevatorConeLowSetPoint, elevatorSubsystem));

    cubeElevatorHigh
        .whileTrue(new SetElevatorPositionCommand(IntakeConstants.elevatorCubeTopSetPoint, elevatorSubsystem));
    cubeElevatorMid
        .whileTrue(new SetElevatorPositionCommand(IntakeConstants.elevatorCubeMidSetPoint, elevatorSubsystem));
    cubeElevatorLow
        .whileTrue(new SetElevatorPositionCommand(IntakeConstants.elevatorCubeLowSetPoint, elevatorSubsystem));

    elevatorUp.whileTrue(new ManualMoveElevatorCommand(-IntakeConstants.elevatorSpeed, elevatorSubsystem));
    elevatorDown.whileTrue(new ManualMoveElevatorCommand(IntakeConstants.elevatorSpeed, elevatorSubsystem));
  }

  /**
   * Configures all the buttons for the intake
   */
  public void configureIntakeButtons() {
    Trigger cubeMode = new JoystickButton(operatorStick, Buttons.cubeModeButton);

    Trigger intakeCone = new JoystickButton(operatorStick, Buttons.intakeButton).and(cubeMode.negate());
    Trigger outtakeCone = new JoystickButton(operatorStick, Buttons.outtakeButton).and(cubeMode.negate());

    Trigger intakeCube = new JoystickButton(operatorStick, Buttons.intakeButton).and(cubeMode);
    Trigger outtakeCube = new JoystickButton(operatorStick, Buttons.outtakeButton).and(cubeMode);

    Trigger intakePiston = new JoystickButton(operatorStick, Buttons.intakeOpenClose);

    intakeCone.whileTrue(Commands.run(intakeSubsystem::intakeCone, intakeSubsystem))
        .toggleOnFalse(Commands.runOnce(intakeSubsystem::stopIntakeMotor, intakeSubsystem));

    outtakeCone.whileTrue(Commands.run(intakeSubsystem::outTakeCone, intakeSubsystem))
        .toggleOnFalse(Commands.runOnce(intakeSubsystem::stopIntakeMotor, intakeSubsystem));

    intakeCube.whileTrue(Commands.run(intakeSubsystem::intakeCube, intakeSubsystem))
        .toggleOnFalse(Commands.runOnce(intakeSubsystem::stopIntakeMotor, intakeSubsystem));

    outtakeCube.whileTrue(Commands.run(intakeSubsystem::outTakeCube, intakeSubsystem))
        .toggleOnFalse(Commands.runOnce(intakeSubsystem::stopIntakeMotor, intakeSubsystem));

    intakePiston.toggleOnTrue(Commands.runOnce(intakeSubsystem::toggleIntakePiston, intakeSubsystem));
  }

  public void initializeOdometry(Command autoCommand) {
    if (autoCommand instanceof PathPlannerCommand) {
      return;
    }

    driveSubsystem.initializeFieldPosition(startingFieldPosition.getSelected());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerCommand("Test Path", driveSubsystem);
    // return Commands.run(() -> {});

    // An example command will be run in autonomous
    // return autoChooser.getSelected();
    // return Autos.exampleAuto(m_exampleSubsystem);
  }
}
