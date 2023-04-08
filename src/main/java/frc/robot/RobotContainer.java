// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Buttons;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldPositionConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.apriltag.AlignToAprilTag;
import frc.robot.commands.autonomous.routines.PlaceConeAuto;
import frc.robot.commands.autonomous.routines.BalanceAuto;
import frc.robot.commands.autonomous.routines.MobilityAuto;
import frc.robot.commands.autonomous.routines.PlaceConeMobilityBalance;
import frc.robot.commands.autonomous.routines.PlaceConeMobilityGrabConeBalance;
import frc.robot.commands.autonomous.routines.PlaceConePlaceCubeLow;
import frc.robot.commands.autonomous.routines.PlaceConePlaceCubeMid;
import frc.robot.commands.autonomous.routines.PlaceConePlaceCubeMidCable;
import frc.robot.commands.autonomous.routines.PlaceConeMobility;
import frc.robot.commands.autonomous.routines.PlaceConeBalance;
import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.drive.AutoTurnToAngle;
import frc.robot.commands.drive.DriveToTape;
import frc.robot.commands.drive.FollowPathPlanner;
import frc.robot.commands.drive.TankDrive;
import frc.robot.commands.drive.TurnToAngle;
import frc.robot.commands.manipulator.ManualMoveElevator;
import frc.robot.commands.manipulator.SetElevatorPosition;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Elevator;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.PathPlannerUtil;

import java.text.FieldPosition;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
  private final Drive drive = new Drive();
  private final Intake intake = new Intake();
  private final Elevator elevator = new Elevator();
  private final LEDs leds = new LEDs();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  public static final XboxController driverControllerHID = driverController.getHID();
  public final static Joystick operatorStick = new Joystick(OperatorConstants.operatorStickPort);

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private static SendableChooser<Integer> startingFieldPosition = new SendableChooser<Integer>();

  public Command placeConeAutoStart(Command pathPlannerCommand) {
    return Commands.sequence(Commands.runOnce(elevator::elevatorTiltOut, elevator),
        new WaitCommand(1.50),

        new SetElevatorPosition(IntakeConstants.elevatorConeTopSetPoint,
            elevator),

        intake.autoOuttakeCone(),
        new WaitCommand(0.50),
        pathPlannerCommand);
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    PathPlannerUtil.initializeCommands(drive, elevator, intake);

    autoChooser.addOption("Drive Backwards", new MobilityAuto(drive));

    autoChooser.addOption("Place Cone", new PlaceConeAuto(intake, leds, drive));

    autoChooser.addOption("Place Cone, Mobility",
        new PlaceConeMobility(elevator, intake,
            drive));

    autoChooser.addOption("Drive Back, Balance", new BalanceAuto(elevator, intake, leds, drive));

    autoChooser.addOption("Place Cone, Balance",
        new PlaceConeBalance(elevator, intake, leds, drive));

    autoChooser.addOption("Place Cone, Mobility, Place Cube Low",
        new PlaceConePlaceCubeLow(elevator, intake, leds, drive));

    autoChooser.addOption("Place Cone, Mobility, Place Cube Mid",
        new PlaceConePlaceCubeMid(elevator, intake, leds, drive));

    autoChooser.addOption("Place Cone, Mobility, Place Cube Mid, CABLE",
        new PlaceConePlaceCubeMidCable(elevator, intake, leds, drive));

    autoChooser.addOption("Place Cone, Mobility, Balance",
        new PlaceConeMobilityBalance(elevator, intake, leds, drive));

    autoChooser.addOption("Place Cone, Mobility, Grab Cone, Balance",
        new PlaceConeMobilityGrabConeBalance(elevator, intake, leds, drive));

    // autoChooser.addOption("(Substation Side) Place Cone, Drive Backwards",
    // placeConeAutoStart(new PathPlannerCommand("Substation Place Cone Drive
    // Backwards", driveSubsystem)));

    // autoChooser.addOption("(Substation Side) Place Cone, Grab Cone",
    // placeConeAutoStart(new PathPlannerCommand("Substation Place Cone Grab Cone",
    // driveSubsystem)));

    // autoChooser.addOption("(Substation Side) Place Cone, Grab Cone, Balance",
    // placeConeAutoStart(
    // new PathPlannerCommand("Substation Place Cone Grab Cone and Balance",
    // driveSubsystem)));

    // autoChooser.addOption("(Charge Station) Place Cone, Balance",
    // placeConeAutoStart(new PathPlannerCommand("Charge Station Place Cone
    // Balance", driveSubsystem))
    // .andThen(new AutoBalanceCommand(ledSubsystem, driveSubsystem)));

    // autoChooser.addOption("(Charge Station) Place Cone, Mobility, Balance",
    // placeConeAutoStart(new PathPlannerCommand("Charge Station Place Cone Mobility
    // Balance", driveSubsystem))
    // .andThen(new AutoBalanceCommand(ledSubsystem, driveSubsystem)));

    // autoChooser.addOption("(Field Edge) Place Cone, Drive Backwards",
    // placeConeAutoStart(new PathPlannerCommand("Field Edge Place Cone Drive
    // Backwards", driveSubsystem)));

    // autoChooser.addOption("(Field Edge) Place Cone, Grab Cone",
    // placeConeAutoStart(new PathPlannerCommand("Field Edge Place Cone Grab Cone",
    // driveSubsystem)));

    // autoChooser.addOption("(Field Edge) Place Cone, Grab Cone, Balance",
    // placeConeAutoStart(new PathPlannerCommand("Field Edge Place Cone Grab Cone
    // and Balance",
    // driveSubsystem)));

    SmartDashboard.putData("Auto Chooser", autoChooser);

    startingFieldPosition.setDefaultOption("Aligned w/ Charge Station", FieldPositionConstants.CHARGE_STATION);
    startingFieldPosition.addOption("Substation Side", FieldPositionConstants.SUBSTATION_SIDE);
    startingFieldPosition.addOption("Perimeter Side", FieldPositionConstants.FIELD_EDGE);

    SmartDashboard.putData("Starting Field Position", startingFieldPosition);

    // Configure the trigger bindings
    configureBindings();

    // driveSubsystem.setDefaultCommand(
    // new ArcadeDriveCommand(driverController::getLeftY,
    // driverController::getRightX, driveSubsystem));

    drive.setDefaultCommand(
        new TankDrive(driverController::getLeftY, driverController::getRightY, drive));
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
    Trigger turboDrive = driverController.rightTrigger();

    Trigger slowDrive = driverController.leftTrigger();

    Trigger turnToAngle = new Trigger(() -> driverControllerHID.getPOV() != -1);

    Trigger autoBalance = driverController.b();

    Trigger toggleBrake = new JoystickButton(operatorStick, Buttons.toggleBrakesButton);

    // Trigger alignToTape = driverController.rightBumper();

    // Trigger alignToAprilTag = driverController.leftBumper();

    Trigger turnToSubstation = driverController.rightBumper();
    Trigger turnToNode = driverController.leftBumper();

    Trigger setPipelineTape = driverController.start();
    Trigger setPipelineAprilTag = driverController.back();

    slowDrive.whileTrue(
        Commands.run(() -> drive.tankDrive(-driverController.getLeftY() * DriveConstants.slowSpeed,
            -driverController.getRightY() * DriveConstants.slowSpeed), drive));

    turboDrive
        .whileTrue(Commands
            .run(() -> drive.tankDrive(-driverController.getLeftY() * DriveConstants.turboSpeed,
                -driverController.getRightY() * DriveConstants.turboSpeed), drive));

    // Uses IEEEremainder to get the angle between -180 and 180
    turnToAngle
        .whileTrue(new AutoTurnToAngle(() -> driverControllerHID.getPOV(), drive));

    turnToSubstation.whileTrue(new TurnToAngle(0, drive));

    turnToNode.whileTrue(new TurnToAngle(180, drive));

    autoBalance.whileTrue(new AutoBalance(leds, drive));

    toggleBrake.toggleOnTrue(Commands.runOnce(drive::toggleBrakes, drive));

    // alignToTape.whileTrue(new DriveToTape(leds, drive));

    // alignToAprilTag.whileTrue(new AlignToAprilTag(drive));

    setPipelineTape
        .toggleOnTrue(
            Commands.runOnce(() -> LimelightHelpers.setPipelineIndex(DriveConstants.limelightName, 0),
                drive));

    setPipelineAprilTag
        .toggleOnTrue(
            Commands.runOnce(() -> LimelightHelpers.setPipelineIndex(DriveConstants.limelightName, 1),
                drive));
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
        new SetElevatorPosition(
            () -> (elevator.getPistonState() == Value.kForward) ? IntakeConstants.elevatorConeTopSetPoint
                : IntakeConstants.startingConfigurationHeight,
            elevator),
        Commands.runOnce(elevator::elevatorTiltIn, elevator), new WaitCommand(1.00),
        // new SetElevatorPosition(IntakeConstants.startingConfigurationHeight,
        // elevator),
        Commands.runOnce(intake::toggleWristOut, intake)));

    elevatorTilt.toggleOnTrue(
        Commands.runOnce(elevator::toggleElevatorTilt, elevator));

    coneElevatorHigh
        .whileTrue(new SetElevatorPosition(IntakeConstants.elevatorConeTopSetPoint, elevator));
    coneElevatorMid
        .whileTrue(new SetElevatorPosition(IntakeConstants.elevatorConeMidSetPoint, elevator));
    coneElevatorLow
        .whileTrue(new SetElevatorPosition(IntakeConstants.elevatorConeLowSetPoint, elevator));

    cubeElevatorHigh
        .whileTrue(new SetElevatorPosition(IntakeConstants.elevatorCubeTopSetPoint, elevator));
    cubeElevatorMid
        .whileTrue(new SetElevatorPosition(IntakeConstants.elevatorCubeMidSetPoint, elevator));
    cubeElevatorLow
        .whileTrue(new SetElevatorPosition(IntakeConstants.elevatorCubeLowSetPoint, elevator));

    elevatorUp.whileTrue(new ManualMoveElevator(-IntakeConstants.elevatorSpeed,
        () -> operatorStick.getRawButton(Buttons.elevatorLimitSwitchOverride), elevator));
    elevatorDown.whileTrue(new ManualMoveElevator(IntakeConstants.elevatorSpeed,
        () -> operatorStick.getRawButton(Buttons.elevatorLimitSwitchOverride), elevator));

    new JoystickButton(operatorStick, 15)
        .onTrue(Commands.runOnce(elevator::zeroElevatorPosition, elevator));
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

    intakeCone.whileTrue(Commands.run(intake::intakeCone, intake))
        .toggleOnFalse(Commands.runOnce(intake::stopIntakeMotor, intake));

    outtakeCone.whileTrue(Commands.run(intake::outTakeCone, intake))
        .toggleOnFalse(Commands.runOnce(intake::stopIntakeMotor, intake));

    intakeCube.whileTrue(Commands.run(intake::intakeCube, intake))
        .toggleOnFalse(Commands.runOnce(intake::stopIntakeMotor, intake));

    outtakeCube.whileTrue(Commands.run(intake::outTakeCube, intake))
        .toggleOnFalse(Commands.runOnce(intake::stopIntakeMotor, intake));

    intakePiston.toggleOnTrue(Commands.runOnce(intake::toggleIntakePiston, intake));
  }

  public static int getStartingFieldPosition() {
    return startingFieldPosition.getSelected();
  }

  public void initializeOdometry(Command autoCommand) {
    if (autoCommand instanceof FollowPathPlanner) {
      return;
    }

    drive.initializeFieldPosition(startingFieldPosition.getSelected());
  }

  public void initializeAutonomous() {
    drive.zeroGyro();
    intake.disableCompressor();
    leds.initializeAllianceColor();
  }

  public void initializeTeleop() {
    leds.initializeAllianceColor();
    intake.enableCompressor();
    drive.turnBrakesOff();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
