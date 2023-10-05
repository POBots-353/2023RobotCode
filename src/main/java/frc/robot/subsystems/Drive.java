// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//testing push

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldPositionConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightTarget_Fiducial;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;

public class Drive extends SubsystemBase {
  // Creates motors
  private CANSparkMax frontLeftMotor = new CANSparkMax(DriveConstants.frontLeftMotorID, MotorType.kBrushless);
  private CANSparkMax backLeftMotor = new CANSparkMax(DriveConstants.backLeftMotorID, MotorType.kBrushless);
  private CANSparkMax frontRightMotor = new CANSparkMax(DriveConstants.frontRightMotorID, MotorType.kBrushless);
  private CANSparkMax backRightMotor = new CANSparkMax(DriveConstants.backRightMotorID, MotorType.kBrushless);

  // Creates motor groups that control BOTH left/right motors
  private MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeftMotor, backLeftMotor);
  private MotorControllerGroup rightMotors = new MotorControllerGroup(frontRightMotor, backRightMotor);

  // Creates encoders that are used to get info about the robot (such as speed)
  private RelativeEncoder frontLeftEncoder = frontLeftMotor.getEncoder();
  private RelativeEncoder frontRightEncoder = frontRightMotor.getEncoder();
  private RelativeEncoder backLeftEncoder = backLeftMotor.getEncoder();
  private RelativeEncoder backRightEncoder = backRightMotor.getEncoder();

  // Creates PID controllers that use PID to control the motors
  private SparkMaxPIDController frontLeftPIDController = frontLeftMotor.getPIDController();
  private SparkMaxPIDController frontRightPIDController = frontRightMotor.getPIDController();
  private SparkMaxPIDController backLeftPIDController = backLeftMotor.getPIDController();
  private SparkMaxPIDController backRightPIDController = backRightMotor.getPIDController();

  /*
   * Creates SlewRateLimiters, which are used to dampen the effects of sudden
   * changes to the motors
   * (such as when the speed suddenly changes, it prevents the robot from going to
   * a sudden stop)
   */
  private SlewRateLimiter leftLimiter = new SlewRateLimiter(3.53);
  private SlewRateLimiter rightLimiter = new SlewRateLimiter(3.53);

  // private PhotonCamera limelight = new PhotonCamera("gloworm");

  // Creates Gyroscope
  private AHRS navx = new AHRS(SPI.Port.kMXP);

  // Creates brake pistion
  private DoubleSolenoid brakePiston = new DoubleSolenoid(IntakeConstants.pneumaticHubID, PneumaticsModuleType.REVPH,
      DriveConstants.pistonBrakeForwardID, DriveConstants.pistonBrakeReverseID);

  // Creates the initial variabes for PID, acceleration, speed, etc. for later use
  private int smartMotionSlot = 0;
  private int allowedErr;
  private int minVel;
  private double kP = 0.65; // 4.8e-4
  private double kI = 0;
  private double kD = 0;
  private double kIz = 0;
  private double kFF = 0.000176; // 0.000156
  private double kMaxOutput = 0.35;
  private double kMinOutput = -0.35;
  private double maxVel = 600; // 1750, 550
  private double maxAcc = 1500; // 2500, 1000

  /*
   * Creates PowerDistribution object, which is used for measuring power usage in
   * the robot,
   * as well as what is using the power
   */
  private PowerDistribution powerDistribution = new PowerDistribution(DriveConstants.powerDistributionID,
      ModuleType.kRev);

  // Creates field object, which is used to display the robot's position on the
  // field.
  private Field2d field = new Field2d();

  // Creates odometry object, which is used to track the position of a robot using
  // encoders and sensors
  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(navx.getRotation2d(), 0, 0,
      AutoConstants.blueSubstationPose);

  private final Vector<N3> smallDistanceDeviation = VecBuilder.fill(1.00, 1.00, Units.degreesToRadians(30));
  private final Vector<N3> largeDistanceDeviation = VecBuilder.fill(2.50, 2.50, Units.degreesToRadians(45));

  private DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(
      DriveConstants.driveKinematics, navx.getRotation2d(), frontLeftEncoder.getPosition(),
      frontRightEncoder.getPosition(), AutoConstants.blueChargeStationPose,
      VecBuilder.fill(0.1, 0.1, 0.1), smallDistanceDeviation);

  private Pose3d lastVisionPose;
  private int lastDetectedID = -1;

  /** Creates a new DriveSubsystem. */
  public Drive() {
    // Resets motor to factory defaults, essentially reseting the motor until
    // controlled.
    frontLeftMotor.restoreFactoryDefaults();
    frontRightMotor.restoreFactoryDefaults();
    backLeftMotor.restoreFactoryDefaults();
    backRightMotor.restoreFactoryDefaults();

    // Sets the conversion factor for the position of the encoder.
    frontLeftEncoder.setPositionConversionFactor(DriveConstants.encoderToDistanceRatio);
    frontRightEncoder.setPositionConversionFactor(DriveConstants.encoderToDistanceRatio);
    backLeftEncoder.setPositionConversionFactor(DriveConstants.encoderToDistanceRatio);
    backRightEncoder.setPositionConversionFactor(DriveConstants.encoderToDistanceRatio);

    // Sets the conversion factor for the velocity of the encoder
    frontLeftEncoder.setVelocityConversionFactor(DriveConstants.encoderToDistanceRatio);
    frontRightEncoder.setVelocityConversionFactor(DriveConstants.encoderToDistanceRatio);
    backLeftEncoder.setVelocityConversionFactor(DriveConstants.encoderToDistanceRatio);
    backRightEncoder.setVelocityConversionFactor(DriveConstants.encoderToDistanceRatio);

    // Sets the phase of the AbsoluteEndoder so that it is set to be in phase witht
    // he motor itself
    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);

    /*
     * Causes the back motors to imitate the output of the leader motors (front
     * motors) so that the back and
     * front motors are in sync
     */
    backLeftMotor.follow(frontLeftMotor);
    backRightMotor.follow(frontRightMotor);

    // Initializes the PID contorllers for each of the motors
    initializePID(frontLeftPIDController);
    initializePID(frontRightPIDController);
    initializePID(backLeftPIDController);
    initializePID(backRightPIDController);

    navx.setAngleAdjustment(180);

    // Sets the brakepistion position to its retracted state
    brakePiston.set(Value.kReverse);

    // Put the gyro on the dashboard
    SmartDashboard.putData(navx);

    // Clear sticky faults
    powerDistribution.clearStickyFaults();

    // Clears sticky faults
    frontLeftMotor.clearFaults();
    frontRightMotor.clearFaults();
    backLeftMotor.clearFaults();
    backRightMotor.clearFaults();

    // Creates Sendable class
    Sendable differentialDriveSendable = new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("DifferentialDrive");
        builder.setActuator(false);
        builder.addDoubleProperty("Left Motor Speed", leftMotors::get, leftMotors::set);
        builder.addDoubleProperty("Right Motor Speed", rightMotors::get, rightMotors::set);
      }
    };

    // Sends data about Drive train, field, and powerdistribution to smartdashboard
    SmartDashboard.putData("Drive Train", differentialDriveSendable);

    SmartDashboard.putData(field);

    SmartDashboard.putData(powerDistribution);
  }

  private void initializePID(SparkMaxPIDController p) {
    p.setP(kP); // Sets the Proportional Gain constat of the PIDF contorller
    p.setI(kI); // Sets the Integral Gain constant of the PIDF contoller
    p.setD(kD); // Sets the Derivative Gain constant of the PIDF controller
    p.setIZone(kIz); // Sets the IZone range
    p.setFF(kFF); // Sets the Feed-forward Gain constant of the PIDF controller
    /*
     * Feed-forward is a prediction technique that estimates the output from a PID
     * controller without waiting
     * for the PID algorithm to respond. They are useful for reducing the error
     * fastor or keeps the error smaller
     * rather than making the PID algorithm do it by itself
     */
    p.setOutputRange(kMinOutput, kMaxOutput); /* Sets the minimum and maximum output of the PID controller */
    p.setSmartMotionMaxVelocity(maxVel, smartMotionSlot); /* Sets the maximum velocity of the SmartMotion mode */
    p.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot); /* Sets the minimum velocity of the Smartmotion mode */
    p.setSmartMotionMaxAccel(maxAcc, smartMotionSlot); /* Sets the maximum acceleration of the SmartMotion mode */
    p.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot); /* Sets the allowed error for the motor */
  }

  public void setMaxVelocity(int maxVelocity) {
    frontLeftPIDController.setSmartMotionMaxVelocity(maxVelocity, smartMotionSlot);
    frontRightPIDController.setSmartMotionMaxVelocity(maxVelocity, smartMotionSlot);
    backLeftPIDController.setSmartMotionMaxVelocity(maxVelocity, smartMotionSlot);
    backRightPIDController.setSmartMotionMaxVelocity(maxVelocity, smartMotionSlot);
  }

  public void setMaxOutput(double maxOutput) {
    frontLeftPIDController.setOutputRange(-maxOutput, maxOutput);
    frontRightPIDController.setOutputRange(-maxOutput, maxOutput);
    backLeftPIDController.setOutputRange(-maxOutput, maxOutput);
    backRightPIDController.setOutputRange(-maxOutput, maxOutput);
  }

  public void tankDrive(double left, double right) {
    double leftSpeed = leftLimiter.calculate(left);
    double rightSpeed = rightLimiter.calculate(right);

    if (Math.abs(leftSpeed) < 0.06) {
      leftSpeed = 0;
      leftLimiter.reset(0);
    }

    if (Math.abs(rightSpeed) < 0.06) {
      rightSpeed = 0;
      rightLimiter.reset(0);
    }

    frontLeftMotor.set(leftSpeed);
    frontRightMotor.set(rightSpeed);

    // backLeftMotor.set(leftSpeed);
    // backRightMotor.set(-rightSpeed);
  }

  public void arcadeDrive(double forward, double turn) {
    // if (Math.abs(forward) < 0.06) {
    // forward = 0;
    // }

    // if (Math.abs(turn) < 0.06) {
    // turn = 0;
    // }

    double leftSpeed = forward + turn;

    double rightSpeed = forward - turn;

    frontLeftMotor.set(leftSpeed);
    frontRightMotor.set(rightSpeed);

    // backLeftMotor.set(leftSpeed);
    // backRightMotor.set(-rightSpeed);
    // leftMotors.set(leftSpeed);
    // rightMotors.set(-rightSpeed);
  }

  public void autoDrive(double meters) {
    frontLeftPIDController.setReference(meters, ControlType.kPosition);
    frontRightPIDController.setReference(meters, ControlType.kPosition);
    // frontLeftPIDController.setReference(meters, ControlType.kSmartMotion);
    // frontRightPIDController.setReference(meters, ControlType.kSmartMotion);

    // backLeftPIDController.setReference(meters, ControlType.kSmartMotion);
    // backRightPIDController.setReference(-meters, ControlType.kSmartMotion);
    // leftPIDController.setReference(convertDistanceToEncoder(meters),
    // ControlType.kSmartMotion);
    // rightPIDController.setReference(-convertDistanceToEncoder(meters),
    // ControlType.kSmartMotion);
  }

  public void toggleBrakes() {
    brakePiston.toggle();
    // Breakes pistion on toggle of button
  }

  public void turnBrakesOn() {
    brakePiston.set(Value.kForward);
  }

  public void turnBrakesOff() {
    brakePiston.set(Value.kReverse);
  }

  public void resetEncoders() {
    Pose2d lastPose = poseEstimator.getEstimatedPosition();

    frontLeftEncoder.setPosition(0);
    frontRightEncoder.setPosition(0);
    backLeftEncoder.setPosition(0);
    backRightEncoder.setPosition(0);

    poseEstimator.resetPosition(navx.getRotation2d(), 0, 0, lastPose);
  }

  public boolean alignedToTapeYaw() {
    if (LimelightHelpers.getTV(DriveConstants.limelightName)) {
      return Math
          .abs(LimelightHelpers.getTX(DriveConstants.limelightName)) <= DriveConstants.tapeAlignmentPitchTolerance;
    }

    return false;
    // PhotonPipelineResult result = limelight.getLatestResult();

    // if (result.hasTargets()) {
    // PhotonTrackedTarget target = result.getBestTarget();

    // return Math.abs(target.getYaw()) <= DriveConstants.tapeAlignmentTolerance;
    // }

    // return false;
  }

  public boolean alignedToTapeArea() {
    if (LimelightHelpers.getTV(DriveConstants.limelightName)) {
      return Math
          .abs(LimelightHelpers.getTA(DriveConstants.limelightName)
              - DriveConstants.tapeAlignmentArea) <= DriveConstants.tapeAlignmentAreaTolerance;
    }

    return false;
  }

  public boolean distanceReached(double distanceMeters) {
    if (distanceMeters < 0) {
      return Math.abs(frontLeftEncoder.getPosition() - distanceMeters) <= 0.060
          || frontLeftEncoder.getPosition() < distanceMeters;
    }
    // else if (distanceMeters > 0) {
    // return Math.abs(frontLeftEncoder.getPosition() - distanceMeters) <= 0.060
    // || frontLeftEncoder.getPosition() < distanceMeters;
    // }

    return Math.abs(frontLeftEncoder.getPosition() - distanceMeters) <= 0.060;
    // return Math.abs(frontLeftEncoder.getPosition() -
    // convertDistanceToEncoder(distanceMeters)) <= 0.10;
  }

  public double convertDistanceToEncoder(double meters) {
    return meters * DriveConstants.distanceToEncoderRatio;
  }

  public double convertEncoderToDistance(double encoder) {
    return encoder * DriveConstants.encoderToDistanceRatio;
  }

  public void initializeFieldPosition(int position) {
    switch (DriverStation.getAlliance()) {
      case Red:
        initializeRedFieldPosition(position);
        break;
      case Blue:
        initializeBlueFieldPosition(position);
        break;
      default:
        break;
    }
  }

  public void initializeRedFieldPosition(int position) {
    Pose2d pose;

    switch (position) {
      case FieldPositionConstants.SUBSTATION_SIDE: // Substation
        pose = AutoConstants.redSubstationPose;
        break;
      case FieldPositionConstants.CHARGE_STATION: // Charge Station
        pose = AutoConstants.redChargeStationPose;
        break;
      case FieldPositionConstants.FIELD_EDGE: // Field Edge
        pose = AutoConstants.redFieldEdgePose;
        break;
      default:
        pose = AutoConstants.redSubstationPose;
        break;
    }

    odometry.resetPosition(navx.getRotation2d(), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition(), pose);
  }

  public void initializeBlueFieldPosition(int position) {
    Pose2d pose;

    switch (position) {
      case FieldPositionConstants.SUBSTATION_SIDE: // Substation
        pose = AutoConstants.blueSubstationPose;
        break;
      case FieldPositionConstants.CHARGE_STATION: // Charge Station
        pose = AutoConstants.blueChargeStationPose;
        break;
      case FieldPositionConstants.FIELD_EDGE: // Field Edge
        pose = AutoConstants.blueFieldEdgePose;
        break;
      default:
        pose = AutoConstants.blueSubstationPose;
        break;
    }

    odometry.resetPosition(navx.getRotation2d(), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition(), pose);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void zeroGyro() {
    navx.zeroYaw();
  }

  public double getGyroYaw() {
    return Math.IEEEremainder(navx.getYaw() - 180, 360);
  }

  public double getGyroPitch() {
    return Math.IEEEremainder(navx.getPitch(), 360);
  }

  public double getAngleError(double expectedAngle) {
    double angleSubtract = Math.IEEEremainder(expectedAngle, 360) - Math.IEEEremainder(navx.getYaw() - 180, 360);

    // if (angleSubtract < -180) {
    // return angleSubtract + 360;
    // } else if (angleSubtract > 180) {
    // return angleSubtract - 360;
    // }

    return MathUtil.inputModulus(angleSubtract, -180, 180);

    // return angleSubtract;
  }

  public Rotation2d getRotation() {
    return navx.getRotation2d();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(frontLeftEncoder.getVelocity() / 60, frontRightEncoder.getVelocity() / 60);
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        navx.getRotation2d(), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition(), pose);
  }

  public void resetOdometry(Pose2d pose, Trajectory trajectory) {
    odometry.resetPosition(
        navx.getRotation2d(), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition(), pose);

    field.getRobotObject().setTrajectory(trajectory);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    frontLeftMotor.setVoltage(leftVolts);
    frontRightMotor.setVoltage(rightVolts);
  }

  public boolean validTarget(int aprilTagID, Pose3d targetPose) {
    if (aprilTagID < 1 || aprilTagID > 8) {
      return false;
    }

    double distance = Math.abs(targetPose.getZ());
    double angle = MathUtil.inputModulus(Math.toDegrees(targetPose.getRotation().getY()), -90.0, 90.0);

    if (distance < 0.30 || distance > 3.0) {
      return false;
    }

    if (Math.abs(angle) > 80) {
      return false;
    }

    if (lastVisionPose != null && lastDetectedID == aprilTagID) {
      double previousRotation = lastVisionPose.getRotation().getY();
      double currentRotation = targetPose.getRotation().getY();

      if (Math.abs(previousRotation - currentRotation) > 30) {
        return false;
      }
    }

    return true;
  }

  public void updateVisionPoseEstimates() {
    LimelightHelpers.Results results = LimelightHelpers.getLatestResults(DriveConstants.limelightName).targetingResults;

    if (!results.valid || results.targets_Fiducials.length == 0) {
      clearDetectedTargets();
      return;
    }

    Pose2d robotPose = results.getBotPose2d_wpiBlue();

    Pose3d closestTargetPose = results.targets_Fiducials[0].getTargetPose_CameraSpace();

    int closestTagID = (int) results.targets_Fiducials[0].fiducialID;

    List<Pose2d> detectedTargets = new ArrayList<>();

    for (LimelightTarget_Fiducial target : results.targets_Fiducials) {
      int targetID = (int) (target.fiducialID);
      if (targetID < 1 || targetID > 8) {
        continue;
      }

      if (target.getTargetPose_CameraSpace().getZ() < closestTargetPose.getZ()) {
        closestTargetPose = target.getTargetPose_CameraSpace();
        closestTagID = targetID;
      }

      detectedTargets.add(FieldPositionConstants.aprilTags.get(targetID).toPose2d());
    }

    boolean validTarget = validTarget(closestTagID, closestTargetPose);

    lastVisionPose = closestTargetPose;
    if (closestTagID != lastDetectedID) {
      lastVisionPose = null;
    }
    lastDetectedID = closestTagID;

    if (!validTarget) {
      clearDetectedTargets();
      return;
    }

    double distance = Math.abs(closestTargetPose.getZ());
    double latency = results.timestamp_LIMELIGHT_publish
        - (results.latency_capture + results.latency_pipeline + results.latency_jsonParse) / 1000.0;

    if (distance < 2.0 || detectedTargets.size() > 1) {
      poseEstimator.addVisionMeasurement(robotPose, latency, smallDistanceDeviation);
    } else {
      poseEstimator.addVisionMeasurement(robotPose, latency, largeDistanceDeviation);
    }

    field.getObject("Detected Target").setPoses(detectedTargets);
  }

  public void clearDetectedTargets() {
    field.getObject("Detected Target").setPoses(List.of());
  }

  public void updateOdometry() {
    odometry.update(navx.getRotation2d(), frontLeftEncoder.getPosition(),
        frontRightEncoder.getPosition());

    poseEstimator.update(navx.getRotation2d(), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition());

    updateVisionPoseEstimates();

    field.setRobotPose(poseEstimator.getEstimatedPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();

    if (DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() <= 0.40 && DriverStation.getMatchTime() > 0
        && Math.abs(getGyroPitch()) > 3.0) {
      turnBrakesOn();
    }

    SmartDashboard.putNumber("Gyro Yaw", Math.IEEEremainder(navx.getYaw(), 360));
    SmartDashboard.putNumber("Gyro Pitch", Math.IEEEremainder(navx.getPitch(), 360));
    SmartDashboard.putNumber("Gyro Roll", Math.IEEEremainder(navx.getRoll(), 360));

    SmartDashboard.putNumber("Left Velocity", frontLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Velocity", frontRightEncoder.getVelocity());

    SmartDashboard.putNumber("Front Left Position", frontLeftEncoder.getPosition());
    SmartDashboard.putNumber("Back Left Position", backLeftEncoder.getPosition());
    SmartDashboard.putNumber("Front Right Position", backLeftEncoder.getPosition());
    SmartDashboard.putNumber("Back Right Position", backLeftEncoder.getPosition());

    SmartDashboard.putNumber("Left Motor Temperature", frontLeftMotor.getMotorTemperature());
    SmartDashboard.putNumber("Right Motor Temperature", frontRightMotor.getMotorTemperature());

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
  }
}
