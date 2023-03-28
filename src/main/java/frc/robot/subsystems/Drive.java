// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//testing push

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.LimelightHelpers;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;

public class Drive extends SubsystemBase {
  private CANSparkMax frontLeftMotor = new CANSparkMax(DriveConstants.frontLeftMotorID, MotorType.kBrushless);
  private CANSparkMax backLeftMotor = new CANSparkMax(DriveConstants.backLeftMotorID, MotorType.kBrushless);
  private CANSparkMax frontRightMotor = new CANSparkMax(DriveConstants.frontRightMotorID, MotorType.kBrushless);
  private CANSparkMax backRightMotor = new CANSparkMax(DriveConstants.backRightMotorID, MotorType.kBrushless);

  private MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeftMotor, backLeftMotor);
  private MotorControllerGroup rightMotors = new MotorControllerGroup(frontRightMotor, backRightMotor);

  private RelativeEncoder frontLeftEncoder = frontLeftMotor.getEncoder();
  private RelativeEncoder frontRightEncoder = frontRightMotor.getEncoder();
  private RelativeEncoder backLeftEncoder = backLeftMotor.getEncoder();
  private RelativeEncoder backRightEncoder = backRightMotor.getEncoder();

  private SparkMaxPIDController frontLeftPIDController = frontLeftMotor.getPIDController();
  private SparkMaxPIDController frontRightPIDController = frontRightMotor.getPIDController();
  private SparkMaxPIDController backLeftPIDController = backLeftMotor.getPIDController();
  private SparkMaxPIDController backRightPIDController = backRightMotor.getPIDController();

  private SlewRateLimiter leftLimiter = new SlewRateLimiter(3.53);
  private SlewRateLimiter rightLimiter = new SlewRateLimiter(3.53);

  // private PhotonCamera limelight = new PhotonCamera("gloworm");

  private AHRS navx = new AHRS(SPI.Port.kMXP);

  private DoubleSolenoid brakePiston = new DoubleSolenoid(IntakeConstants.pneumaticHubID, PneumaticsModuleType.REVPH,
      DriveConstants.pistonBrakeForwardID, DriveConstants.pistonBrakeReverseID);

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

  private PowerDistribution powerDistribution = new PowerDistribution(DriveConstants.powerDistributionID,
      ModuleType.kRev);

  private Field2d field = new Field2d();

  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(navx.getRotation2d(), 0, 0,
      AutoConstants.blueSubstationPose);

  /** Creates a new DriveSubsystem. */
  public Drive() {
    frontLeftMotor.restoreFactoryDefaults();
    frontRightMotor.restoreFactoryDefaults();
    backLeftMotor.restoreFactoryDefaults();
    backRightMotor.restoreFactoryDefaults();

    frontLeftEncoder.setPositionConversionFactor(DriveConstants.encoderToDistanceRatio);
    frontRightEncoder.setPositionConversionFactor(DriveConstants.encoderToDistanceRatio);
    backLeftEncoder.setPositionConversionFactor(DriveConstants.encoderToDistanceRatio);
    backRightEncoder.setPositionConversionFactor(DriveConstants.encoderToDistanceRatio);

    frontLeftEncoder.setVelocityConversionFactor(DriveConstants.encoderToDistanceRatio);
    frontRightEncoder.setVelocityConversionFactor(DriveConstants.encoderToDistanceRatio);
    backLeftEncoder.setVelocityConversionFactor(DriveConstants.encoderToDistanceRatio);
    backRightEncoder.setVelocityConversionFactor(DriveConstants.encoderToDistanceRatio);

    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);

    backLeftMotor.follow(frontLeftMotor);
    backRightMotor.follow(frontRightMotor);

    initializePID(frontLeftPIDController);
    initializePID(frontRightPIDController);
    initializePID(backLeftPIDController);
    initializePID(backRightPIDController);

    navx.setAngleAdjustment(180);

    brakePiston.set(Value.kReverse);

    // Put the gyro on the dashboard
    SmartDashboard.putData(navx);

    // Clear sticky faults
    powerDistribution.clearStickyFaults();

    frontLeftMotor.clearFaults();
    frontRightMotor.clearFaults();
    backLeftMotor.clearFaults();
    backRightMotor.clearFaults();

    Sendable differentialDriveSendable = new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("DifferentialDrive");
        builder.setActuator(false);
        builder.addDoubleProperty("Left Motor Speed", leftMotors::get, leftMotors::set);
        builder.addDoubleProperty("Right Motor Speed", rightMotors::get, rightMotors::set);
      }
    };

    SmartDashboard.putData("Drive Train", differentialDriveSendable);

    SmartDashboard.putData(field);

    SmartDashboard.putData(powerDistribution);
  }

  private void initializePID(SparkMaxPIDController p) {
    p.setP(kP);
    p.setI(kI);
    p.setD(kD);
    p.setIZone(kIz);
    p.setFF(kFF);
    p.setOutputRange(kMinOutput, kMaxOutput);
    p.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    p.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    p.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    p.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
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
    Pose2d lastPose = odometry.getPoseMeters();

    frontLeftEncoder.setPosition(0);
    frontRightEncoder.setPosition(0);
    backLeftEncoder.setPosition(0);
    backRightEncoder.setPosition(0);

    odometry.resetPosition(navx.getRotation2d(), 0, 0, lastPose);
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
      case 1: // Substation
        pose = AutoConstants.redSubstationPose;
        break;
      case 2: // Charge Station
        pose = AutoConstants.redChargeStationPose;
        break;
      case 3: // Field Edge
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
      case 1: // Substation
        pose = AutoConstants.blueSubstationPose;
        break;
      case 2: // Charge Station
        pose = AutoConstants.blueChargeStationPose;
        break;
      case 3: // Field Edge
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

    if (angleSubtract < -180) {
      return angleSubtract + 360;
    } else if (angleSubtract > 180) {
      return angleSubtract - 360;
    }

    return angleSubtract;
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

  public void updateOdometry() {
    odometry.update(navx.getRotation2d(), frontLeftEncoder.getPosition(),
        frontRightEncoder.getPosition());

    field.setRobotPose(odometry.getPoseMeters());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();

    SmartDashboard.putNumber("Gyro Yaw", Math.IEEEremainder(navx.getYaw(), 360));
    SmartDashboard.putNumber("Gyro Pitch", Math.IEEEremainder(navx.getPitch(), 360));
    SmartDashboard.putNumber("Gyro Roll", Math.IEEEremainder(navx.getRoll(), 360));

    SmartDashboard.putNumber("Front Velocity", frontLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Back Velocity", backLeftEncoder.getVelocity());

    SmartDashboard.putNumber("Front Left Position", frontLeftEncoder.getPosition());
    SmartDashboard.putNumber("Back Left Position", backLeftEncoder.getPosition());
    SmartDashboard.putNumber("Front Right Position", backLeftEncoder.getPosition());
    SmartDashboard.putNumber("Back Right Position", backLeftEncoder.getPosition());
    // SmartDashboard.putNumber("Voltage", powerDistribution.getVoltage());
  }
}
