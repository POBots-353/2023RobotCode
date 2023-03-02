// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//testing push

package frc.robot.subsystems;

import java.security.spec.EncodedKeySpec;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.drive.DriveToTapeCommand;
import frc.robot.util.LimelightHelpers;
import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;

public class DriveSubsystem extends SubsystemBase {
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

  private SparkMaxPIDController leftPIDController = frontLeftMotor.getPIDController();
  private SparkMaxPIDController rightPIDController = frontRightMotor.getPIDController();

  private SlewRateLimiter leftLimiter = new SlewRateLimiter(3.53);
  private SlewRateLimiter rightLimiter = new SlewRateLimiter(3.53);

  // private PhotonCamera limelight = new PhotonCamera("gloworm");

  private AHRS navx = new AHRS(SPI.Port.kMXP);

  // private Ultrasonic coneUltrasonic = new Ultrasonic(1, 2);

  // private Ultrasonic cubeUltrasonic = new Ultrasonic(3, 4);

  private DoubleSolenoid brakePiston = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      DriveConstants.pistonBrakeForwardID, DriveConstants.pistonBrakeReverseID);

  private PIDController balancePIDController = new PIDController(0.010, 0, 0.00125);

  private int smartMotionSlot = 0;
  private int allowedErr;
  private int minVel;
  private double kP = 4e-4;
  private double kI = 0;
  private double kD = 0;
  private double kIz = 0;
  private double kFF = 0.000146;
  private double kMaxOutput = 1;
  private double kMinOutput = -1;
  private double maxVel = 1750;
  private double maxAcc = 2500;

  private PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);

  private Field2d field = new Field2d();

  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(navx.getRotation2d(), 0, 0,
      new Pose2d(2, 4.3, navx.getRotation2d()));

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    frontLeftMotor.restoreFactoryDefaults();
    frontRightMotor.restoreFactoryDefaults();
    backLeftMotor.restoreFactoryDefaults();
    backRightMotor.restoreFactoryDefaults();

    frontLeftEncoder.setPositionConversionFactor(DriveConstants.encoderToDistanceRatio);
    frontRightEncoder.setPositionConversionFactor(DriveConstants.encoderToDistanceRatio);

    frontLeftEncoder.setVelocityConversionFactor(DriveConstants.encoderToDistanceRatio);
    frontRightEncoder.setVelocityConversionFactor(DriveConstants.encoderToDistanceRatio);
    backLeftEncoder.setVelocityConversionFactor(DriveConstants.encoderToDistanceRatio);
    backRightEncoder.setVelocityConversionFactor(DriveConstants.encoderToDistanceRatio);

    backLeftMotor.follow(frontLeftMotor);
    backRightMotor.follow(frontRightMotor);

    initializePID(leftPIDController);
    initializePID(rightPIDController);

    brakePiston.set(Value.kReverse);

    // Put the gyro on the dashboard
    SmartDashboard.putData(navx);

    // Clear sticky faults
    powerDistribution.clearStickyFaults();

    SmartDashboard.putData(field);

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
    frontRightMotor.set(-rightSpeed);
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

    leftMotors.set(leftSpeed);
    rightMotors.set(-rightSpeed);
  }

  public void autoDrive(double meters) {
    leftPIDController.setReference(meters, ControlType.kSmartMotion);
    rightPIDController.setReference(-meters, ControlType.kSmartMotion);
    // leftPIDController.setReference(convertDistanceToEncoder(meters),
    // ControlType.kSmartMotion);
    // rightPIDController.setReference(-convertDistanceToEncoder(meters),
    // ControlType.kSmartMotion);
  }

  public void toggleBrakes() {
    brakePiston.toggle();
    // Breakes pistion on toggle of button
  }

  public void resetEncoders() {
    frontLeftEncoder.setPosition(0);
    frontRightEncoder.setPosition(0);
  }

  public void autoBalance() {
    double gyroPitch = navx.getPitch();

    if (Math.abs(gyroPitch) <= 0.5) {
      arcadeDrive(0, 0);
      SmartDashboard.putBoolean("Balanced", true);
      // Stops Robot
      return;
    }

    if (Math.abs(gyroPitch) < 5.5) {
      if (Math.abs(gyroPitch) < 2.5) {
        balancePIDController.setP(0.0085);
        SmartDashboard.putBoolean("Balanced", false);
        // Adjusts PID values
      } else {
        balancePIDController.setP(0.0061);
        SmartDashboard.putBoolean("Balanced", false);
        // Adjusts PID values
      }
    }

    double forwardSpeed = balancePIDController.calculate(gyroPitch, 0);

    arcadeDrive(forwardSpeed, 0);
    SmartDashboard.putBoolean("Balanced", false);
  }

  public void resetBalance() {
    balancePIDController.setP(0.010);
  }

  public double getGyroYaw() {
    return Math.IEEEremainder(navx.getYaw(), 360);
  }

  public double getGyroPitch() {
    return Math.IEEEremainder(navx.getPitch(), 360);
  }

  public double getAngleError(double expectedAngle) {
    double angleSubtract = Math.IEEEremainder(expectedAngle, 360) - Math.IEEEremainder(navx.getYaw(), 360);

    if (angleSubtract < -180) {
      return angleSubtract + 360;
    } else if (angleSubtract > 180) {
      return angleSubtract - 360;
    }

    return angleSubtract;
  }

  public boolean alignedToTapeYaw() {
    if (LimelightHelpers.getTV(DriveConstants.limelightName)) {
      return Math.abs(LimelightHelpers.getTX(DriveConstants.limelightName)) <= DriveConstants.tapeAlignmentTolerance;
    }

    return false;
    // PhotonPipelineResult result = limelight.getLatestResult();

    // if (result.hasTargets()) {
    // PhotonTrackedTarget target = result.getBestTarget();

    // return Math.abs(target.getYaw()) <= DriveConstants.tapeAlignmentTolerance;
    // }

    // return false;
  }

  public boolean alignedToTapePitch() {
    if (LimelightHelpers.getTV(DriveConstants.limelightName)) {
      return Math
          .abs(LimelightHelpers.getTY(DriveConstants.limelightName)
              - DriveConstants.tapeAlignmentPitch) <= DriveConstants.tapeAlignmentTolerance;
    }

    return false;
  }

  public double getConeDistanceFromCenter() {
    // return coneUltrasonic.getRangeMM() - DriveConstants.coneCenterMM;
    return 0;
  }

  public double getCubeDistanceFromCenter() {
    // return cubeUltrasonic.getRangeMM() - DriveConstants.cubeCenterMM;
    return 0;
  }

  public boolean distanceReached(double distanceMeters) {
    return Math.abs(frontLeftEncoder.getPosition() - distanceMeters) <= 0.006;
    // return Math.abs(frontLeftEncoder.getPosition() -
    // convertDistanceToEncoder(distanceMeters)) <= 0.10;
  }

  public double convertDistanceToEncoder(double meters) {
    return meters * DriveConstants.distanceToEncoderRatio;
    // return 2 * (meters / DriveConstants.wheelCircumference) * 42 /
    // DriveConstants.gearBoxRatio;
  }

  public double convertEncoderToDistance(double encoder) {
    return encoder * DriveConstants.encoderToDistanceRatio;
    // return 0.5 * DriveConstants.gearBoxRatio * DriveConstants.wheelCircumference
    // * encoder / 42;
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
    double startX = 15.513558 - 0.9652;
    double startY = 0;

    switch (position) {
      case 1:
        startY = 4.424;
        break;
      case 2:
        startY = 2.748;
        break;
      case 3:
        startY = 1.07;
        break;
      default:
        break;
    }

    odometry.resetPosition(navx.getRotation2d(), convertEncoderToDistance(frontLeftEncoder.getPosition()),
        convertEncoderToDistance(frontRightEncoder.getPosition()),
        new Pose2d(startX, startY, navx.getRotation2d()));
  }

  public void initializeBlueFieldPosition(int position) {
    double startX = 1.02743 + 0.9652;
    double startY = 0;

    switch (position) {
      case 1:
        startY = 4.424;
        break;
      case 2:
        startY = 2.748;
        break;
      case 3:
        startY = 1.07;
        break;
      default:
        break;
    }

    odometry.resetPosition(navx.getRotation2d(), convertEncoderToDistance(frontLeftEncoder.getPosition()),
        convertEncoderToDistance(frontRightEncoder.getPosition()),
        new Pose2d(startX, startY, navx.getRotation2d()));
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public Rotation2d getRotation() {
    return navx.getRotation2d();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(frontLeftEncoder.getVelocity(), frontRightEncoder.getVelocity());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(
        navx.getRotation2d(), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition(), pose);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(-rightVolts);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(navx.getRotation2d(), frontLeftEncoder.getPosition(),
        -frontRightEncoder.getPosition());
  
    field.setRobotPose(odometry.getPoseMeters());
    // SmartDashboard.putNumber("Ultrasonic Distance",
    // coneUltrasonic.getRangeInches());
    // SmartDashboard.putBoolean("Ultrasonic Valid", coneUltrasonic.isRangeValid());

    SmartDashboard.putNumber("Gyro Yaw", Math.IEEEremainder(navx.getYaw(), 360));
    SmartDashboard.putNumber("Gyro Pitch", Math.IEEEremainder(navx.getPitch(), 360));
    SmartDashboard.putNumber("Gyro Roll", Math.IEEEremainder(navx.getRoll(), 360));

    SmartDashboard.putNumber("Front Velocity", frontLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Back Velocity", backLeftEncoder.getVelocity());

    SmartDashboard.putNumber("Front Left Current", powerDistribution.getCurrent(10));
    SmartDashboard.putNumber("Front Right Current", powerDistribution.getCurrent(9));

    SmartDashboard.putNumber("Back Left Current", powerDistribution.getCurrent(14));
    SmartDashboard.putNumber("Back Right Current", powerDistribution.getCurrent(5));

    // SmartDashboard.putNumber("Voltage", powerDistribution.getVoltage());
  }
}
