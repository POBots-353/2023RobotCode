// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private CANSparkMax frontLeftMotor = new CANSparkMax(DriveConstants.frontLeftMotorID, MotorType.kBrushless);
  private CANSparkMax backLeftMotor = new CANSparkMax(DriveConstants.backLeftMotorID, MotorType.kBrushless);
  private CANSparkMax frontRightMotor = new CANSparkMax(DriveConstants.frontRightMotorID, MotorType.kBrushless);
  private CANSparkMax backRightMotor = new CANSparkMax(DriveConstants.backRightMotorID, MotorType.kBrushless);

  private MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeftMotor, backLeftMotor);
  private MotorControllerGroup rightMotors = new MotorControllerGroup(frontRightMotor, backRightMotor);

  private RelativeEncoder frontLeftEncoder = frontLeftMotor.getEncoder();
  private RelativeEncoder frontRightEncoder = frontRightMotor.getEncoder();

  private SparkMaxPIDController frontLeftPIDController = frontLeftMotor.getPIDController();
  private SparkMaxPIDController backLeftPIDController = backLeftMotor.getPIDController();
  private SparkMaxPIDController frontRightPIDController = frontRightMotor.getPIDController();

  private DifferentialDrive differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

  private SlewRateLimiter leftLimiter = new SlewRateLimiter(3.53);
  private SlewRateLimiter rightLimiter = new SlewRateLimiter(3.53);

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
  private double maxVel = 5000;
  private double maxAcc = 2500;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    rightMotors.setInverted(true);

    // backLeftMotor.follow(frontLeftMotor);
    // backRightMotor.follow(frontRightMotor);

    initializePID(frontLeftPIDController);
    initializePID(backLeftPIDController);
    initializePID(frontRightPIDController);
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

    if (Math.abs(left) < 0.06 && Math.abs(right) < 0.06) {
      leftSpeed = 0;
      rightSpeed = 0;

      leftLimiter.reset(0);
      rightLimiter.reset(0);
    }

    differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void autoDrive(double meters) {
    SmartDashboard.putNumber("Desired Encoder Distance", convertDistanceToEncoder(meters));
    // leftPIDController.setReference(convertDistanceToEncoder(meters),
    // ControlType.kSmartMotion);
    // rightPIDController.setReference(-convertDistanceToEncoder(meters),
    // ControlType.kSmartMotion);
    frontLeftPIDController.setReference(20, ControlType.kSmartMotion);
    backLeftPIDController.setReference(20, ControlType.kSmartMotion);
    // rightPIDController.setReference(-500, ControlType.kSmartMotion);
  }

  public void resetEncoders() {
    frontLeftEncoder.setPosition(0);
    frontRightEncoder.setPosition(0);
  }

  public boolean distanceReached(double distanceMeters) {
    return Math.abs(frontLeftEncoder.getPosition() - convertDistanceToEncoder(distanceMeters)) <= 0.5;
  }

  public double convertDistanceToEncoder(double meters) {
    return (meters / DriveConstants.wheelCircumference) * 42 / 12;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Encoder Position", frontLeftEncoder.getPosition());
    SmartDashboard.putNumber("Left Encoder Velocity", frontLeftEncoder.getVelocity());
  }
}
