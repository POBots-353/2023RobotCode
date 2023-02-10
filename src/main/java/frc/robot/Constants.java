// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static int operatorStickPort = 1;
  }

  public static class Buttons {
    // Elevator
    public static final int elevatorHighButton = 7;
    public static final int elevatorMidButton = 6;
    public static final int elevatorLowButton = 5;
    public static final int toggleElevatorPistonsButton = 8;

    // Intake
    public static final int intakeInButton = 13;
    public static final int intakeOutButton = 12;
    public static final int intakeOpenClose = 14;

    // Drive
    public static final int toggleBrakesButton = 3;
  }

  public static class DriveConstants {
    public static final int frontLeftMotorID = 4; // 10 4
    public static final int backLeftMotorID = 2; // 14 2
    public static final int frontRightMotorID = 1; // 9 1
    public static final int backRightMotorID = 3; // 5 3

    public static final double defaultSpeed = 0.45;
    public static final double slowSpeed = 0.25;

    public static final double tapeAlignmentTolerance = 1.50;
    public static final double tapeAlignmentPitch = -10.0;

    public static final double gearBoxRatio = 10.7;

    public static final double trackDiameter = 0.581025;

    public static final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(trackDiameter);

    public static final double wheelCircumference = 0.1524 * Math.PI;

    public static final double distanceToEncoderRatio = 2 * 42 / (wheelCircumference * gearBoxRatio);
    public static final double encoderToDistanceRatio = 0.5 * wheelCircumference * gearBoxRatio / 42;

    public final static int pistonBrakeForwardID = 8;
    public final static int pistonBrakeReverseID = 7;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1.25;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    public static final double kRamseteB = 1.25; // 2
    public static final double kRamseteZeta = 0.7; // 0.7
    public static final double ksVolts = 0.22; // 0.22
    public static final double kvVoltSecondsPerMeter = 1.98; // 1.98
    public static final double kaVoltSecondsSquaredPerMeter = 0.2; // 0.2

    public static final double kPDriveVel = 0.0004;
  }

  public static class IntakeConstants {
    public static final int elevatorMotorID = 5;

    public static final int elevatorPistonForwardID = 6;
    public static final int elevatorPistonReverseID = 9;

    public static final int leftIntakeMotorID = 6;
    public static final int rightIntakeMotorID = 7;

    public static final int leftActuatorID = 0;
    public static final int rightActuatorID = 0;

    public static final int intakePistonForwardID = 0;
    public static final int intakePistonReverseID = 0;

    public static final int manipulatorBreakForwardID = 0;
    public static final int manipulatorBreakReverseID = 0;

    public static final double intakeSpeed = 0.5;
    public static final double elevatorTopSetPoint = -60;
    public static final double elevatorMidSetPoint = -34;
    public static final double elevatorLowSetPoint = 0;

    public static final double actuatorDownPosition = 1.0;
  }
}
