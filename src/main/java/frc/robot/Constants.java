// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int frontLeftMotorID = 4; // 10
    public static final int backLeftMotorID = 2; // 14
    public static final int frontRightMotorID = 1; // 9
    public static final int backRightMotorID = 3; // 5

    public static final double defaultSpeed = 0.45;
    public static final double slowSpeed = 0.25;

    public static final double tapeAlignmentTolerance = 1.50;
    public static final double tapeAlignmentPitch = -10.0;

    public static final double gearBoxRatio = 10.7;

    public static final double wheelCircumference = 0.1524 * Math.PI;

    public final static int pistonBrakeForwardID = 1;
    public final static int pistonBrakeReverseID = 2;
  }

  public static class IntakeConstants {
    public static final int elevatorMotorID = 0;

    public static final int elevatorPistonForwardID = 0;
    public static final int elevatorPistonReverseID = 0;

    public static final int leftIntakeMotorID = 0;
    public static final int rightIntakeMotorID = 0;

    public static final int intakePistonForwardID = 0;
    public static final int intakePistonReverseID = 0;

    public static final int manipulatorBreakForwardID = 0;
    public static final int manipulatorBreakReverseID = 0;

    public static final double intakeSpeed = 0.5;
    public static final double elevatorTopSetPoint = 3000;
    public static final double elevatorMidSetPoint = 1500;
    public static final double elevatorLowSetPoint = 0;
  }
}
