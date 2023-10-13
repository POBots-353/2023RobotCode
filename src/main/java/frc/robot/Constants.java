// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

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
    public static final int operatorStickPort = 1;
  }

  public static class Buttons {
    // Cone/Cube Mode
    public static final int cubeModeButton = 16;
    // Elevator
    public static final int elevatorHighButton = 7;
    public static final int elevatorMidButton = 6;
    public static final int elevatorLowButton = 5;
    public static final int toggleElevatorPistonsButton = 8;

    public static final int elevatorManualUpButton = 10;
    public static final int elevatorManualDownButton = 9;

    public static final int elevatorLimitSwitchOverride = 2;

    // Intake
    public static final int intakeButton = 13;
    public static final int outtakeButton = 12;

    public static final int intakeOpenClose = 11;

    // Drive
    public static final int toggleBrakesButton = 1;
  }

  public static class FieldPositionConstants {
    public static final int SUBSTATION_SIDE = 1;
    public static final int CHARGE_STATION = 2;
    public static final int FIELD_EDGE = 3;

    public static final Map<Integer, Pose3d> aprilTags = Map.of(
        1,
        new Pose3d(
            Units.inchesToMeters(610.77),
            Units.inchesToMeters(42.19),
            Units.inchesToMeters(18.22),
            new Rotation3d(0.0, 0.0, Math.PI)),
        2,
        new Pose3d(
            Units.inchesToMeters(610.77),
            Units.inchesToMeters(108.19),
            Units.inchesToMeters(18.22),
            new Rotation3d(0.0, 0.0, Math.PI)),
        3,
        new Pose3d(
            Units.inchesToMeters(610.77),
            Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
            Units.inchesToMeters(18.22),
            new Rotation3d(0.0, 0.0, Math.PI)),
        4,
        new Pose3d(
            Units.inchesToMeters(636.96),
            Units.inchesToMeters(265.74),
            Units.inchesToMeters(27.38),
            new Rotation3d(0.0, 0.0, Math.PI)),
        5,
        new Pose3d(
            Units.inchesToMeters(14.25),
            Units.inchesToMeters(265.74),
            Units.inchesToMeters(27.38),
            new Rotation3d()),
        6,
        new Pose3d(
            Units.inchesToMeters(40.45),
            Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
            Units.inchesToMeters(18.22),
            new Rotation3d()),
        7,
        new Pose3d(
            Units.inchesToMeters(40.45),
            Units.inchesToMeters(108.19),
            Units.inchesToMeters(18.22),
            new Rotation3d()),
        8,
        new Pose3d(
            Units.inchesToMeters(40.45),
            Units.inchesToMeters(42.19),
            Units.inchesToMeters(18.22),
            new Rotation3d()));
  }

  public static class DriveConstants {
    public static final String limelightName = "limelight";

    public static final int powerDistributionID = 15;

    public static final int frontLeftMotorID = 4;
    public static final int backLeftMotorID = 2;
    public static final int frontRightMotorID = 1;
    public static final int backRightMotorID = 3;

    public static final double turboSpeed = 0.825;
    public static final double defaultSpeed = 0.675;// 0.65
    public static final double slowSpeed = 0.25;

    public static final double tapeAlignmentPitchTolerance = 1.50;
    public static final double tapeAlignmentAreaTolerance = 0.027;
    public static final double tapeAlignmentPitch = 10.5;
    public static final double tapeAlignmentArea = 0.295;

    public static final double gearBoxRatio = 8.45; // 8.41, 10.7

    public static final double trackDiameter = 0.581025;

    public static final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(trackDiameter);

    public static final double wheelCircumference = 0.1524 * Math.PI;

    public static final double distanceToEncoderRatio = gearBoxRatio / wheelCircumference;
    public static final double encoderToDistanceRatio = wheelCircumference / gearBoxRatio;

    public static final double coneCenterMM = 3.535353;
    public static final double cubeCenterMM = 3.535353;

    // The ratio of how many degrees we need to offset by the MM the cube/cone is
    // from the center
    public static final double mmDegreesOffsetRatio = 3.53535353 / 20.8; // measured millimeters vs the degrees from the
                                                                         // center

    public final static int pistonBrakeForwardID = 4; // 8
    public final static int pistonBrakeReverseID = 2; // 7
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.750;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.75;

    public static final double kRamseteB = 2; // 2
    public static final double kRamseteZeta = 0.7; // 0.7
    public static final double ksVolts = -0.34902; // 0.22 0.36108 -0.34902
    public static final double kvVoltSecondsPerMeter = 2.3678; // 1.98 2.4237 2.3678
    public static final double kaVoltSecondsSquaredPerMeter = 0.45517; // 0.2 0.43606 0.45517

    public static final double kPDriveVel = 0.59315; // 0.0004

    public static final Pose2d blueSubstationPose = new Pose2d(1.97, 4.84, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d blueChargeStationPose = new Pose2d(1.97, 3.30, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d blueFieldEdgePose = new Pose2d(1.97, 0.50, new Rotation2d(Math.toRadians(180)));

    public static final Pose2d redSubstationPose = new Pose2d(14.59, 4.84, new Rotation2d(Math.toRadians(0)));
    public static final Pose2d redChargeStationPose = new Pose2d(14.59, 3.30, new Rotation2d(Math.toRadians(0)));
    public static final Pose2d redFieldEdgePose = new Pose2d(14.59, 0.50, new Rotation2d(Math.toRadians(0)));
  }

  public static class IntakeConstants {
    public static final int pneumaticHubID = 0;

    public static final int elevatorMotorID = 8;

    public static final int elevatorPistonForwardID = 1; // 6
    public static final int elevatorPistonReverseID = 5; // 9

    public static final int intakeMotorID = 7; // 7

    public static final int leftActuatorID = 0;
    public static final int rightActuatorID = 0;

    public static final int intakePistonForwardID = 0; // 5
    public static final int intakePistonReverseID = 6; // 10

    public static final int manipulatorBreakForwardID = 3; // 4
    public static final int manipulatorBreakReverseID = 7; // 11

    public static final double intakeSpeed = 0.80;
    public static final double outtakeSpeed = 1.00;

    public static final double autoIntakeTime = 0.50;

    public static final double elevatorSpeed = 0.50;
  }

  public static final class ElevatorConstants {
    public static final double elevatorKg = 0.5324;
    public static final double elevatorKs = 0.23288;
    public static final double elevatorKv = 0.13315;
    public static final double elevatorKa = 0.0036323;

    public static final double elevatorMaxVelocity = 40.0;
    public static final double elevatorMaxAcceleration = 15.0;

    public static final double elevatorConeTopSetPoint = 59.6;
    public static final double elevatorConeMidSetPoint = 40.4;
    public static final double elevatorConeLowSetPoint = 2.45;

    public static final double elevatorCubeTopSetPoint = 59.0;
    public static final double elevatorCubeMidSetPoint = 45.8;
    public static final double elevatorCubeLowSetPoint = 1.5;

    public static final double startingConfigurationHeight = 59.30;
  }

  public static final class LEDConstants {
    public static final int redOutputChannel = 6;
    public static final int greenOutputChannel = 5;
    public static final int blueOutputChannel = 2;
  }

  public static final class CubeIntakeConstants {
    public static final double lowCubeIntakePos = 0;
    public static final double middleCubeIntakePos = 3;
    public static final double highCubeIntakePos = 6;
  }
}
