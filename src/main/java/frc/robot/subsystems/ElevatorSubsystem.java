// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private CANSparkMax elevatorMotor = new CANSparkMax(IntakeConstants.elevatorMotorID, MotorType.kBrushless);

  private SparkMaxPIDController elevatorPIDController = elevatorMotor.getPIDController();

  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

  private DoubleSolenoid elevatorPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      IntakeConstants.elevatorPistonForwardID, IntakeConstants.elevatorPistonReverseID);

  private DoubleSolenoid manipulatorBreak = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      IntakeConstants.manipulatorBreakForwardID, IntakeConstants.manipulatorBreakReverseID);

  private DigitalInput topLimitSwitch = new DigitalInput(1);
  private DigitalInput bottomLimitSwitch = new DigitalInput(9);

  private int smartMotionSlot = 0;
  private int allowedErr;
  private int minVel;
  private double kP = 4e-4;
  private double kI = 0;
  private double kD = 0;
  private double kIz = 0;
  private double kFF = 0.000156;
  private double kMaxOutput = 1;
  private double kMinOutput = -1;
  private double maxVel = 5000; // 5000
  private double maxAcc = 3000; // 2500

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorMotor.restoreFactoryDefaults();

    elevatorPiston.set(Value.kReverse);

    manipulatorBreak.set(Value.kForward);

    initializePID(elevatorPIDController);
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

  public void toggleElevatorTilt() {
    elevatorPiston.toggle();
  }

  public void setElevatorPosition(double elevatorPos) {
    elevatorPIDController.setReference(elevatorPos, CANSparkMax.ControlType.kSmartMotion);
  }

  public void elevatorUp() {
    elevatorMotor.set(-IntakeConstants.elevatorSpeed);
  }

  public void elevatorDown() {
    elevatorMotor.set(IntakeConstants.elevatorSpeed);
  }

  public void setElevatorSpeed(double speed) {
    elevatorMotor.set(speed);
  }

  public void elevatorStop() {
    elevatorMotor.set(0);
  }

  public double getElevatorPosition() {
    return elevatorEncoder.getPosition();
  }

  public void toggleOnManipulatorBreak() {
    manipulatorBreak.set(Value.kForward);
  }

  public void toggleOffManipulatorBreak() {
    manipulatorBreak.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position", elevatorEncoder.getPosition());

    // SmartDashboard.putNumber("Pressure", pneumaticHub.getPressure(0));
    // if (topLimitSwitch.get()) {
    //   elevatorEncoder.setPosition(0);
    // }
    // if (bottomLimitSwitch.get()) {
    //   elevatorEncoder.setPosition(IntakeConstants.elevatorTopSetPoint);
    // }
  }
}