// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class ElementTransitSubsystem extends SubsystemBase {
  // Pneumatic stuff
  private Compressor pcmCompressor = new Compressor(1, PneumaticsModuleType.REVPH);

  private PneumaticHub pneumaticHub = new PneumaticHub(1);

  // Intake objects
  private CANSparkMax leftIntakeMotor = new CANSparkMax(IntakeConstants.leftIntakeMotorID, MotorType.kBrushless);
  private CANSparkMax rightIntakeMotor = new CANSparkMax(IntakeConstants.rightIntakeMotorID, MotorType.kBrushless);

  private DoubleSolenoid intakePiston = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      IntakeConstants.intakePistonForwardID, IntakeConstants.intakePistonReverseID);

  // Elevator objects
  private CANSparkMax elevatorMotor = new CANSparkMax(IntakeConstants.elevatorMotorID, MotorType.kBrushless);

  private SparkMaxPIDController elevatorPIDController = elevatorMotor.getPIDController();

  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

  private DoubleSolenoid elevatorPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      IntakeConstants.elevatorPistonForwardID, IntakeConstants.elevatorPistonReverseID);

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

  /** Creates a new ElementTransitSubsystem. */
  public ElementTransitSubsystem() {
    pcmCompressor.enableDigital();

    intakePiston.set(Value.kReverse);

    elevatorPiston.set(Value.kReverse);

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

  public void inTake() {
    leftIntakeMotor.set(IntakeConstants.intakeSpeed);
    rightIntakeMotor.set(IntakeConstants.intakeSpeed);
  }

  public void outTake() {
    leftIntakeMotor.set(-IntakeConstants.intakeSpeed);
    rightIntakeMotor.set(-IntakeConstants.intakeSpeed);
  }

  public void stopClawMotors() {
    leftIntakeMotor.set(0);
    rightIntakeMotor.set(0);
  }

  public void openCloseClaw() {
    intakePiston.toggle();
  }

  public void toggleElevatorTilt() {
    elevatorPiston.toggle();
  }

  public void setElevatorPosition(double elevatorPos) {
    elevatorPIDController.setReference(elevatorPos, CANSparkMax.ControlType.kSmartMotion);
  }

  public double getElevatorPosition() {
    // return elevatorEncoder.getPosition();
    return 0;
  }
  
  public void toggleOnManipulatorBreak(){
    manipulatorBreak.set(Value.kForward);
  }
  
  public void toggleOffManipulatorBreak(){
    manipulatorBreak.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Elevator Position", elevatorEncoder.getPosition());
    // SmartDashboard.putNumber("Pressure", pneumaticHub.getPressure(0));
  }

}
