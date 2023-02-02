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
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class ElementTransitSubsystem extends SubsystemBase {
  // private Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  // private DoubleSolenoid longSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
  // private DoubleSolenoid shortSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
  // private CANSparkMax leftMotor = new CANSparkMax(0, MotorType.kBrushless);
  // private CANSparkMax rightMotor = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax elevator = new CANSparkMax(7, MotorType.kBrushless);
  private DoubleSolenoid leftPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
  private DoubleSolenoid rightPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
  private SparkMaxPIDController elevatorPid = elevator.getPIDController();

  private RelativeEncoder elevatorEncoder = elevator.getEncoder();
  
  private DoubleSolenoid manipulatorBreak = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1,2 );
  
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
    // pcmCompressor.enableDigital();
    // pcmCompressor.disable();
    //upper 4 lines enable/disable compressor, return if compressor active,
    // shortSolenoid.set(Value.kReverse);//placeholders
    // longSolenoid.set(Value.kReverse);
    // leftPiston.set(Value.kReverse);
    // rightPiston.set(Value.kReverse);

    initializePID(elevatorPid);
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
  public void runClawMotors() {
    // leftMotor.set(IntakeConstants.intakeSpeed);
    // rightMotor.set(IntakeConstants.intakeSpeed);
  }
  public void stopClawMotors(){
    // leftMotor.set(0);
    // rightMotor.set(0);
  }
  public void toggleShort() {
    // shortSolenoid.toggle();
  }
  public void toggleLong() {
    // longSolenoid.toggle();
  }
  public void elevatorOn() {
    elevator.set(IntakeConstants.elevatorPulleySpeed);
  }
  public void elevatorOff() {
    elevator.set(0);
  }
  public void elevatorTiltOn() {
    // leftPiston.toggle();
    // rightPiston.toggle();
  }
  public void elevatorHigh() {
    // elevatorPid.setReference(IntakeConstants.elevatorTopSetPoint, CANSparkMax.ControlType.kPosition);
  }
  public void elevatorMid() {
    // elevatorPid.setReference(IntakeConstants.elevatorMidSetPoint, CANSparkMax.ControlType.kPosition);
  }
  public void elevatorLow() {
    // elevatorPid.setReference(IntakeConstants.elevatorLowSetPoint, CANSparkMax.ControlType.kPosition);
  }
  public void setElevatorPosition(double elevatorPos){
    elevatorPid.setReference(elevatorPos, CANSparkMax.ControlType.kSmartMotion);
    // leftPiston.set(Value.kForward);
    // rightPiston.set(Value.kForward);
  }
  public double getElevatorPosition() {
    return elevatorEncoder.getPosition();
  }
  public void toggleOnManipulatorBreak(){
    manipulatorBreak.set(Value.kForward);
  }
  public void toggleOffManipulatorBreak(){
    manipulatorBreak.set(Value.kReverse);
  }
  // turn on motor then pistons then turn motor off

  // public static void main(String[] args) { //main method
  //   System.out.println("Kurt sucks"); //kurt sucks
  // } charlie's footprint

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", elevatorEncoder.getPosition());
  } 

}
