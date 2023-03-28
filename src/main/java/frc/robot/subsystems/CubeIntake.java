// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CubeIntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

public class CubeIntake extends SubsystemBase {
  private CANSparkMax cubeIntakeSnapper = new CANSparkMax(0, MotorType.kBrushless);
  private RelativeEncoder cubeIntakeEncoder = cubeIntakeSnapper.getEncoder();
  private SparkMaxPIDController cubeIntakePID = cubeIntakeSnapper.getPIDController();
  
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
  /** Creates a new Intake3. */
  public CubeIntake() {
    initializePID(cubeIntakePID);
    cubeIntakeSnapper.restoreFactoryDefaults();
    cubeIntakeEncoder.setPosition(0);//make constant later
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

  public double getBottomWristSnapperPos(){
    return cubeIntakeEncoder.getPosition();
  }

  public void setHighIntakePos() {
    cubeIntakeEncoder.setPosition(CubeIntakeConstants.lowCubeIntakePos);
  }
  public void setMidIntakePos() {
    cubeIntakeEncoder.setPosition(CubeIntakeConstants.middleCubeIntakePos);
  }
  public void setLowIntakePos() {
    cubeIntakeEncoder.setPosition(CubeIntakeConstants.highCubeIntakePos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
