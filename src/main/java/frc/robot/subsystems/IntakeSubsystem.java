// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  // Pneumatic stuff
  private Compressor pcmCompressor = new Compressor(1, PneumaticsModuleType.REVPH);

  // Intake objects
  private DoubleSolenoid intakePiston = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      IntakeConstants.intakePistonForwardID, IntakeConstants.intakePistonReverseID);

  private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);

  private Ultrasonic cubeUltrasonic = new Ultrasonic(0, 1);
  private Ultrasonic coneUltrasonic = new Ultrasonic(3, 4);

  /** Creates a new ElementTransitSubsystem. */
  public IntakeSubsystem() {
    pcmCompressor.enableDigital();

    intakePiston.set(Value.kReverse);
  }

  public CommandBase autoIntakeCube() {
    return Commands.race(run(this::intakeCube), new WaitCommand(IntakeConstants.autoIntakeTime))
        .andThen(runOnce(this::stopIntakeMotor));
  }

  public CommandBase autoOuttakeCube() {
    return Commands.race(run(this::outTakeCube), new WaitCommand(IntakeConstants.autoIntakeTime))
        .andThen(runOnce(this::stopIntakeMotor));
  }

  public CommandBase autoIntakeCone() {
    return Commands.race(run(this::intakeCone), new WaitCommand(IntakeConstants.autoIntakeTime))
        .andThen(runOnce(this::stopIntakeMotor));
  }

  public CommandBase autoOuttakeCone() {
    return Commands.race(run(this::outTakeCone), new WaitCommand(IntakeConstants.autoIntakeTime))
        .andThen(runOnce(this::stopIntakeMotor));
  }

  public void intakeCube() {
    // double distanceInches = cubeUltrasonic.getRangeInches();
    // if (distanceInches <= 15.0) {
    // stopIntakeMotor();
    // }
    intakeMotor.set(IntakeConstants.intakeSpeed);
  }

  public void outTakeCube() {
    intakeMotor.set(-IntakeConstants.intakeSpeed);
  }

  public void intakeCone() {
    intakeMotor.set(-IntakeConstants.intakeSpeed);
  }

  public void outTakeCone() {
    intakeMotor.set(IntakeConstants.intakeSpeed);
  }

  public void stopIntakeMotor() {
    intakeMotor.set(0);
  }

  public void toggleIntakePiston() {
    intakePiston.toggle();
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Pressure", pneumaticHub.getPressure(0));
    // if (topLimitSwitch.get()) {
    // elevatorEncoder.setPosition(0);
    // }

    SmartDashboard.putNumber("Cube Distance", cubeUltrasonic.getRangeInches());
    SmartDashboard.putNumber("Cone Distance", coneUltrasonic.getRangeInches());
  }
}
