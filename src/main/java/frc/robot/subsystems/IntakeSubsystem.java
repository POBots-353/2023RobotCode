// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  // Pneumatic stuff
  private Compressor pcmCompressor = new Compressor(IntakeConstants.pneumaticHubID, PneumaticsModuleType.REVPH);

  private PneumaticHub pneumaticHub = new PneumaticHub(IntakeConstants.pneumaticHubID);

  // Intake objects
  private DoubleSolenoid intakeWristPiston = new DoubleSolenoid(IntakeConstants.pneumaticHubID,
      PneumaticsModuleType.REVPH,
      IntakeConstants.intakePistonForwardID, IntakeConstants.intakePistonReverseID);

  private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);

  /** Creates a new ElementTransitSubsystem. */
  public IntakeSubsystem() {
    pcmCompressor.enableDigital();

    pneumaticHub.clearStickyFaults();
    intakeMotor.clearFaults();

    intakeWristPiston.set(Value.kForward);
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
    return run(this::intakeCone).withTimeout(0.50).andThen(runOnce(this::stopIntakeMotor));
  }

  public CommandBase autoOuttakeCone() {
    return Commands.race(run(this::outTakeCone), new WaitCommand(IntakeConstants.autoIntakeTime))
        .andThen(runOnce(this::stopIntakeMotor));
  }

  public void disableCompressor() {
    pcmCompressor.disable();
  }

  public void enableCompressor() {
    pcmCompressor.enableDigital();
  }

  public void intakeCube() {
    // double distanceInches = cubeUltrasonic.getRangeInches();
    // if (distanceInches <= 15.0) {
    // stopIntakeMotor();
    // }
    intakeMotor.set(IntakeConstants.intakeSpeed);
  }

  public void outTakeCube() {
    intakeMotor.set(-IntakeConstants.outtakeSpeed);
  }

  public void intakeCone() {
    intakeMotor.set(-IntakeConstants.intakeSpeed);
  }

  public void outTakeCone() {
    intakeMotor.set(IntakeConstants.outtakeSpeed);
  }

  public void stopIntakeMotor() {
    intakeMotor.set(0);
  }

  public void toggleIntakePiston() {
    intakeWristPiston.toggle();
  }

  public void toggleWristIn() {
    intakeWristPiston.set(Value.kReverse);
  }

  public void toggleWristOut() {
    intakeWristPiston.set(Value.kForward);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pressure", pneumaticHub.getPressure(0));
  }
}
