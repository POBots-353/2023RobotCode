// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private DigitalOutput redOutput = new DigitalOutput(5);
  private DigitalOutput greenOutput = new DigitalOutput(6);
  private DigitalOutput blueOutput = new DigitalOutput(7);

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
  }

  public void initializeAllianceColor() {
    switch (DriverStation.getAlliance()) {
      case Red:
        setRed();
        break;
      case Blue:
        setBlue();
        break;
      default:
        break;
    }
  }

  public void setBlue() {
    redOutput.set(false);
    greenOutput.set(false);
    blueOutput.set(true);
  }

  public void setRed() {
    redOutput.set(true);
    greenOutput.set(false);
    blueOutput.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
