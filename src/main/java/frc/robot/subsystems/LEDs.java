// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase {
  private DigitalOutput redOutput = new DigitalOutput(LEDConstants.redOutputChannel);
  private DigitalOutput greenOutput = new DigitalOutput(LEDConstants.greenOutputChannel);
  private DigitalOutput blueOutput = new DigitalOutput(LEDConstants.blueOutputChannel);

  /** Creates a new LEDSubsystem. */
  public LEDs() {
    initializeAllianceColor();
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
        setGreen();
        break;
    }
  }

  public void setBlue() {
    redOutput.set(true);
    greenOutput.set(true);
    blueOutput.set(false);
  }

  public void setRed() {
    redOutput.set(false);
    greenOutput.set(true);
    blueOutput.set(true);
  }

  public void setGreen() {
    redOutput.set(true);
    greenOutput.set(false);
    blueOutput.set(true);
  }

  public void setYellow() {
    redOutput.set(false);
    greenOutput.set(false);
    blueOutput.set(true);
  }

  public void setPurple() {
    redOutput.set(false);
    greenOutput.set(true);
    blueOutput.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
