// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElementTransitSubsystem extends SubsystemBase {
  private Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private DoubleSolenoid leftShortSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
  private DoubleSolenoid leftLongSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
  private DoubleSolenoid rightShortSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
  private DoubleSolenoid rightLongSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
  /** Creates a new ElementTransitSubsystem. */
  public ElementTransitSubsystem() {
    pcmCompressor.enableDigital();
    pcmCompressor.disable();
    boolean enabled = pcmCompressor.isEnabled();
    boolean pressureSwitch = pcmCompressor.getPressureSwitchValue();
    //upper 4 lines enable/disable compressor, return if compressor active, returns state of pressure switch in piston
    rightLongSolenoid.set(Value.kReverse);//placeholders
    rightShortSolenoid.set(Value.kReverse);
    leftLongSolenoid.set(Value.kReverse);
    leftShortSolenoid.set(Value.kReverse);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
