// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElementTransitSubsystem;

public class SetElevatorPositionCommand extends CommandBase {
  private ElementTransitSubsystem elementTransit;
  private double elevatorPosition;
  /** Creates a new LowManipulator. */
  public SetElevatorPositionCommand(double position, ElementTransitSubsystem elementTransit) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elementTransit = elementTransit;
    elevatorPosition = position;
    addRequirements(elementTransit);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elementTransit.toggleOffManipulatorBreak();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elementTransit.setElevatorPosition(elevatorPosition);
    if(elevatorPosition == elementTransit.getElevatorPosition() ){
      elementTransit.toggleOnManipulatorBreak();
    } else {
      elementTransit.toggleOffManipulatorBreak();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elementTransit.getElevatorPosition() == elevatorPosition;
  }
}
