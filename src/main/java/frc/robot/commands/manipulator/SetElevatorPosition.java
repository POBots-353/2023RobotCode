// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class SetElevatorPosition extends CommandBase {
  private Elevator elevator;
  private double elevatorPosition;

  /** Creates a new LowManipulator. */
  public SetElevatorPosition(double position, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    elevatorPosition = position;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.toggleOffManipulatorBreak();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setElevatorPosition(elevatorPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.toggleOnManipulatorBreak();
    elevator.elevatorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    return Math.abs(elevator.getElevatorPosition() - elevatorPosition) < 0.10;
  }
}
