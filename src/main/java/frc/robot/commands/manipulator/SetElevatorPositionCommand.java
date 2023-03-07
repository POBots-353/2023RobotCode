// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorPositionCommand extends CommandBase {
  private ElevatorSubsystem elevatorSystem;
  private double elevatorPosition;

  /** Creates a new LowManipulator. */
  public SetElevatorPositionCommand(double position, ElevatorSubsystem elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSystem = elevator;
    elevatorPosition = position;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSystem.toggleOffManipulatorBreak();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSystem.setElevatorPosition(elevatorPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSystem.toggleOnManipulatorBreak();
    elevatorSystem.elevatorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    return Math.abs(elevatorSystem.getElevatorPosition() - elevatorPosition) < 0.2;
  }
}
