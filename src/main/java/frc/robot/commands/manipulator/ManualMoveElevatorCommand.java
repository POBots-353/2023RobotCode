// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualMoveElevatorCommand extends CommandBase {
  private ElevatorSubsystem elevatorSystem;

  private double speed;

  /** Creates a new ManualMoveElevatorCommand. */
  public ManualMoveElevatorCommand(double speed, ElevatorSubsystem elevatorSystem) {
    this.speed = speed;
    this.elevatorSystem = elevatorSystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSystem.toggleOffManipulatorBreak();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSystem.setElevatorSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSystem.elevatorStop();
    elevatorSystem.toggleOnManipulatorBreak();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return speed > 0 && elevatorSystem.bottomSwitchPressed();
  }
}
