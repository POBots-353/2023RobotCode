// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ManualMoveElevatorCommand extends CommandBase {
  private Elevator elevatorSystem;

  private BooleanSupplier limitSwitchOverride;
  private double speed;

  private SlewRateLimiter rateLimiter = new SlewRateLimiter(3.53);

  /** Creates a new ManualMoveElevatorCommand. */
  public ManualMoveElevatorCommand(double speed, BooleanSupplier limitSwitchOverride,
      Elevator elevatorSystem) {
    this.speed = speed;

    this.limitSwitchOverride = limitSwitchOverride;

    this.elevatorSystem = elevatorSystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSystem);
  }

  /** Creates a new ManualMoveElevatorCommand. */
  public ManualMoveElevatorCommand(double speed, Elevator elevator) {
    this(speed, () -> false, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSystem.toggleOffManipulatorBreak();

    rateLimiter.reset(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSystem.setElevatorSpeed(rateLimiter.calculate(speed));
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
    if (limitSwitchOverride.getAsBoolean()) {
      return false;
    }

    if (speed > 0) {
      return elevatorSystem.bottomSwitchPressed();
    }

    return false;
  }
}
