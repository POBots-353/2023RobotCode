// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class SetElevatorPosition extends CommandBase {
  private Elevator elevator;
  private double elevatorPosition;
  private DoubleSupplier positionSupplier;

  private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
      ElevatorConstants.elevatorMaxVelocity,
      ElevatorConstants.elevatorMaxAcceleration);

  /** Creates a new SetElevatorPosition. */
  public SetElevatorPosition(DoubleSupplier position, Elevator elevator) {
    positionSupplier = position;
    this.elevator = elevator;

    addRequirements(elevator);
  }

  public SetElevatorPosition(double position, Elevator elevator) {
    this(() -> position, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorPosition = positionSupplier.getAsDouble();

    elevator.toggleOffManipulatorBreak();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPosition = elevator.getElevatorPosition();
    double currentVelocity = elevator.getElevatorVelocity();

    TrapezoidProfile profile = new TrapezoidProfile(constraints, new TrapezoidProfile.State(elevatorPosition, 0),
        new TrapezoidProfile.State(currentPosition, currentVelocity));

    elevator.setElevatorProfile(profile.calculate(0.020));
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
    return Math.abs(elevator.getElevatorPosition() - elevatorPosition) < 0.10;
  }
}
