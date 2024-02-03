// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import java.io.Console;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.falcon;

public class positionCommand extends Command {
  /** Creates a new positionCommand. */
  private falcon motor;
  private double position;
  private PIDController controller = new PIDController(Constants.kP, Constants.kI, Constants.kD);

  
  public positionCommand(falcon  motor, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.motor = motor;
    this.position = position;
    addRequirements(motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    motor.reset_pose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motor.setVoltage(controller.calculate(motor.getPosition(),position));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
