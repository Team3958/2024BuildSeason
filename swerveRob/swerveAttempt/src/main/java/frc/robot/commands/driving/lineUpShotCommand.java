// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driving;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class lineUpShotCommand extends Command {
  /** Creates a new lineUpShotCommand. */
  private final SwerveSubsystem dt;
  private DoubleSupplier distanceX;
  private DoubleSupplier distanceY;
  private DoubleSupplier theta;
  double radius = 1.2;
  
  public lineUpShotCommand(SwerveSubsystem dt, DoubleSupplier distanceX,DoubleSupplier distanceY,
  DoubleSupplier theta) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dt = dt;
    this.distanceX = distanceX;
    this.distanceY = distanceY;
    this.theta = theta;
    addRequirements(dt);
  }
  double x = distanceX.getAsDouble();
  double desiredTheta = Math.acos(x/radius);
  double desiredY = radius*Math.sin(desiredTheta);
  
  private final ProfiledPIDController yController = new ProfiledPIDController(Constants.yP, 0, 0, new TrapezoidProfile.Constraints(2,2));
  private final ProfiledPIDController thetaController = new ProfiledPIDController(Constants.thetaP, 0, 0, new TrapezoidProfile.Constraints(2,2));


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yController.setGoal(desiredY);
    yController.setTolerance(0.1);
    thetaController.setGoal(desiredTheta);
    thetaController.setTolerance(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new Rotation2d();
    dt.setStatesFromChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(yController.calculate(distanceY.getAsDouble(), desiredY), 0, thetaController.calculate(theta.getAsDouble(), desiredTheta), Rotation2d.fromDegrees(theta.getAsDouble())));
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
