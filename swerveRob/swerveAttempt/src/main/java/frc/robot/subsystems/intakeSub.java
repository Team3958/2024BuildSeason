// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class intakeSub extends SubsystemBase {
  /** Creates a new intakeSub. */
  private final TalonFX intakeMot = new TalonFX(Constants.intake);
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(2.8, 2.5);
  public intakeSub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void runIntake(boolean reverse){
    int direction =(reverse)? -1:1;
    //intakeMot.set(0.7*direction);
    intakeMot.setVoltage(ff.calculate(direction));
  }
  public void zero(){
    intakeMot.set(0);
  }
}
