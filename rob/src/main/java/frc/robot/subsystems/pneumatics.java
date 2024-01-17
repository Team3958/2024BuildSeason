// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class pneumatics extends SubsystemBase {
  /** Creates a new pneumatics. */
  private final DoubleSolenoid piston = new DoubleSolenoid(Constants.pcm, 0,1);

  public pneumatics() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setForword(){
    piston.set(Value.kForward);
  }

  public void setReverse(){
    piston.set(Value.kReverse);
  }

  public void setOff(){
    piston.set(Value.kOff);
  }
}
