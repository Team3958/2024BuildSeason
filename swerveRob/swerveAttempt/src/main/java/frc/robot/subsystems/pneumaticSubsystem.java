// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class pneumaticSubsystem extends SubsystemBase {
  /** Creates a new pneumaticSubsystem. */
  //Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  private final DoubleSolenoid climber1 = new DoubleSolenoid(Constants.PCM,PneumaticsModuleType.CTREPCM, 0,1);
  private final DoubleSolenoid climber2 = new DoubleSolenoid(Constants.PCM,PneumaticsModuleType.CTREPCM, 2,3);
 
  public pneumaticSubsystem() {
    //compressor.enableDigital();
  }
  public void down(){
    climber1.set(Value.kReverse);
    climber2.set(Value.kReverse);
  }
  public void up(){
    climber1.set(Value.kForward);
    climber2.set(Value.kForward);
  }
  public void off(){
    climber1.set(Value.kOff);
    climber2.set(Value.kOff);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
