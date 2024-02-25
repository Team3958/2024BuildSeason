// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class pneumaticSubsystem extends SubsystemBase {
  /** Creates a new pneumaticSubsystem. */
  //Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  private final DoubleSolenoid intake1 = new DoubleSolenoid(Constants.PCM,PneumaticsModuleType.CTREPCM, 0,1);
  private final DoubleSolenoid intake2 = new DoubleSolenoid(Constants.PCM,PneumaticsModuleType.CTREPCM, 0,1);
  private final DoubleSolenoid climber = new DoubleSolenoid(Constants.PCM,PneumaticsModuleType.CTREPCM, 0,1);
  public pneumaticSubsystem() {
    //compressor.enableDigital();
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
