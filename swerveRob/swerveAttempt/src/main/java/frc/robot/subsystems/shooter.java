// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class shooter extends SubsystemBase {
  /** Creates a new shooter. */
  private final Spark feeder = new Spark(0);
  private final TalonFX topFlyWheel = new TalonFX(Constants.top_flywheel);
  private final TalonFX bottomFlyWheel = new TalonFX(Constants.bottom_flywheel);
  private TalonFXConfiguration config = new TalonFXConfiguration();
  public shooter() {
    config.Voltage.PeakForwardVoltage= Constants.kMaxFlywheelVoltage;
    config.Voltage.PeakReverseVoltage= Constants.kMaxFlywheelVoltage;
    config.CurrentLimits.SupplyCurrentLimit = Constants.kMaxFlywheelCurrent;
    motor__init__(topFlyWheel, Constants.top_flywheel_reversed);
    motor__init__(bottomFlyWheel, Constants.bottom_flywheel_reversed);
    
  }
  private void motor__init__(TalonFX mot, boolean inverse){
    mot.getConfigurator().apply(config);
    mot.setNeutralMode(NeutralModeValue.Coast);
    mot.setInverted(inverse);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void feedIntoShooter(){
    feeder.set(0.4);
  }
}
