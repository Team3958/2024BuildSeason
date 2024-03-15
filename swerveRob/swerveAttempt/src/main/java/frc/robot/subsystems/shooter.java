// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.util.PrimitiveArrayBuilder;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class shooter extends SubsystemBase {
  /** Creates a new shooter. */
  
  private final TalonFX topFlyWheel = new TalonFX(Constants.top_flywheel);
  private final TalonFX bottomFlyWheel = new TalonFX(Constants.bottom_flywheel);
  private TalonFXConfiguration config = new TalonFXConfiguration();
  private final SimpleMotorFeedforward shooterffTop = new SimpleMotorFeedforward(Constants.KSshootTop, Constants.KVshooterTop, Constants.KAshooterTop);
  private final SimpleMotorFeedforward shooterffBottom = new SimpleMotorFeedforward(Constants.KSshootBottom, Constants.KVshooterBottom, Constants.KAshooterBottom);
  //private final BangBangController controller1 = new BangBangController();
  //\private final BangBangController controller2 = new BangBangController();

  public shooter() {
    config.Voltage.PeakForwardVoltage= Constants.kMaxFlywheelVoltage;
    config.Voltage.PeakReverseVoltage= Constants.kMaxFlywheelVoltage;
    config.CurrentLimits.SupplyCurrentLimit = Constants.kMaxFlywheelCurrent;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.kMaxFlywheelCurrent;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
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
    SmartDashboard.putNumber("top flywheel vel", getTopVelocity());
    SmartDashboard.putNumber("bottom flywheel vel", getBottomVelocity());
  }
 
  public void shoot(){
    double velocity= 7.5;
    topFlyWheel.setVoltage(shooterffTop.calculate(velocity));//+controller1.calculate(getTopVelocity());
    bottomFlyWheel.setVoltage(shooterffBottom.calculate(velocity));//+controller2.calculate(getBottomVelocity()));
  }

  public double getTopVelocity(){
    return Units.rotationsToRadians(topFlyWheel.getVelocity().getValueAsDouble())*Constants.WHEELRADIUS
    *Constants.top_flywheel_ratio;
  }
  public double getBottomVelocity(){
    return Units.rotationsToRadians(bottomFlyWheel.getVelocity().getValueAsDouble())*Constants.WHEELRADIUS
    *Constants.bottom_flywheel_ratio;
  }
  public void zero(){
    topFlyWheel.set(0);
    bottomFlyWheel.set(0);
  }
  public void shootAmp(){
    double ampvelocity= 0.2;
    topFlyWheel.setVoltage(shooterffTop.calculate(ampvelocity));//+controller1.calculate(getTopVelocity());
    bottomFlyWheel.setVoltage(shooterffBottom.calculate(ampvelocity*0.5));//+controller2.calculate(getBottomVelocity()));
  }
  
}
