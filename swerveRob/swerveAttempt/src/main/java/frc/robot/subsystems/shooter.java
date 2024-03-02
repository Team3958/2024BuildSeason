// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.util.PrimitiveArrayBuilder;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class shooter extends SubsystemBase {
  /** Creates a new shooter. */
  private final TalonFX feeder = new TalonFX(Constants.intake);
  private final TalonFX topFlyWheel = new TalonFX(Constants.top_flywheel);
  private final TalonFX bottomFlyWheel = new TalonFX(Constants.bottom_flywheel);
  private TalonFXConfiguration config = new TalonFXConfiguration();
  private final SimpleMotorFeedforward shooterff = new SimpleMotorFeedforward(Constants.KSshoot, Constants.KVshooter, Constants.KAshooter);
  private final BangBangController controller1 = new BangBangController();
  private final BangBangController controller2 = new BangBangController();

  public shooter() {
    config.Voltage.PeakForwardVoltage= Constants.kMaxFlywheelVoltage;
    config.Voltage.PeakReverseVoltage= Constants.kMaxFlywheelVoltage;
    config.CurrentLimits.SupplyCurrentLimit = Constants.kMaxFlywheelCurrent;
    motor__init__(topFlyWheel, Constants.top_flywheel_reversed);
    motor__init__(bottomFlyWheel, Constants.bottom_flywheel_reversed);
    feeder.setNeutralMode(NeutralModeValue.Brake);
    feeder.setInverted(false);
    
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
  public void feedIntoShooter(boolean flag){
    double direction = (flag)? 1:-1;
    feeder.set(0.2*direction);
  }
  public void shoot(){
    double velocity= 0;
    topFlyWheel.setVoltage(shooterff.calculate(velocity)+controller1.calculate(topFlyWheel.getVelocity().getValueAsDouble()));
    bottomFlyWheel.setVoltage(shooterff.calculate(velocity)+controller2.calculate(bottomFlyWheel.getVelocity().getValueAsDouble()));
  }
  public void zero(){
    topFlyWheel.set(0);
    bottomFlyWheel.set(0);
  }
  public void shootAmp(){
    topFlyWheel.set(0.08);
    bottomFlyWheel.set(0.1);
  }
  public void zeroFeeder(){
    feeder.set(0);
  }
}
