// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class falcon extends SubsystemBase {
  /** Creates a new falcon. */
 
  
  private final TalonFX motor = new TalonFX(Constants.falcon1);
  
  private TalonFXConfiguration t = new TalonFXConfiguration();
  private double vel = 0;
  private double pose;
  private PIDController controller = new PIDController(Constants.kP, Constants.kI, Constants.kD);
  private double volts = 0;

  public falcon() {
    t.Voltage.PeakForwardVoltage = 12;
    t.Voltage.PeakForwardVoltage = 12;
    var slot0Configs = new Slot0Configs();
    motor.getConfigurator().apply(t);
    updatePID();
  }

  private void updatePID(){
    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = Constants.kV;
    slot0Configs.kP = Constants.kP;
    slot0Configs.kI = Constants.kI;
    slot0Configs.kD = Constants.kD;
    motor.getConfigurator().apply(slot0Configs);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    vel = motor.getVelocity().getValueAsDouble();
    pose = motor.getPosition().getValueAsDouble();
    SmartDashboard.putNumber("velocity", vel);
    SmartDashboard.putNumber("posiotion", pose);
    SmartDashboard.putNumber("P", Constants.kP);
    SmartDashboard.putNumber("I", Constants.kI);
    SmartDashboard.putNumber("D", Constants.kD);
    SmartDashboard.putNumber("V", Constants.kV);
    SmartDashboard.putNumber("Motor Voltage", volts);

    Constants.kP = SmartDashboard.getNumber("get new P", Constants.kP);
    Constants.kI = SmartDashboard.getNumber("get new I", Constants.kI);
    Constants.kD = SmartDashboard.getNumber("get new D", Constants.kD);
    Constants.kV = SmartDashboard.getNumber("get new V", Constants.kV);
  }

  public void runMotor(double speedt){
    volts = controller.calculate(vel, speedt);
    motor.setVoltage(volts);
  }
  public void runMotorPose(double p){
    motor.setVoltage(controller.calculate(pose, p));
  }

  public void zeroMotor(){
    motor.setVoltage(0);
    motor.set(0);
  }
}
