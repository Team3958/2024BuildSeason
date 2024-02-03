package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
     private final TalonFX driveMotor;
     private final TalonFX turningMotor;


     private final ProfiledPIDController con = new ProfiledPIDController(Constants.kPDriving, 0,0, new TrapezoidProfile.Constraints(10, 3));
     private final PIDController turningPidController;
     
     private final AnalogInput absoluteEncoder;
     private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    double driveVolts;
    double turnVolts;
    
    

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new TalonFX(turningMotorId);


        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);
        
        

      

        turningPidController = new PIDController(Constants.kPTurning, 0, 0);
        //velocityPidController = new PIDController(Constants.kPDriving, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
            
        reset_encoders();
    }

    public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble()*Constants.WHEELRADIUS*Constants.kdriveGearRation;
    }

    public double getTurningPosition() {
        return turningMotor.getPosition().getValueAsDouble()*2*Math.PI;
    }

    public double getDriveVelocity() {

        return driveMotor.getVelocity().getValueAsDouble()*Constants.WHEELRADIUS*Constants.kdriveGearRation;
    }

    public double getTurningVelocity() {
        return turningMotor.getVelocity().getValueAsDouble()*2*Math.PI;
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    
    public void reset_encoders(){
        driveMotor.setPosition(0);
        turningMotor.setPosition(getAbsoluteEncoderRad());
    }
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }
    public SwerveModulePosition getSwerveModulePosition(){
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAbsoluteEncoderRad()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveVolts = con.calculate(getDriveVelocity()*2, state.speedMetersPerSecond );
        turnVolts = turningPidController.calculate(getTurningPosition(), state.angle.getRadians());
        driveMotor.setVoltage(driveVolts); // double check rotor ratio
        turningMotor.setVoltage(turnVolts);
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
        
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

}
