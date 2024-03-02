package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

//import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class SwerveSubsystem extends SubsystemBase {
    //private PhotonCamera cam = new PhotonCamera(Constants.kCamName);
    double sum;
    boolean flagOverDrive;
;
    double normalizedSpeed;
    
    private final SwerveModule frontLeft = new SwerveModule(
            Constants.kFrontLeftDriveMotorPort,
            Constants.kFrontLeftTurningMotorPort,
            Constants.kFrontLeftDriveEncoderReversed,   
            Constants.kFrontLeftTurningEncoderReversed,
            Constants.kFrontLeftDriveAbsoluteEncoderPort,
            Constants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            Constants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            Constants.kFrontRightDriveMotorPort,
            Constants.kFrontRightTurningMotorPort,
            Constants.kFrontRightDriveEncoderReversed,
            Constants.kFrontRightTurningEncoderReversed,
            Constants.kFrontRightDriveAbsoluteEncoderPort,
            Constants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            Constants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            Constants.kBackLeftDriveMotorPort,
            Constants.kBackLeftTurningMotorPort,
            Constants.kBackLeftDriveEncoderReversed,
            Constants.kBackLeftTurningEncoderReversed,
            Constants.kBackLeftDriveAbsoluteEncoderPort,
            Constants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            Constants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            Constants.kBackRightDriveMotorPort,
            Constants.kBackRightTurningMotorPort,
            Constants.kBackRightDriveEncoderReversed,
            Constants.kBackRightTurningEncoderReversed,
            Constants.kBackRightDriveAbsoluteEncoderPort,
            Constants.kBackRightDriveAbsoluteEncoderOffsetRad,
            Constants.kBackRightDriveAbsoluteEncoderReversed);

     private final AHRS gyro = new AHRS(SPI.Port.kMXP);
     private SwerveModulePosition[] swerveModPose= new SwerveModulePosition[]{
        frontLeft.getSwerveModulePosition(),
        backLeft.getSwerveModulePosition(),
        frontRight.getSwerveModulePosition(),
        backRight.getSwerveModulePosition()
     };
    //private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.kDriveKinematics, new Rotation2d(0), swerveModPose);
    private final SwerveDrivePoseEstimator poseEstimator ;
    StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
    private final Pose2d initPose;
     
    
    public SwerveSubsystem(Pose2d initpose) {
        //poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5,0.5,20));
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        this.initPose = initpose;
        poseEstimator = new SwerveDrivePoseEstimator(Constants.kDriveKinematics, new Rotation2d(), swerveModPose, initpose);
        AutoBuilder.configureHolonomic(this::getPose, 
        this::resetOdometry, 
        this::getRelatChassisSpeeds, 
        this::setStatesFromChassisSpeeds, 
        new HolonomicPathFollowerConfig(
            new PIDConstants(5,0,0),
            new PIDConstants(5,0,0),
            2, 
            0.4,
            new ReplanningConfig(false,false)),
        () ->{
            var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
        },  
        this);
        
    }

    public void updateSwerveModPose(){
        this.swerveModPose = new SwerveModulePosition[]{
            frontLeft.getSwerveModulePosition(),
            backLeft.getSwerveModulePosition(),
            frontRight.getSwerveModulePosition(),
            backRight.getSwerveModulePosition()
     };
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
       return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() { 
        //return new Pose2d(new Translation2d(gyro.getDisplacementX(), gyro.getDisplacementY()), getRotation2d());
        Pose2d dis = poseEstimator.getEstimatedPosition();
        return new Pose2d(new Translation2d(dis.getX()+initPose.getX(),dis.getY()+initPose.getY()), dis.getRotation());
    }
    public SwerveModuleState[] getStates(){
        return new SwerveModuleState[]
        {frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState()};
    }

    public void resetOdometry(Pose2d pose) {
        //odometer.resetPosition(getRotation2d(),swerveModPose ,pose);
        poseEstimator.resetPosition(pose.getRotation(), swerveModPose, pose);
    }
    
    public ChassisSpeeds getRelatChassisSpeeds(){
        return Constants.kDriveKinematics.toChassisSpeeds(getStates());
    }

    @Override
    public void periodic() {
        updateSwerveModPose();
        //Logger.recordOutput("MyPose", getPose());
        /*var res = cam.getLatestResult();
        if (res.hasTargets()) {
            var imageCaptureTime = res.getTimestampSeconds();
            var camToTargetTrans = res.getBestTarget().getBestCameraToTarget();
            var camPose = Constants.kFarTargetPose.transformBy(camToTargetTrans.inverse());
            poseEstimator.addVisionMeasurement(
                    camPose.transformBy(Constants.kCameraToRobot).toPose2d(), imageCaptureTime);
        }*/
        
        updateShuffleBoard();
        poseEstimator.update(getRotation2d(), swerveModPose);
        publisher.set(getStates());
    }
    private void updateShuffleBoard(){
        SmartDashboard.putNumber("fl encoder angle", frontLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("fr encoder angle", frontRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("bl encoder angle", backLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("br encoder angle", backRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Gyro", getHeading());
        SmartDashboard.putNumber("fl velcity", frontLeft.getDriveVelocity());
        SmartDashboard.putNumber("fr velocity", frontRight.getDriveVelocity());
        SmartDashboard.putNumber("bl velocity", backLeft.getDriveVelocity());
        SmartDashboard.putNumber("br velocity", backRight.getDriveVelocity());
        SmartDashboard.putNumber("translation x", getPose().getX());
        SmartDashboard.putNumber("translation y", getPose().getY());
    }

    public void stopModules() {
        //frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
    
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond);
    
        //Logger.recordOutput("MyStates", desiredStates);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
    public void setAutoModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond);
    
        //Logger.recordOutput("MyStates", desiredStates);
        frontLeft.setAutoDesiredState(desiredStates[0]);
        frontRight.setAutoDesiredState(desiredStates[1]);
        backLeft.setAutoDesiredState(desiredStates[2]);
        backRight.setAutoDesiredState(desiredStates[3]);
    }
    public void setStatesFromChassisSpeeds(ChassisSpeeds speeds){
        setAutoModuleStates(Constants.kDriveKinematics.toSwerveModuleStates(speeds));
    }
    
}

// might need to make new odometry 
/*
 * use x cos u and y sin u and subtract rotional values from gyro
 * 
 * or just gyro could work. orentaition might just have been wrong before.
 * flip roborio*****
 */