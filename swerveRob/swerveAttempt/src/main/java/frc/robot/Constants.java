// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  
  public static final double kPTurning = 10;
  public static final double kPDriving = 0.1;
  public static final double ks = 0.01;
  public static final double kv = 0;
public static final double WHEELRADIUS = 0.21;


// define swerve
public static final int kFrontLeftDriveMotorPort = 6;
public static final int kFrontLeftTurningMotorPort = 31;
public static  boolean kFrontLeftDriveEncoderReversed = true;
public static final boolean kFrontLeftTurningEncoderReversed = false;
public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 1.64;
public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;

public static final int kFrontRightDriveMotorPort = 25;
public static final int kFrontRightTurningMotorPort = 27;
public static  boolean kFrontRightDriveEncoderReversed = true;
public static final boolean kFrontRightTurningEncoderReversed = false;
public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 4.15;
public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;

public static final int kBackLeftDriveMotorPort = 20;
public static final int kBackLeftTurningMotorPort = 23;
public static  boolean kBackLeftDriveEncoderReversed = true;
public static final boolean kBackLeftTurningEncoderReversed = false;
public static final int kBackLeftDriveAbsoluteEncoderPort = 3;
public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 5.16;
public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;

public static final int kBackRightDriveMotorPort = 24;
public static final int kBackRightTurningMotorPort = 26;
public static  boolean kBackRightDriveEncoderReversed = false ;
public static boolean isTurningCCW = false;
public static final boolean kBackRightTurningEncoderReversed = false;
public static final int kBackRightDriveAbsoluteEncoderPort = 2;
public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 1.71+2.15;
public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

public static final double kWheelBase = 0.752475;
public static final double kTrackWidth= 0.76835;

public static final Translation2d leftFrontModule = new Translation2d(kWheelBase / 2, -kTrackWidth / 2);
public static final Translation2d leftBackModule = new Translation2d(kWheelBase / 2, kTrackWidth / 2);
public static final Translation2d rightFrontModule = new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);
public static final Translation2d rightBackModule = new Translation2d(-kWheelBase / 2, kTrackWidth / 2);

public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
  leftFrontModule,
  leftBackModule,
  rightFrontModule,
  rightBackModule);// rb module
public static final double kPhysicalMaxSpeedMetersPerSecond = 8;
public static final double kPThetaController = 0.05;
public static final double kThetaControllerConstraints = 0;
public static final double kPXController = 0.05;
public static final double kPYController = 0.05;
public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 40;
public static final double kDeadband = 0.15;
public static final double kTeleDriveMaxSpeedMetersPerSecond = 4;
public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 6;
public static final double kdriveGearRation = 1/12;
public static final String kCamName = null;

public static final Pose3d kFarTargetPose = new Pose3d();
public static final Transform3d kCameraToRobot = new Transform3d();
public static final Matrix<N3, N1> StandardDev = null;
public static final Matrix<N3, N1> visionStandardDev = null;


//holonomic controllor pid constants.
public static final double xP = 0;
public static final double yP = 0;
public static final double thetaP = 0;

// driving feed forward

//xc
public static final int buttonA = 1;

//pneumatics
public static final int PCM = 2;

//shooter
public static final int top_flywheel = 0;
public static final int bottom_flywheel = 0;
public static final double kMaxFlywheelVoltage = 0;
public static final double kMaxFlywheelCurrent = 0;
public static final boolean top_flywheel_reversed = false;
public static final boolean bottom_flywheel_reversed = false;




}
