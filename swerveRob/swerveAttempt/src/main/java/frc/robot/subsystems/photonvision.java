// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class photonvision extends SubsystemBase {
  /** Creates a new photonvision. */
  double pitch;
  Transform3d pose;
  double yaw;
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  
  public photonvision() {
    PortForwarder.add(5800, "photonvision.local", 5800);
    
    Thread m_visonThread = new Thread(
    () ->{
    UsbCamera cam = CameraServer.startAutomaticCapture();
    
    cam.setResolution(320, 240);
    CvSource outputstream = CameraServer.putVideo("Rectangle", 320, 240);
    CvSink cvSink = CameraServer.getVideo();
    Mat mat = new Mat();
      while (!Thread.interrupted()){
      if (cvSink.grabFrame(mat) ==0){
        outputstream.notifyError("something wrong man");
        continue;
      }
      Imgproc.rectangle(mat, new Rect(), new Scalar(255,255,255),5);
    }
    outputstream.putFrame(mat);
  });
  m_visonThread.setDaemon(true);
  m_visonThread.start();
  }
  PhotonCamera camera = new PhotonCamera("photonvision");
  //PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera,Constants.kCameraToRobot);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result = camera.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    pitch = target.getPitch();
    pose = target.getBestCameraToTarget();
    yaw = target.getYaw();
    updateShuffleBoard();
  
  }
  /* public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }*/
  private void updateShuffleBoard(){
    SmartDashboard.putNumber("pitch", pitch);
    SmartDashboard.putNumber("pose x", pose.getX());
    SmartDashboard.putNumber("pose y", pose.getY());
    SmartDashboard.putNumber("pose z", pose.getZ());
  }
  public double getX(){
    return pose.getX();
  }
  public double gety(){
    return pose.getY();
  }
  public double getTheta(){
    return yaw;
  }
}
