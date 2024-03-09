package frc.robot;


import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;

import org.opencv.core.Mat;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.zeroHeading;
import frc.robot.commands.climber.climbRetrack;
import frc.robot.commands.climber.climberUP;
import frc.robot.commands.driving.drivingCommand;
import frc.robot.commands.driving.resetEncodersCommnad;
import frc.robot.commands.shooter.Intake;
import frc.robot.commands.shooter.extake;
import frc.robot.commands.shooter.fullshootercommand;
import frc.robot.commands.shooter.shootSpeaker;
import frc.robot.subsystems.PDPSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.intakeSub;
import frc.robot.subsystems.pneumaticSubsystem;
import frc.robot.subsystems.shooter;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final PDPSubsystem m_pdp = new PDPSubsystem();
    private final XboxController xc = new XboxController(0);
    private final intakeSub m_intake = new intakeSub();
   // private final XboxController xc2 = new XboxController(1);
    private final shooter m_Shooter = new shooter();
    //private SwerveControllerCommand controllerCommand;
     //private final pneumaticSubsystem m_PneumaticSubsystem = new pneumaticSubsystem();
    
   // private SequentialCommandGroup main = new SequentialCommandGroup(new shootSpeaker(m_Shooter), fullShoot);
    //PathPlannerAuto N = new PathPlannerAuto("New Auto");
    //PathfindThenFollowPathHolonomic findPath;
    Pose2d startPose;

    //private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // known way to follow paths
        /*PIDController xController = new PIDController(Constants.xP, 0, 0);
        PIDController yController = new PIDController(Constants.yP, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(Constants.thetaP, 0, 0, new TrapezoidProfile.Constraints(2*Math.PI,2*Math.PI));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        HolonomicDriveController hController = new HolonomicDriveController(xController, yController, thetaController);
        TrajectoryConfig config = new TrajectoryConfig(4, 4);
        edu.wpi.first.math.trajectory.Trajectory path = TrajectoryGenerator.generateTrajectory( new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        new Pose2d(3, 0, new Rotation2d(0)),
        config);*/
       // controllerCommand = new SwerveControllerCommand(path, () ->swerveSubsystem.getPose(), Constants.kDriveKinematics, hController, swerveSubsystem::setModuleStates, swerveSubsystem);
        
        
        // set swerve drive
        
        
        swerveSubsystem.setDefaultCommand(
        new drivingCommand(
            swerveSubsystem,
            () -> xc.getLeftY(),
            () -> xc.getLeftX(),
            () -> xc.getRightX(),
            () -> !xc.getYButton()));
        
        
        configureButtonBindings();
        registerCommands();
        
    
    //findPath = new PathfindThenFollowPathHolonomic(null, null, null, null, null, null, null, null);
        
    }
    private void registerCommands(){
       // NamedCommands.registerCommand("zero", new zeroHeading(swerveSubsystem));
       //NamedCommands.registerCommand("shoot", main);
    }

    private void configureButtonBindings() {
        new JoystickButton(xc, Constants.buttonA).onTrue(new zeroHeading(swerveSubsystem));
        //new JoystickButton(xc2, Constants.buttonB).toggleOnTrue(new climberUP(m_PneumaticSubsystem));
        //new JoystickButton(xc2, Constants.buttonX).toggleOnTrue(new climbRetrack(m_PneumaticSubsystem));
        new JoystickButton(xc, Constants.buttonY).whileTrue(new fullshootercommand(m_Shooter, m_intake));
        new JoystickButton(xc, Constants.buttonLB).whileTrue(new Intake(m_intake));
        new JoystickButton(xc, Constants.buttonRB).whileTrue(new extake(m_intake));
        new JoystickButton(xc, Constants.buttonStart).whileTrue(new resetEncodersCommnad(swerveSubsystem));
    }

    public Command getAutonomousCommand() {
        //return new shootSpeaker(m_Shooter);
        swerveSubsystem.zeroHeading();
        return null;
    }
}