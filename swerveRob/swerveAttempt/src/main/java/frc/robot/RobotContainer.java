package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drivingCommand;
import frc.robot.commands.zeroHeading;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final XboxController xc = new XboxController(0);

    //private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new drivingCommand(swerveSubsystem,
        () -> xc.getLeftX(),
        () -> xc.getLeftY(),
        () -> xc.getRightX(),
        () -> !xc.getYButton()));
                

        configureButtonBindings();
        //autoChooser = AutoBuilder.buildAutoChooser();
        //AutoBuilder.followPath(PathPlannerPath.fromPathFile("New Path")).schedule();
        
    }

    private void configureButtonBindings() {
        new JoystickButton(xc, 2).onTrue(new zeroHeading(swerveSubsystem));
    }

    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        return null;
    }
}