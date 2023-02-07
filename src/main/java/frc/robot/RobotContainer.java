// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.HorizontalExtension;
import frc.robot.subsystems.VerticalExtension;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Drive.SlowSwerveDriveTeleop;
import frc.robot.commands.Drive.SwerveDriveTeleop;
import frc.robot.commands.Elevator.Ground;
import frc.robot.commands.Elevator.High;
import frc.robot.commands.Elevator.IntakeGround;
import frc.robot.commands.Elevator.IntakeHigh;
import frc.robot.commands.Elevator.Mid;

import java.util.List;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    // private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
    private final Drive drive;
    // private final HorizontalExtension hExtension;
    // private final VerticalExtension vExtension;
    
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driver =
        new CommandXboxController(DroidRageConstants.Gamepad.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operator =
        new CommandXboxController(DroidRageConstants.Gamepad.OPERATOR_CONTROLLER_PORT);
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        
        drive = new Drive();
        // hExtension = new HorizontalExtension();
        // vExtension = new VerticalExtension();
      
      
        
        configureBindings();
        
    }
    
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        // new Trigger(exampleSubsystem::exampleCondition)
            // .onTrue(new ExampleCommand(exampleSubsystem));
    
        
        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // // cancelling on release.
        // driver.b().onTrue(drive.recalibrateHeading());
        // driver.a().onTrue(drive.toggleFieldOriented());
        // driver.rightBumper().onTrue(new SlowSwerveDriveTeleop(drive, driver, drive::isFieldOriented, 0.5));
        // drive.setDefaultCommand(new SlowSwerveDriveTeleop(drive, driver, drive::isFieldOriented, 1));
    
        driver.a().onTrue(drive.runResetHeading());
    
        drive.setDefaultCommand(new SwerveDriveTeleop(
            drive, 
            driver::getLeftX, 
            driver::getLeftY, 
            driver::getRightX,
            drive::isFieldOriented)
        );
    
    
        // operator.rightTrigger().whileTrue(new IntakeGround(hExtension, vExtension));
        // operator.leftTrigger().whileTrue(new IntakeHigh(hExtension, vExtension));
        // operator.a().whileTrue(new Ground(hExtension, vExtension));
        // operator.x().whileTrue(new Mid(hExtension, vExtension));
        // operator.y().whileTrue(new High(hExtension, vExtension));
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        // return Autos.exampleAuto(exampleSubsystem);
        
        // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        //   Drive.Constants.Auto.MAX_SPEED_METERS_PER_SECOND, 
        //   Drive.Constants.Auto.MAX_ACCELERATION_METERS_PER_SECOND)
        //     .setKinematics(Drive.Constants.DRIVE_KINEMATICS);
        
        // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        //   new Pose2d(),
        //   List.of(
        //     new Translation2d(1,0),
        //     new Translation2d(1, -1)
        //   ), 
        //   new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
        //   trajectoryConfig
        //   );
        
        // PIDController xController = new PIDController(Drive.Constants.Auto.TRANSLATIONAL_KP, 0, 0);
        // PIDController yController = new PIDController(Drive.Constants.Auto.TRANSLATIONAL_KP, 0, 0);
        // ProfiledPIDController thetaController = new ProfiledPIDController(
        //   Drive.Constants.Auto.THETA_KP, 
        //   0, 
        //   0, 
        //   Drive.Constants.Auto.THETA_CONSTRAINTS);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        //   trajectory,
        //   drive::getPose,
        //   Drive.Constants.DRIVE_KINEMATICS,
        //   xController,
        //   yController,
        //   thetaController,
        //   drive::setModuleStates,
        //   drive
        // );
        
        
        // return new SequentialCommandGroup(
        //   new InstantCommand(() -> drive.resetOdometry(trajectory.getInitialPose())),
        //   swerveControllerCommand,
        //   new InstantCommand(() -> {
        //       drive.stop();
        //       xController.close();
        //       yController.close();
        //     }
        //   )
        // );
        return new InstantCommand();
    }
}
