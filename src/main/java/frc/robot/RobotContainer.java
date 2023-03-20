package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.Position;
import frc.robot.subsystems.Intake.IntakeWithPID;
import frc.robot.subsystems.Pivot.Pivot;
import frc.robot.utilities.ComplexWidgetBuilder;
import frc.robot.commands.Autos;
import frc.robot.commands.Drive.SwerveDriveTeleop;
import frc.robot.commands.Manual.ManualPivot;
import frc.robot.commands.Manual.ToggleIntake;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    //TODO: Ideas
    // all abort button
    // override button for when sensors fail
    // read about shuffleboard and do great things
    // find out how to make a field graph with robot pose on it and preferably a visual for path following
    // custom shuffleboard droid rage theme
    // Consider setitng drive motor to break and turn to coast
    // ----Bright Color on shuffleboard when block is detected
    // ----path weaver vs path planner
    // reset network tables button
    // auto align 
    // fixnetowrk tables!!!
    // recalibrate pigeon on start
    // why are swerves not simplifying turns
    //add Physical and digital limit switch
    // make shuffleboard stuff able to use a class instead of a string
    // safe sparkmax
    // arm class
    // FIX OUTTAKE COMMAND!!!!!!!!!!!!!!!!!!!
    // fix auto balace
    // fix lock wheels



    // limielite
    // 

    private final Drive drive = new Drive();
    private final Elevator elevator = new Elevator();
    // private final Pivot2 pivot = new Pivot2(); 
    private final Pivot pivot = new Pivot();
    // private final Intake intake = new Intake();
    private final IntakeWithPID intake = new IntakeWithPID();
    private final Arm arm = new Arm(elevator, pivot);

    private final CommandXboxController driver =
        new CommandXboxController(DroidRageConstants.Gamepad.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operator =
        new CommandXboxController(DroidRageConstants.Gamepad.OPERATOR_CONTROLLER_PORT);

    SendableChooser<CommandBase> autoChooser = new SendableChooser<CommandBase>();

    public RobotContainer() {
        autoChooser.addOption("PreloadPlusPark", Autos.preloadPlusPark(drive, arm, intake));
        autoChooser.addOption("Straight Test", Autos.straightTest(drive));
        autoChooser.addOption("Straight Test BACK", Autos.straightTestBack(drive));
        autoChooser.addOption("One: CUbe + drop", Autos.oneToCubeAndToDrop(drive, arm, intake));
        autoChooser.addOption("three: CUbe + drop", Autos.threeToCubeAndToDrop(drive, arm, intake));
        autoChooser.addOption("Turn Test", Autos.turnTest(drive));
        autoChooser.addOption("Charge", Autos.charge(drive, arm, intake));

        ComplexWidgetBuilder.create(autoChooser, "Auto Chooser", "Misc")
            .withSize(1, 3);

        ComplexWidgetBuilder.create(CameraServer.startAutomaticCapture(), "USB Camera Stream", "Misc")
            .withSize(5, 5);
    }

    public void configureTeleOpBindings() {
         /*
         * Driver Controls
         */

        drive.setDefaultCommand(
            new SwerveDriveTeleop(
                drive, 
                driver::getLeftX, 
                driver::getLeftY, 
                driver::getRightX,
                driver.x()
                )
            );

        // driver.rightBumper()
        //     .onTrue(drive.setSlowSpeed())
        //     .onFalse(drive.setNormalSpeed());


        //     driver.rightTrigger()
        //     .onTrue(pivot.setPower(0.6)) 
        //     .onFalse(pivot.setPower(0));

        // driver.leftTrigger()
        //     .onTrue(pivot.setPower(-0.6))
        //     .onFalse(pivot.setPower(0));
        driver.rightTrigger()
            .onTrue(intake.run(intake::intake)) 
            .onFalse(intake.run(intake::stop));

        driver.leftTrigger()
            .onTrue(intake.run(intake::outtake))
            .onFalse(intake.run(intake::stop));

        driver.a()
            .onTrue(drive.resetOffsetCommand());

        driver.b()
            .onTrue(
                // new MoveToPosition(elevator, pivot, intake)
                // Commands.sequence(
                    // intake.toggleCommand()
                    new ToggleIntake(arm, intake)
                    
                // )
            ); 

        driver.back()
            .onTrue(drive.toggleFieldOriented()
            );
        
        driver.y()
            .onTrue(drive.toggleBreakMode());

        // driver.x()
        //     .onTrue(new LockWheels(drive));


        /*
         * Operator Controls
         */
        pivot.setDefaultCommand(new ManualPivot(operator::getLeftY, pivot));
        // elevator.setDefaultCommand(new ManualElevator(operator::getRightX, operator::getRightY, elevator));

        operator.a()
            .onTrue(
                arm.setPositionCommand(Position.LOW)
            );
        operator.x()
            .onTrue(
                arm.setPositionCommand(Position.MID)
            );
        operator.y()
            .onTrue(
                arm.setPositionCommand(Position.HIGH)
            );
        
        operator.povUp()
            .onTrue(
                arm.setPositionCommand(Position.INTAKE_HIGH_1)
            );
        operator.povRight()
            .onTrue(
                arm.setPositionCommand(Position.INTAKE_HIGH_2)
            );
        operator.povLeft()
            .onTrue(
                arm.setPositionCommand(Position.INTAKE_LOW)
            );

            
        operator.povDown()
            .onTrue(

                arm.setPositionCommand(Position.HOLD)
            );

        operator.start()
            .onTrue(elevator.runOnce(elevator::resetEncoders));
      
      
      
        operator.back()
            .onTrue(pivot.runOnce(pivot::resetEncoder)); //Pivot is Absolute(not)
    }

    public void configureTestBindings(){}
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
