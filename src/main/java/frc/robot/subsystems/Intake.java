package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.EnumPositions.TrackedElement;
import frc.robot.subsystems.EnumPositions.TrackedElement.Element;
import frc.robot.utilities.ShuffleboardValue;
import frc.robot.utilities.ShuffleboardValueBuilder;

public class Intake extends SubsystemBase {

    private double SHOOTCUBE = -1,
    OUTTAKE = -0.28,
    CUBEINTAKE = 0.19,
    INTAKE = 0.3,
    HOLD = 0.05, STOP = 0;

    private final CANSparkMax intakeMotor;
    private final RelativeEncoder intakeEncoder;
    private final DoubleSolenoid intakeSolenoid;
    private PneumaticHub pneumaticHub;

    private final ShuffleboardValue<Boolean> isEnabled = ShuffleboardValue.create
        (true, "Is Enabled", Intake.class.getSimpleName())
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .build();
            
    private final ShuffleboardValue<Double> targetVelocityWriter = ShuffleboardValue.create
        (0.0, "Target Intake Velocity", "Intake").build();
    private boolean isOpen = TrackedElement.get() == Element.CUBE ? true : false;

    public Intake() {
        intakeMotor = new CANSparkMax(19, MotorType.kBrushless);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setInverted(false);
        intakeEncoder = intakeMotor.getEncoder();


        pneumaticHub = new PneumaticHub(10);
        intakeSolenoid = pneumaticHub.makeDoubleSolenoid(9, 11);

        close();
    }

    @Override
    public void periodic() {
    }
  
    @Override
    public void simulationPeriodic() {}
  
    public void close() {
        if (!isEnabled.get()) return;
        intakeSolenoid.set(Value.kForward);//TODO:change
        isOpen = false;
        TrackedElement.set(Element.CONE); 
    }

    public void open() {
        if (!isEnabled.get()) return;
        intakeSolenoid.set(Value.kReverse);//TODO:change
        isOpen = true;
        TrackedElement.set(Element.CUBE);
    }
  
    public CommandBase toggleCommand() {
        return runOnce(() -> {
            if(isOpen) close();
            else open();
        });
    }

    public CommandBase runClose() {
        return runOnce(() -> close());//Cone
    }

    public CommandBase runOpen() {
        return runOnce(() -> open());//Cube
    }

    public void intake() {
        switch(TrackedElement.get()) {
            case CONE -> setIntakePower(INTAKE);
            case CUBE -> setIntakePower(CUBEINTAKE); ////////////////////
        }
    }

    public CommandBase runIntake() {
        return
                switch(TrackedElement.get()) {
                case CONE -> setIntakePower(INTAKE);
                case CUBE -> setIntakePower(CUBEINTAKE); ////////////////////
            };
    }

    public CommandBase runOuttake() {
        return
                switch(TrackedElement.get()) {
                case CONE -> setIntakePower(OUTTAKE);
                case CUBE -> setIntakePower(OUTTAKE);
            };
    }

    public CommandBase runStop() { 
        return setTargetVelocity(STOP);
    }

    public CommandBase runHoldIntake() { 
        return switch(TrackedElement.get()) {
            case CONE -> setIntakePower(HOLD);
            case CUBE -> setIntakePower(HOLD);
            
        };
    }

    public CommandBase runIntakeFor(double wait) {
        return Commands.sequence(
            runIntake(),
            Commands.waitSeconds(wait),
            runStop()
        );
    }
    public CommandBase runOuttakeFor(double wait) {
        return Commands.sequence(
            runOuttake(),
            Commands.waitSeconds(wait),
            runStop()
        );
    }

    private CommandBase setTargetVelocity(double power) {
        return runOnce(() -> {
            intakeMotor.set(power);
        });
    }

    public double getIntakeVelocity() {
        return intakeEncoder.getVelocity();
    }
      
    private CommandBase setIntakePower(double power) {

        return runOnce(() -> {
            intakeMotor.set(power);
        });
        
    }

    private void setIntakePowerV(double power) {

            intakeMotor.set(power);
        
    }
}

