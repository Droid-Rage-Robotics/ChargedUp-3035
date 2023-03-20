package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;

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
import frc.robot.utilities.SafeCanSparkMax;
import frc.robot.utilities.ShuffleboardValue;
import frc.robot.utilities.ShuffleboardValueBuilder;

public class Intake extends SubsystemBase {

    protected double SHOOTCUBE = -1,
    OUTTAKE = -0.28,
    CUBE_INTAKE = 0.19,
    CONE_INTAKE = 0.3,
    HOLD = 0.05, STOP = 0;

    protected final SafeCanSparkMax intakeMotor;
    protected final DoubleSolenoid intakeSolenoid;
    protected PneumaticHub pneumaticHub;

    protected static final boolean isOpenDefault = TrackedElement.get() == Element.CUBE ? true : false;
    protected final ShuffleboardValue<Boolean> isOpen = ShuffleboardValue.create(isOpenDefault, "Is Open", Intake.class.getSimpleName()).build();
    protected final ShuffleboardValue<Boolean> compressorEnabledWriter = ShuffleboardValue.create(true, "Compressor Enabled", Intake.class.getSimpleName())
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .build();

    public Intake() {
        intakeMotor = new SafeCanSparkMax(
            19, 
            MotorType.kBrushless,
            ShuffleboardValue.create(true, "Motor Enabled", Intake.class.getSimpleName())
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .build(),
            ShuffleboardValue.create(0.0, "Power", Intake.class.getSimpleName())
                .build()
        );

        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setInverted(false);

        pneumaticHub = new PneumaticHub(10);
        intakeSolenoid = pneumaticHub.makeDoubleSolenoid(9, 11);

        close();
    }

    @Override
    public void periodic() {
        if (!compressorEnabledWriter.get()) pneumaticHub.disableCompressor();
    }
  
    @Override
    public void simulationPeriodic() {}
  
    public void close() {
        intakeSolenoid.set(Value.kForward);//TODO:change
        isOpen.set(false);
        TrackedElement.set(Element.CONE); 
    }

    public void open() {
        intakeSolenoid.set(Value.kReverse);//TODO:change
        isOpen.set(true);
        TrackedElement.set(Element.CUBE);
    }

    public void toggle() {
        if(isOpen.get()) close();
            else open();
    }

    protected void setVoltage(double power) {
        intakeMotor.setVoltage(power * 12);
    }

    public void intake() {
        switch(TrackedElement.get()) {
            case CONE -> setVoltage(CONE_INTAKE);
            case CUBE -> setVoltage(CUBE_INTAKE);
        }
    }

    public void outtake() {
        switch(TrackedElement.get()) {
            case CONE -> setVoltage(OUTTAKE);
            case CUBE -> setVoltage(OUTTAKE);
        }
    }

    public void stop() {
        setVoltage(STOP);
    }

    public void hold() {
        setVoltage(HOLD);
    }

    public CommandBase runIntakeFor(double waitSeconds) {
        return Commands.sequence(
            runOnce(this::intake),
            Commands.waitSeconds(waitSeconds),
            runOnce(this::stop)
        );
    }
    public CommandBase runOuttakeFor(double waitSeconds) {
        return Commands.sequence(
            runOnce(this::outtake),
            Commands.waitSeconds(waitSeconds),
            runOnce(this::stop)
        );
    }
      
}

