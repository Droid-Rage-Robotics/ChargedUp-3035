package frc.robot.subsystem.intake;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystem.TrackedElement;
import frc.robot.subsystem.TrackedElement.Element;
import frc.robot.utility.SafeCanSparkMax;
import frc.robot.utility.ShuffleboardValue;

public class Intake extends SubsystemBase {

    protected double SHOOTCUBE = -1,
    OUTTAKE = -0.18,//-0.28
    //-0.18 shoot cones
    CUBE_INTAKE = 0.19,
    CONE_INTAKE = 0.3,
    HOLD = 0.05, STOP = 0,
    VOLTAGETHRESHOLD = 13;//TODO: Test the 
    //Input Voltage (nominal): 12V
    //Absolute Maximum Voltage: 30V

    protected final SafeCanSparkMax intakeMotor;
    protected final DoubleSolenoid intakeSolenoid;
    protected PneumaticHub pneumaticHub;

    protected static final boolean isOpenDefault = TrackedElement.get() == Element.CUBE ? true : false;
    protected final ShuffleboardValue<Boolean> isOpen = ShuffleboardValue.create(isOpenDefault, "Is Open", Intake.class.getSimpleName()).build();
    protected final ShuffleboardValue<Boolean> compressorEnabledWriter = ShuffleboardValue.create(true, "Compressor Enabled", Intake.class.getSimpleName())
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .withSize(1, 2)
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
        if (!compressorEnabledWriter.get()) pneumaticHub.disableCompressor(); // Keep in mind the compressor cannot be reenabled until class is reinitialized becuase there is no enableCompressor() method on PneumaticHub
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

    protected void setPower(double power) {
        intakeMotor.set(power);
    }

    protected void setVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    public void intake() {
        switch(TrackedElement.get()) {
            case CONE -> setPower(CONE_INTAKE);
            case CUBE -> setPower(CUBE_INTAKE);
        }
    }

    public void outtake() {
        switch(TrackedElement.get()) {
            case CONE -> setPower(OUTTAKE);
            case CUBE -> setPower(OUTTAKE);
        }
    }

    public void stop() {
        setPower(STOP);
    }

    public void hold() {
        setPower(HOLD);
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

