package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DroidRageConstants;
import frc.robot.utilities.ComplexWidgetBuilder;
import frc.robot.utilities.MutableDouble;
import frc.robot.utilities.WriteOnlyDouble;

public class Pivot extends SubsystemBase {
    public static class Constants {
        public static final double GEAR_RATIO = 1 / 1;
        public static final double READINGS_PER_REVOLUTION = 1;
        public static final double ROTATIONS_TO_DEGREES = (GEAR_RATIO * READINGS_PER_REVOLUTION / 360);
    }

    private enum ArmPos {
        START(0),
        INTAKELOW(0),//TODO:Change

        LOWCONE(0),
        LOWCUBE(0),

        MIDCONE(0),
        MIDCUBE(0),

        HIGHCONE(0),
        HIGHCUBE(0),

        INTAKEHIGH(0),
        HOLD(0),
        ;

        private final MutableDouble degrees;

        private ArmPos(double degrees) {
            this.degrees = new MutableDouble(degrees, ArmPos.class.getSimpleName()+"/"+name()+" (Degrees)", Pivot.class.getSimpleName());
        }
    }
    private final CANSparkMax pivotMotor;
    private final PIDController controller;
    private volatile ArmPos armPos = ArmPos.START;
    private final AbsoluteEncoder pivotAbsoluteEncoder;

    private final WriteOnlyDouble targetPositionWriter = new WriteOnlyDouble(0, "Target Position (Degrees)", Pivot.class.getSimpleName());
    private final WriteOnlyDouble encoderPositionWriter = new WriteOnlyDouble(0, "Encoder Position (Degrees)", Pivot.class.getSimpleName());
    
    public Pivot() {
        pivotMotor = new CANSparkMax(18, MotorType.kBrushless);//TODO: Where is it plugged in?

        pivotMotor.setIdleMode(IdleMode.kBrake);

        pivotAbsoluteEncoder = pivotMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle); //TODO:Test // 8192
        pivotAbsoluteEncoder.setPositionConversionFactor(Constants.ROTATIONS_TO_DEGREES);
        // armAbsoluteEncoder.setInverted(false);
        // armMotor.getForwardLimitSwitch(null);//What does this do
  

        controller = new PIDController(0, 0, 0);
        controller.setTolerance(0.10); // meters

        new ComplexWidgetBuilder(controller, "PID Controller", Pivot.class.getSimpleName())
            .withWidget(BuiltInWidgets.kPIDController);

        setTargetPosition(ArmPos.START);
    }

    @Override
    public void periodic() {
        
    }
  
    @Override
    public void simulationPeriodic() {
        periodic();
    }

    public double getTargetPosition() {
        return armPos.degrees.get();
    }

    private CommandBase setTargetPosition(ArmPos position) {
        return runOnce(() -> {
            armPos = position;
            targetPositionWriter.set(position.degrees.get());
        });
    }
    public CommandBase setTargetPosition(double position) {
        return runOnce(() -> {
            controller.setSetpoint(position);
            targetPositionWriter.set(position);
        });
    }

    public CommandBase moveIntakeLow() {
        return setTargetPosition(ArmPos.INTAKELOW);
    }
    public CommandBase moveIntakeHigh() {
        return setTargetPosition(ArmPos.INTAKEHIGH);
    }
    
    public CommandBase moveLow() {
        return setTargetPosition(
            switch(TrackedElement.get()) {
                case CONE -> ArmPos.LOWCONE;
                case CUBE -> ArmPos.LOWCUBE;
                case NONE -> ArmPos.LOWCUBE;
            }
        );
    }

    public CommandBase moveMid() {
        return setTargetPosition(
            switch(TrackedElement.get()) {
                case CONE -> ArmPos.MIDCONE;
                case CUBE -> ArmPos.MIDCUBE;
                case NONE -> ArmPos.MIDCUBE;
            }
        );
    }

    public CommandBase moveHigh() {
        return setTargetPosition(
            switch(TrackedElement.get()) {
                case CONE -> ArmPos.HIGHCONE;
                case CUBE -> ArmPos.HIGHCUBE;
                case NONE -> ArmPos.HIGHCUBE;
            }
        );
    }

    public CommandBase moveHold() {
        return setTargetPosition(ArmPos.HOLD);
    }

    // public CommandBase moveToPosition() {
    //     return runOnce(() -> pivotMotor.set(controller.calculate(getTargetPosition())));
    // }
}  
