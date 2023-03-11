package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ComplexWidgetBuilder;
import frc.robot.utilities.MutableBoolean;
import frc.robot.utilities.MutableDouble;
import frc.robot.utilities.SimpleWidgetBuilder;
import frc.robot.utilities.WriteOnlyDouble;

public class Pivot2 extends SubsystemBase {
    public static class Constants {
        public static final double GEAR_RATIO = 3 / 1;
        public static final double READINGS_PER_REVOLUTION = 1;
        public static final double ROTATIONS_TO_DEGREES = (GEAR_RATIO * READINGS_PER_REVOLUTION) / 360;
    }

    public enum PivotPosition {
        //170 - straight
        START(65),
        INTAKELOWCUBE(90),//TODO:Change
        INTAKELOWCONE(130),

        LOWCONE(226),
        LOWCUBE(226),

        // MIDCONE(LOWCONE.degrees.get()),
        MIDCONE(100),
        MIDCUBE(100),

        HIGHCONE(110),
        HIGHCUBE(110),

        INTAKEHIGHCUBE(30), //parallel
        INTAKEHIGHCONE(30) ,
        HOLD(3), // straight up

        //AUTO
        INTAKEAUTOCUBE(50), //Intaking as far down as possible for dube
        INTAKEAUTOCONE(50), //intake auto position for cone

        FORRWARDSOFTLIMIT(60),
        UP(71)



        // START(0),
        // INTAKELOWCUBE(53),//TODO:Change
        // INTAKELOWCONE(56),

        // LOWCONE(40.8),
        // LOWCUBE(43.7),

        // MIDCONE(LOWCONE.degrees.get()),
        // MIDCUBE(45),

        // HIGHCONE(LOWCONE.degrees.get()),
        // HIGHCUBE(30),

        // INTAKEHIGHCUBE(43), //parallel
        // INTAKEHIGHCONE(43),
        // HOLD(0), // straight up

        // //AUTO
        // INTAKEAUTOCUBE(49.7), //Intaking as far down as possible for dube
        // INTAKEAUTOCONE(50), //intake auto position for cone

        // FORRWARDSOFTLIMIT(60)
        ;

        private final MutableDouble degrees;

        private PivotPosition(double degrees) {
            this.degrees = new MutableDouble(degrees, PivotPosition.class.getSimpleName()+"/"+name()+" (Degrees)", Pivot2.class.getSimpleName());
        }
    }
    private final CANSparkMax pivotMotor;
    private final PIDController motorController, pivotController;
    private final SparkMaxAbsoluteEncoder pivotAbsoluteEncoder;
    
    private final RelativeEncoder motorRelativeEncoder;
    private volatile PivotPosition position = PivotPosition.START;
    // private final AbsoluteEncoder pivotAbsoluteEncoder;
    // private final RelativeEncoder pivotRelativeEncoder;
    

    // private final WriteOnlyDouble targetPositionWriter = new WriteOnlyDouble(0, "Target Position (Degrees)", Pivot.class.getSimpleName());
    private final WriteOnlyDouble absEncoderPositionWriter = new WriteOnlyDouble(0, "Absolute Encoder Position (Degrees)", Pivot2.class.getSimpleName());
    private final WriteOnlyDouble motorEncoderPositionWriter = new WriteOnlyDouble(0, "Motor Encoder Position (Degrees)", Pivot2.class.getSimpleName());

    private final MutableBoolean isEnabled = new SimpleWidgetBuilder<Boolean>(false, "Is Enabled", Pivot2.class.getSimpleName())
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .buildMutableBoolean();

    private double offset = 0;
    
    public Pivot2() {
        pivotMotor = new CANSparkMax(18, MotorType.kBrushless);
        pivotMotor.setIdleMode(IdleMode.kCoast);//TODO:make brake mode

        motorRelativeEncoder = pivotMotor.getEncoder();
        // motorRelativeEncoder.
        pivotAbsoluteEncoder = pivotMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle); //TODO:Test // 8192
        pivotAbsoluteEncoder.setPositionConversionFactor(360);
        pivotAbsoluteEncoder.setInverted(false);//TODO:Direction?

        // armMotor.getForwardLimitSwitch(null);//What does this do
  

        motorController = new PIDController(0.005, 0, 0); //0.023
        motorController.setTolerance(0.10); // degress
        pivotController = new PIDController(0.1, 0, 0);
        pivotController.setTolerance(1); // degress
        pivotController.setSetpoint(PivotPosition.START.degrees.get());

        new ComplexWidgetBuilder(motorController, "Motor PID Controller", Pivot2.class.getSimpleName())
            .withWidget(BuiltInWidgets.kPIDController)
            .withSize(2, 2);

            new ComplexWidgetBuilder(pivotController, "Pivot PID Controller", Pivot2.class.getSimpleName())
            .withWidget(BuiltInWidgets.kPIDController)
            .withSize(2, 2);

        // setTargetPosition(PivotPosition.START);

        // new ComplexWidgetBuilder(resetPivotEncoder(), "Reset claw encoder", Pivot.class.getSimpleName());
        // pivotMotor.setSoftLimit(SoftLimitDirection.kForward, PivotPosition.FORRWARDSOFTLIMIT);
        // pivotMotor.setSoftLimit(SoftLimitDirection.kReverse, PivotPosition.START);//TODO:Test
    }

    @Override
    public void periodic() {
        // setPower(pivotController.calculate(getAbsPosition()));

        motorController.setSetpoint(pivotController.calculate(getAbsPosition()));

        setPower(motorController.calculate(getMotorPosition()));

    }
  
    @Override
    public void simulationPeriodic() {
        periodic();
    }


    public double getAbsPosition() {
        // return pivotRelativeEncoder.getPosition() + offset;
        absEncoderPositionWriter.set(pivotAbsoluteEncoder.getPosition());
        return pivotAbsoluteEncoder.getPosition(); //PLus 5, cause we can't offset to anything but 0, and sometimed encoder goes to 359, causin pivot to reverse
    }

    public double getMotorPosition() {
        motorEncoderPositionWriter.set(motorRelativeEncoder.getPosition());
        return motorRelativeEncoder.getPosition();
        // return pivotAbsoluteEncoder.getPosition();
    }

    public double getTargetPosition() {
        return pivotController.getSetpoint();
    }

    private CommandBase setTargetPosition(PivotPosition position) {
        return runOnce(() -> {
            this.position = position;
            pivotController.setSetpoint(position.degrees.get());
        });
    }

    private void setTargetPosition(double position) {

            pivotController.setSetpoint(position);
    }
    public void setCurrentPositionManually(double position) {
            setTargetPosition(getTargetPosition() + position);
    }

    // public CommandBase setTargetPosition(double position) {
    //     return runOnce(() -> {
    //         this.position.degrees.set(position);
    //         controller.setSetpoint(position);
    //     });
    // }

    public CommandBase moveIntakeLow() {
        return setTargetPosition(
            switch(TrackedElement.get()) {
                case CONE -> PivotPosition.INTAKELOWCONE;
                case CUBE -> PivotPosition.INTAKELOWCUBE;
                case NONE -> PivotPosition.INTAKELOWCUBE;
            }
        );
    }
    public CommandBase moveIntakeHigh() {
        return setTargetPosition(
            switch(TrackedElement.get()) {
                case CONE -> PivotPosition.INTAKEHIGHCONE;
                case CUBE -> PivotPosition.INTAKEHIGHCUBE;
                case NONE -> PivotPosition.INTAKEHIGHCUBE;
            }
        );
    }
    
    public CommandBase moveLow() {
        return setTargetPosition(
            switch(TrackedElement.get()) {
                case CONE -> PivotPosition.LOWCONE;
                case CUBE -> PivotPosition.LOWCUBE;
                case NONE -> PivotPosition.LOWCUBE;
            }
        );
    }

    public CommandBase moveMid() {
        return setTargetPosition(
            switch(TrackedElement.get()) {
                case CONE -> PivotPosition.MIDCONE;
                case CUBE -> PivotPosition.MIDCUBE;
                case NONE -> PivotPosition.MIDCUBE;
            }
        );
    }

    public CommandBase moveHigh() {
        return setTargetPosition(
            switch(TrackedElement.get()) {
                case CONE -> PivotPosition.HIGHCONE;
                case CUBE -> PivotPosition.HIGHCUBE;
                case NONE -> PivotPosition.HIGHCUBE;
            }
        );
    }

    public CommandBase moveHold() {
        return setTargetPosition(PivotPosition.HOLD);
    }

    private void setPower(double power) {
        // if (!isEnabled.get()) return;
        pivotMotor.set(power);
    }

    // public CommandBase resetPivotEncoder() {
    //     return runOnce(() -> {
    //         offset = -getRawPosition();
    //     });
    // }

    public CommandBase setPowerC(double power) {
        return runOnce(() ->setPower(power));
    }
}  
