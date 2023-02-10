package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HorizontalExtension extends SubsystemBase {
    protected enum Position {
        INTAKE(0),
        BOTTOM(0),
        MID(0),
        TOP(0),
        ;

        protected final double length_meters;

        private Position(double length_meters) {
            this.length_meters = length_meters;
        }

    }

    private final CANSparkMax elevMotor;
    private final DutyCycleEncoder elevEncoder;
    private final PIDController elevController;
    
    /** Creates a new ExampleSubsystem. */
    public HorizontalExtension() {
        elevMotor = new CANSparkMax(0, MotorType.kBrushless); //TODO: change
        elevEncoder = new DutyCycleEncoder(0);
        elevMotor.setIdleMode(IdleMode.kBrake);
        elevController = new PIDController(0, 0, 0); //TODO: change
        elevController.setTolerance(5);
    } 
   
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Horizontal Encoder Pos", elevEncoder.get());
    }
   
    @Override
    public void simulationPeriodic() {
        periodic();
    }

    private CommandBase move(Position level) {
        return runOnce(() -> elevController.setSetpoint(level.length_meters));
    }
   
    public CommandBase toIntake() {
        return move(Position.INTAKE);
    }
   
    public CommandBase toBottom() {
        return move(Position.BOTTOM);
    }
   
    public CommandBase toMid() {
        return move(Position.MID);
    }
   
    public CommandBase toTop() {
        return move(Position.TOP);
    }
}
