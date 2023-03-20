package frc.robot.subsystems.Pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class PivotMotionProfiled extends Pivot {
    protected TrapezoidProfile profile;
    protected final TrapezoidProfile.Constraints constraints;
    protected TrapezoidProfile.State state;
    protected TrapezoidProfile.State goal;
    
    public PivotMotionProfiled() {
        super();

        pidController.setPID(0, 0, 0);
        pidController.setTolerance(0.10);

        feedforward = new ArmFeedforward(0, 0, 0, 0);

        constraints = new TrapezoidProfile.Constraints(
            0.03, // radians per second // The theorical max velocity 0.104719755 rad/s
            0.03 //radians per second per second
        );

        state = new TrapezoidProfile.State(0, 0);
        goal = new TrapezoidProfile.State(0, 0);
        
        profile = new TrapezoidProfile(constraints, goal, state);
    }

    @Override
    public void periodic() {
        profile = new TrapezoidProfile(constraints, goal, state);
        state = profile.calculate(0.02); // 0.02 taken from TrapezoidProfileSubsystem measured in seconds
        setVoltage(calculateFeedforward(state.position, state.velocity) + calculatePID(state.position));
    }
  
    @Override
    public void simulationPeriodic() {
        periodic();
    }

    @Override
    public void setTargetPosition(double positionRadians) {
        goal = new TrapezoidProfile.State(positionRadians, 0);
    }
}  