// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DroidRageConstants;

public class ExampleSubsystem extends SubsystemBase {

    private enum State {
        ONE(1, 1),
        TWO(2, 2),
        THREE(3, 3),
        ;

        // Even though this field is final, it can be changed bc the getters check network table
        private final double first; 
        private final double second;

        private State(double first, double second) {
            this.first = first;
            this.second = second;
        }
    }

    private State state;

    /** Creates a new ExampleSubsystem. */
    public ExampleSubsystem() {}
    
    /**
     * Example command factory method.
     *
     * @return a command
     */
    public CommandBase exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
                /* one-time action goes here */
            });
    }
    
    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public double getFirst() {
        return DroidRageConstants.getNumber("Example/State/"+ state.name(), state.first);
    }

    public double getSecond() {
        return DroidRageConstants.getNumber("Example/State/"+ state.name(), state.second);
    }

    private CommandBase setSpeed(State state) {
        return runOnce(() -> {
            this.state = state;
            DroidRageConstants.putString("Example/Current State", state.name());
        });
    }

    public CommandBase setOne() {
        return setSpeed(State.ONE);
    }

    public CommandBase setTwo() {
        return setSpeed(State.TWO);
    }

    public CommandBase setThree() {
        return setSpeed(State.THREE);
    }
}
