package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Intake.IntakeState;

public class UnstuckBall extends SequentialCommandGroup {

    public UnstuckBall() {
        Intake intake = Robot.getIntakeInstance();

        for (int i = 0; i < 5; i++) {
            addCommands(
                Commands.runOnce(() -> intake.setState(IntakeState.Backward), intake)
                        .andThen(Commands.waitSeconds(0.15)),
                Commands.runOnce(() -> intake.setState(IntakeState.Forward), intake)
                        .andThen(Commands.waitSeconds(0.15))
            );
        }
            
        addCommands(Commands.runOnce(() -> intake.setState(IntakeState.Idle), intake));   
    }
}