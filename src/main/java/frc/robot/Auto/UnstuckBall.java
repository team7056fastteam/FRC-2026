package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Intake.IntakeState;

public class UnstuckBall extends SequentialCommandGroup {

    public UnstuckBall() {
        Intake intake = Robot.getIntakeInstance();

        addCommands(
            // 1
            Commands.runOnce(() -> intake.setState(IntakeState.Backward), intake)
                    .andThen(Commands.waitSeconds(0.15)),
            Commands.runOnce(() -> intake.setState(IntakeState.Forward), intake)
                    .andThen(Commands.waitSeconds(0.15)),

            // 2
            Commands.runOnce(() -> intake.setState(IntakeState.Backward), intake)
                    .andThen(Commands.waitSeconds(0.15)),
            Commands.runOnce(() -> intake.setState(IntakeState.Forward), intake)
                    .andThen(Commands.waitSeconds(0.15)),

            // 3
            Commands.runOnce(() -> intake.setState(IntakeState.Backward), intake)
                    .andThen(Commands.waitSeconds(0.15)),
            Commands.runOnce(() -> intake.setState(IntakeState.Forward), intake)
                    .andThen(Commands.waitSeconds(0.15)),

            // End stopped
            Commands.runOnce(() -> intake.setState(IntakeState.Idle), intake)
        );
    }
}