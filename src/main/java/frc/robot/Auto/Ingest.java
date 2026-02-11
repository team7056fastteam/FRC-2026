package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.IntakePivot;

public class Ingest extends Command {

    private final Intake _intake;
    private final IntakePivot _intakePivot;
    private final Intake.IntakeState intakeState;
    private final IntakePivot.IntakePivotState pivotState;

    public Ingest(Intake.IntakeState intakeState, IntakePivot.IntakePivotState pivotState) {
        this._intake = Robot.getIntakeInstance();
        this._intakePivot = Robot.getIntakePivotInstance();
        this.intakeState = intakeState;
        this.pivotState = pivotState;

        addRequirements(_intake, _intakePivot);
    }

    @Override
    public void initialize() {
        _intakePivot.setState(pivotState);
    }

    @Override
    public void execute() {
        if(_intakePivot.inPos()){
            _intake.setState(intakeState);
        }
    }

    @Override
    public void end(boolean interrupted) {
        _intake.setState(Intake.IntakeState.Idle);
        _intakePivot.setState(IntakePivot.IntakePivotState.Up);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
