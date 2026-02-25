// package frc.robot.Auto;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Robot;
// import frc.robot.Subsystems.Intake;

// public class Ingest extends Command {

//     private final Intake _intake;
//     private final Intake.IntakeState intakeState;

//     public Ingest(Intake.IntakeState intakeState) {
//         this._intake = Robot.getIntakeInstance();
//         this.intakeState = intakeState;

//         addRequirements(_intake);
//     }

//     @Override
//     public void initialize() {
//         _intake.setState(intakeState);
//     }

//     @Override
//     public void execute() {}

//     @Override
//     public void end(boolean interrupted) {
//         _intake.setState(Intake.IntakeState.Idle);
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }
