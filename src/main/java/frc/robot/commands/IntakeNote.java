package frc.robot.commands;

//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeNote extends SequentialCommandGroup {
    public IntakeNote(Arm arm, IntakeSubsystem intake){
        addCommands(
            new SetArm(arm, 23),
            new SetIntakeSpeed_SensorStop(intake, -1),
            new SetArm(arm, 42),
            new SetIntakeSpeed(intake, .3),
            new WaitCommand(.05),
            new SetIntakeSpeed(intake, 0)
            );
    }

}
