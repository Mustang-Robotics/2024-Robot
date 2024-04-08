package frc.robot.commands;

//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BottomShooter;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TopShooter;

public class AShoot extends SequentialCommandGroup {
    public AShoot(Arm arm, IntakeSubsystem intake, TopShooter top, BottomShooter bottom){
        addCommands(
            new SetArm(arm, 51), //Arm Angle to shoot --prev:51
            new SetShooterSpeed(top, bottom, 3500, 400, 75), //Shooter shooting speed. Takes time to rev up
            
            //Wait until arm and shooter are ready

            new SetIntakeSpeed(intake, -1), //Load intake to shoot

            new WaitCommand(0.3),

            //Wait while shooting
            new SetIntakeSpeed(intake, 0), //Intake speed set to 0
            //new ZeroShooter(bottom, top),
            new SetShooterSpeed(top, bottom, 0, 0, 100), //Intake speed set to 0
            new SetArm(arm, 42) //Return arm to base angle

            //new SetArm(arm, 23),
            //new SetIntake(intake, -1),
            //new SetArm(arm, 42),
            //new IntakeSpeed(intake, .3),
            //new WaitCommand(.05),
            //new IntakeSpeed(intake, 0)
            );
    }

}

