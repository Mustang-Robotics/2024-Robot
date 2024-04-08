package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.networktables.GenericEntry;
//import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SpeakerShoot;
import frc.robot.commands.AShoot;
import frc.robot.commands.CornerAShoot;
import frc.robot.commands.IntakeNote;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.SetArm;
import frc.robot.commands.SetArmAim;
import frc.robot.commands.AutoIntakeNote;

import frc.robot.commands.AmpShoot;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;


import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BottomShooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TopShooter;
import frc.robot.subsystems.ClimbingSubsystem;
//import edu.wpi.first.networktables.GenericEntry;

public class RobotContainer {
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    SendableChooser<Command> m_chooser;
    private final Arm m_arm = new Arm();
    public final TopShooter m_ShooterSubsystem = new TopShooter();
    public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    public final BottomShooter m_bottom = new BottomShooter();
    public final ClimbingSubsystem m_climbingSubsystem = new ClimbingSubsystem();
    public Command centerPathCommand;
    public Command speakerPathCommand;
    public Command stageBottomPathCommand;
    public Command stageMiddleCommand;
    public Command stageTopPathCommand;
    public Command AmpPathCommand;
    PIDController circleNote = new PIDController(.02, 0, 0);

    public boolean ArmAdjustmentActiveTF;
    public boolean buttonPressActiveTF;

    public RobotContainer() {// Configure the button bindings
        //m_robotDrive.AutonomousBuilder();
        //Subsystem Initialization Functions
        m_arm.armInitialize();
        m_ShooterSubsystem.enable();
        m_bottom.enable();
        m_arm.enable();
        buildPathCommands();

        ArmAdjustmentActiveTF = false;
        buttonPressActiveTF = false;
        
        
        
        
    
        configureButtonBindings();
        m_robotDrive.calibrateGyro();
        // Configure default commands
        m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true, true), //Base: true, true
                m_robotDrive));
            //new RunCommand(
            //    () -> m_ShooterSubsystem.runFeeder(m_driverController.getRightTriggerAxis()), m_ShooterSubsystem);
        m_climbingSubsystem.setDefaultCommand(
            new RunCommand(
                () -> m_climbingSubsystem.SetClimbSpeed(-m_driverController.getLeftTriggerAxis() + m_driverController.getRightTriggerAxis()), m_climbingSubsystem));    
            

        NamedCommands.registerCommand("Intake", new AutoIntakeNote(m_arm, m_intakeSubsystem));//new IntakeNote(m_arm, m_intakeSubsystem));
        NamedCommands.registerCommand("SubwooferShoot", new SpeakerShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom));
        NamedCommands.registerCommand("AmpShoot", new AmpShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom));
        NamedCommands.registerCommand("AShoot", new AShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom));
        NamedCommands.registerCommand("CornerAShoot", new CornerAShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom));
        m_chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", m_chooser);
                
    }
    
    //private GenericEntry setPoint = m_arm.tab.add("setPoint", 90).getEntry();
    //private GenericEntry shooterSpeed = m_arm.tab.add("shooterSpeed", 4500).getEntry();
    //private GenericEntry shooterSpeedDiff = m_arm.tab.add("shooterSpeedDiff", 0).getEntry();
    private void configureButtonBindings() { //NOTE: All button commands
         
        new JoystickButton(m_driverController, Button.kA.value).whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
        //new JoystickButton(m_driverController, Button.kA.value).onTrue(new RunCommand(() -> m_arm.setGoal(setPoint.getDouble(0)), m_arm));
        //new JoystickButton(m_driverController, Button.kStart.value).toggleOnTrue(new IntakeNote(m_arm, m_intakeSubsystem));//new StartEndCommand(() -> m_ShooterSubsystem.runFeeder(.5),() -> m_ShooterSubsystem.runFeeder(0), m_ShooterSubsystem));
        new JoystickButton(m_driverController, Button.kStart.value).onTrue(new SpeakerShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom).andThen(new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true, true), //Base: true, true
                m_robotDrive)));
        new JoystickButton(m_driverController, Button.kBack.value).toggleOnTrue(new ParallelCommandGroup(
            new SetArmAim(m_arm, m_robotDrive, m_ShooterSubsystem, m_bottom), 
            new RunCommand(() -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    circleNote.calculate(m_robotDrive.vision.rotationToSpeaker()[1], 0)
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true, true), //Base: true, true
                m_robotDrive)));
        new JoystickButton(m_driverController, Button.kB.value).whileTrue(new StartEndCommand(() -> m_intakeSubsystem.setSpeed(1),
            () -> m_intakeSubsystem.setSpeed(0), m_intakeSubsystem));
        

        //new JoystickButton(m_driverController, Button.kBack.value).onTrue(new InitializePrepareShoot(ArmAdjustmentActiveTF, this, m_ShooterSubsystem, m_bottom));
        new JoystickButton(m_driverController, Button.kRightBumper.value).toggleOnTrue(new ParallelCommandGroup(new SetArm(m_arm, 115), AmpPathCommand).andThen(new AmpShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom)));
        new JoystickButton(m_driverController, Button.kLeftBumper.value).whileTrue(new RunCommand(() -> m_arm.setGoal(115), m_arm));
        //new JoystickButton(m_driverController, Button.kB.value).toggleOnTrue(stageBottomPathCommand.andThen(new AShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom)));
        //new JoystickButton(m_driverController, Button.kY.value).toggleOnTrue(stageMiddleCommand.andThen(new AShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom)));
        new JoystickButton(m_driverController, Button.kX.value).toggleOnTrue(stageTopPathCommand.andThen(new AShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom)));
        //new JoystickButton(m_driverController, Button.kRightBumper.value).toggleOnTrue(new AmpShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom));
        new JoystickButton(m_driverController, Button.kY.value).toggleOnTrue(new ParallelRaceGroup(new RunCommand(
                () -> m_robotDrive.drive(
                    (MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband)*0.7)-0.3,
                    MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    circleNote.calculate(m_robotDrive.vision.rotationToObject(), 0)
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                false, true, false), //Base: true, true
                m_robotDrive), new IntakeNote(m_arm, m_intakeSubsystem)));
    }
    //If (on & button.on), run adjustment, else, dont
    
    

    //private GenericEntry kp = m_ShooterSubsystem.tab.add("kp", 0).getEntry();
    //private GenericEntry ki = m_ShooterSubsystem.tab.add("ki", 0).getEntry();
    //private GenericEntry kd = m_ShooterSubsystem.tab.add("kd", 0).getEntry();
    //private GenericEntry PID_kp = m_arm.tab.add("PID_kp", 0.5).getEntry();
    //private GenericEntry PID_ki = m_arm.tab.add("PID_ki", 0.5).getEntry();
    //private GenericEntry PID_kd = m_arm.tab.add("PID_kd", 0).getEntry();
    public void SetArmDistanceAngle() {
        new SetArmAim(m_arm, m_robotDrive, m_ShooterSubsystem, m_bottom);
    }
    public void SetArmAdjustmentActiveTrue() {
        ArmAdjustmentActiveTF = true;
    }
    public void SetArmAdjustmentActiveFalse() {
        ArmAdjustmentActiveTF = false;
    }

    public void anglePublish(){
        SmartDashboard.putNumber("Arm Angle", m_arm.getMeasurement());
        SmartDashboard.putData(m_arm);
        //m_arm.getController().setPID(PID_kp.getDouble(0.5), PID_ki.getDouble(0.5
        //), PID_kd.getDouble(0));
        //m_ShooterSubsystem.getController().setPID(kp.getDouble(0), ki.getDouble(0), kd.getDouble(0));
    }

    private void buildPathCommands(){
        //Pathfinding commands, going to 3 set points. Prep for shooting a note from defined locations
        PathPlannerPath path = PathPlannerPath.fromPathFile("Cent");

        PathConstraints constraints = new PathConstraints(2, 2, Units.degreesToRadians(360), Units.degreesToRadians(360));
    
        centerPathCommand = AutoBuilder.pathfindThenFollowPath(path, constraints, 0.0);

        
        //####End of Orig Commands

        
        //PathPlannerPath SpeakerPath = PathPlannerPath.fromPathFile("Shoot - At Speaker");
        //speakerPathCommand = AutoBuilder.pathfindThenFollowPath(SpeakerPath, constraints, 0.0);
        PathPlannerPath StageBottomPath = PathPlannerPath.fromPathFile("B");
        stageBottomPathCommand = AutoBuilder.pathfindThenFollowPath(StageBottomPath, constraints, 0.0);
        PathPlannerPath StageMiddlePath = PathPlannerPath.fromPathFile("Y");
        stageMiddleCommand = AutoBuilder.pathfindThenFollowPath(StageMiddlePath, constraints, 0.0);
        PathPlannerPath StageTopPath = PathPlannerPath.fromPathFile("X");
        stageTopPathCommand = AutoBuilder.pathfindThenFollowPath(StageTopPath, constraints, 0.0);
        PathPlannerPath AmpPath = PathPlannerPath.fromPathFile("RB");
        AmpPathCommand = AutoBuilder.pathfindThenFollowPath(AmpPath, constraints, 0.0);

    }
    
    
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
    }

}
