// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.Autos;
//import frc.robot.commands.ExampleCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
 private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
//xbox controller buttons

//Button board controls:
Joystick bb1 = new Joystick(1);
Trigger bb1B1 = new JoystickButton(bb1, 1);  //Level 1
Trigger bb1B2 = new JoystickButton(bb1, 2);  //Level 2
Trigger bb1B3 = new JoystickButton(bb1, 3);  //Level 3
Trigger bb1B4 = new JoystickButton(bb1, 4);  //Level 4
Trigger bb1B5 = new JoystickButton(bb1, 5);  //Intake Height
Trigger bb1B6 = new JoystickButton(bb1, 6);  //Wrist Rest
Trigger bb1B7 = new JoystickButton(bb1, 7);  //Algae L1  37
Trigger bb1B8 = new JoystickButton(bb1, 8);  //Algae L2  54
Trigger bb1B9 = new JoystickButton(bb1, 9);  //Elevator 0
Trigger bb1B10 = new JoystickButton(bb1, 10);  //Field Centrivity

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController m_driverController =
      //new CommandXboxController(OperatorConstants.kDriverControllerPort);
     XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort); 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kRightStick.value) //May need to change the button.kLeftStick.Value back to kR1
      .whileTrue(new RunCommand(
        () -> m_robotDrive.setX(),
        m_robotDrive));

        new JoystickButton(m_driverController,4 ).whileTrue(new InstantCommand(m_ElevatorSubsystem:: ElevatorUp, m_ElevatorSubsystem)); //Elevator go brrr upwards
        new JoystickButton(m_driverController,4 ).whileFalse(new InstantCommand(m_ElevatorSubsystem:: ElevatorStop , m_ElevatorSubsystem)); //Y to go up

        new JoystickButton(m_driverController,1 ).whileTrue(new InstantCommand(m_ElevatorSubsystem:: ElevatorDown, m_ElevatorSubsystem)); //Elevator go brrr downwards
        new JoystickButton(m_driverController,1 ).whileFalse(new InstantCommand(m_ElevatorSubsystem:: ElevatorStop , m_ElevatorSubsystem)); // A to go down

        new JoystickButton(m_driverController, 5).whileTrue(new InstantCommand(m_IntakeSubsystem:: IntakeAlgae, m_IntakeSubsystem)); // LB to intake
        new JoystickButton(m_driverController, 5).whileFalse(new InstantCommand(m_IntakeSubsystem:: AlgaeStop, m_IntakeSubsystem));

        new JoystickButton(m_driverController, 6).whileTrue(new InstantCommand(m_IntakeSubsystem:: ScoreAlgae, m_IntakeSubsystem)); //RB to score
        new JoystickButton(m_driverController, 6).whileFalse(new InstantCommand(m_IntakeSubsystem:: AlgaeStop, m_IntakeSubsystem));

       new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value)
       .whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(),m_robotDrive));

        // intake controls (run while button is held down, run retract command once when the button is released)
    new Trigger(
      () ->
          m_driverController.getLeftTriggerAxis()
              > Constants.OIConstants.kTriggerButtonThreshold)
  .whileTrue(new StartEndCommand(() -> m_IntakeSubsystem.IntakeCoral(),
        () -> m_IntakeSubsystem.StopCoral(), m_IntakeSubsystem));
            
   //.whileFalse(m_IntakeSubsystem.StopCoral());

    // intake controls (run while button is held down, run retract command once when the button is released)
    new Trigger(
      () ->
          m_driverController.getRightTriggerAxis()
              > Constants.OIConstants.kTriggerButtonThreshold)
     .whileTrue(new StartEndCommand(() -> m_IntakeSubsystem.ScoreCoral(),
       () -> m_IntakeSubsystem.StopCoral(), m_IntakeSubsystem));
   //.whileFalse(m_IntakeSubsystem.StopCoral());

         new POVButton(m_driverController, 180)
        .whileTrue(new StartEndCommand(() -> m_IntakeSubsystem.MoveWristUp(),
        () -> m_IntakeSubsystem.StopWrist(), m_IntakeSubsystem));

    new POVButton(m_driverController, 0)
    .whileTrue(new StartEndCommand(() -> m_IntakeSubsystem.MoveWristDown(),
    () -> m_IntakeSubsystem.StopWrist(), m_IntakeSubsystem));
  
      //Climber go down
       new POVButton(m_driverController, 90)
      .whileTrue(new StartEndCommand(() -> m_ClimbSubsystem.ClimberUp(),
      () -> m_ClimbSubsystem.ClimberStop(), m_ClimbSubsystem));
  
        //Climber go up
        new POVButton(m_driverController, 270)
        .whileTrue(new StartEndCommand(() -> m_ClimbSubsystem.ClimberDown(),
        () -> m_ClimbSubsystem.ClimberStop(), m_ClimbSubsystem));


    //(Constants.controlLoop(( m_ElevatorSubsystem.m_RelativeEncoder1).getPosition(),Units.inchesToMeters(0)));
     // bb1B1.onTrue(new InstantCommand(() -> ( m_ElevatorSubsystem).setVoltage(Constants.controlLoop((m_ElevatorSubsystem.m_RelativeEncoder1).getPosition(), Units.inchesToMeters(30)))));
     // bb1B9.onTrue(new InstantCommand(() -> Constants.controlLoop((m_ElevatorSubsystem.m_RelativeEncoder1).getPosition(), Units.inchesToMeters(0))));
      bb1B10.onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(),m_robotDrive));
  
  } 



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

        var thetaController = new ProfiledPIDController(
          AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
          exampleTrajectory,
          m_robotDrive::getPose, // Functional interface to feed supplier
          DriveConstants.kDriveKinematics,
  
          // Position controllers
          new PIDController(AutoConstants.kPXController, 0, 0),
          new PIDController(AutoConstants.kPYController, 0, 0),
          thetaController,
          m_robotDrive::setModuleStates,
          m_robotDrive);
  
      // Reset odometry to the starting pose of the trajectory.
      m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
  
      // Run path following command, then stop at the end.
      return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
