// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  private final Drivetrain drivetrain;
  

  private final DefaultDriveCommand defaultDriveCommand;

  private final XboxController controllerDriver = new XboxController(0);
  private final XboxController controllerManipulator = new XboxController(1);

  private SendableChooser<PathPlannerTrajectory> autoChooser = new SendableChooser<>();

  
  


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drivetrain = new Drivetrain();
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    defaultDriveCommand = new DefaultDriveCommand(
      drivetrain,
      () -> -modifyAxis(controllerDriver.getLeftY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(controllerDriver.getLeftX()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(controllerDriver.getRightX()) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
      controllerDriver::getAButton,
      controllerDriver::getXButton
    );
    
    drivetrain.setDefaultCommand(defaultDriveCommand);


    InitTrajectorys();

    
    //SmartDashboard.putData(autoChooser);
    // Configure the button bindings
    configureButtonBindings();
  }

  

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //  button zeros the gyroscope
    new Trigger(controllerDriver::getBackButton)
      .onTrue( new InstantCommand(() -> drivetrain.zeroGyroscope()));

  }

  public BooleanSupplier getRightTrigger(XboxController controller){
    
    return new BooleanSupplier() {

      @Override
      public boolean getAsBoolean() {
        return controller.getRightTriggerAxis() > .1;
      }
    };
  }

  public BooleanSupplier getLeftTrigger(XboxController controller){
    
    return new BooleanSupplier() {

      @Override
      public boolean getAsBoolean() {
        return controller.getLeftTriggerAxis() > .1;
      }
    };
  }

  private void InitTrajectorys() {
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // This will load the file "Example Path.path" and generate it with a max velocity of 8 m/s and a max acceleration of 5 m/s^2
    

    HashMap<String, Command> eventMap = new HashMap<>();
    
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      drivetrain::getPose, 
      drivetrain::resetOdometry, 
      Constants.kinematics, 
      new PIDConstants(Constants.kPXYController, 0, 0), 
      new PIDConstants(Constants.kPThetaController, 0, Constants.kDThetaController), 
      drivetrain::setModuleStates, 
      eventMap, 
      true,
      drivetrain
      );
      
    Command auto;
    
    auto = autoBuilder.fullAuto(autoChooser.getSelected());
    
      
      

    // Reset odometry to the starting pose of the trajectory.
    //drivetrain.resetOdometry(examplePath.getInitialPose());

    // Run path following command, then stop at the end.
    return auto
    .andThen(() -> drivetrain.stopDrive());

  }

  private static double deadband(double value, double deadband) { 
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.2);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
