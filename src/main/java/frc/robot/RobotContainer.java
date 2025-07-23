
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.StopRobot;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {

  //Controllers
  XboxController driverController = new XboxController(Constants.Controller.USB_DRIVECONTROLLER);
  XboxController auxController = new XboxController(Constants.Controller.USB_AUXCONTROLLER);

  //AUTO SWITCHES
  private static DigitalInput autoSwitch1 = new DigitalInput(Constants.DIO_AUTO_1);
  private static DigitalInput autoSwitch2 = new DigitalInput(Constants.DIO_AUTO_2);
  private static DigitalInput autoSwitch3 = new DigitalInput(Constants.DIO_AUTO_3);
  private static DigitalInput autoSwitch4 = new DigitalInput(Constants.DIO_AUTO_4);

  //Subsystems 
  private final Swerve s_Swerve = new Swerve();

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driverController, XboxController.Button.kY.value);
  private final JoystickButton robotCentric = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);

  //COMMANDS BELOW
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driverController.getRawAxis(translationAxis), 
                () -> -driverController.getRawAxis(strafeAxis), 
                () -> -driverController.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

    // PathPlanner - Register Named Commands (so that they can be used during autos with event markers)
    // Also, PathPlanner Docs suggests that this is done "before the creation of any other commands", so it may be beneficial to do what I did below with StopRobot, which is
    // to create it right when the button is pressed instead of initializing it earlier 

    NamedCommands.registerCommand("stopRobot", new StopRobot(s_Swerve)); // EXAMPLE
    
    // Configure the trigger bindings
    configureBindings();
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
  private void configureBindings() {
    //Buttons
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    //Main Xbox Controller
    JoystickButton a = new JoystickButton(driverController, Constants.XboxController.A);
    JoystickButton b = new JoystickButton(driverController, Constants.XboxController.B);
    //Y on driver controller is assigned to "zeroGyro" above
    // JoystickButton y = new JoystickButton(driver, Constants.XboxController.Y);
    //leftBumper on driver controller is assigned to "robotCentric" above
    // JoystickButton lb = new JoystickButton(driver, Constants.XboxController.LB);
    // JoystickButton y = new JoystickButton(driverController, Constants.XboxController.Y);
    JoystickButton x = new JoystickButton(driverController, Constants.XboxController.X);
    // JoystickButton lb = new JoystickButton(driverController, Constants.XboxController.LB);
    JoystickButton rb = new JoystickButton(driverController, Constants.XboxController.RB);
    JoystickButton lm = new JoystickButton(driverController, Constants.XboxController.LM);
    JoystickButton rm = new JoystickButton(driverController, Constants.XboxController.RM);
    JoystickButton view = new JoystickButton(driverController, Constants.XboxController.VIEW);
    JoystickButton menu = new JoystickButton(driverController, Constants.XboxController.MENU);
    POVButton upPov = new POVButton(driverController,Constants.XboxController.POVXbox.UP_ANGLE);
    POVButton downPov = new POVButton(driverController,Constants.XboxController.POVXbox.DOWN_ANGLE); 
    POVButton leftPov = new POVButton(driverController,Constants.XboxController.POVXbox.LEFT_ANGLE);
    POVButton rightPov = new POVButton(driverController,Constants.XboxController.POVXbox.RIGHT_ANGLE);
    Trigger lt = new Trigger(() -> driverController.getRawAxis(Constants.XboxController.AxesXbox.LTrig) > 0.5);
    Trigger rt = new Trigger(() -> driverController.getRawAxis(Constants.XboxController.AxesXbox.RTrig) > 0.5);

    
    //Secondary Xbox Controller
    JoystickButton x1 = new JoystickButton(auxController, Constants.XboxController.X);
    JoystickButton a1 = new JoystickButton(auxController, Constants.XboxController.A);
    JoystickButton b1 = new JoystickButton(auxController, Constants.XboxController.B);
    JoystickButton y1 = new JoystickButton(auxController, Constants.XboxController.Y);
    JoystickButton lb1 = new JoystickButton(auxController, Constants.XboxController.LB);
    JoystickButton rb1 = new JoystickButton(auxController, Constants.XboxController.RB);
    JoystickButton lm1 = new JoystickButton(auxController, Constants.XboxController.LM);
    JoystickButton rm1 = new JoystickButton(auxController, Constants.XboxController.RM);
    JoystickButton view1 = new JoystickButton(auxController, Constants.XboxController.VIEW);
    JoystickButton menu1 = new JoystickButton(auxController, Constants.XboxController.MENU);
    POVButton upPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.UP_ANGLE);
    POVButton downPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.DOWN_ANGLE);
    POVButton leftPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.LEFT_ANGLE);
    POVButton rightPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.RIGHT_ANGLE);
    Trigger lt1 = new Trigger(() -> auxController.getRawAxis(Constants.XboxController.AxesXbox.LTrig) > 0.5);
    Trigger rt1 = new Trigger(() -> auxController.getRawAxis(Constants.XboxController.AxesXbox.RTrig) > 0.5);


    //Inputs

    //y button is already assigned to ZeroGyro
    //leftBumper lb button is already assigned to RobotCentric


    //DRIVER CONTROLLER

    a.onTrue(new StopRobot(s_Swerve));
    b.onTrue(s_Swerve.followPathCommand("Test"));
    x.onTrue(s_Swerve.followPathCommand("Test").andThen(s_Swerve.followPathCommand("Test2")));
  }

  public Command getAutonomousCommand() {
    try{
        //NOTE: Look at Build an Auto page on PP docs for other way, which is to create/return PathPlannerAuto

        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Test");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

}


