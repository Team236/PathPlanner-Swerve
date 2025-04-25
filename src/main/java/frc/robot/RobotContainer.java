
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

 //GUILLOTINE
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
  // private final AlgaeHold  algaeHold = new AlgaeHold();
  // private final AlgaePivot algaePivot = new AlgaePivot();
  // private final Elevator elevator = new Elevator();
  // private final CoralHold coralHold = new CoralHold();
  // private final CoralPivot coralPivot = new CoralPivot(); 
  private final Swerve s_Swerve = new Swerve();

    
    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driverController, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);

   //COMMANDS

   //Drive
  //   private final OrientWithLL orientWithLL = new OrientWithLL(s_Swerve);

  //  //Targeting
  //   private final AlgaeTarget algaeTarget = new AlgaeTarget(s_Swerve);
  //   private final CoralLeftTarget coralLeftTarget = new CoralLeftTarget(s_Swerve);
  //   private final CoralRightTarget coralRightTarget = new CoralRightTarget(s_Swerve);
  //   //private final GoToCoralRightLL goToCoralRightLL = new GoToCoralRightLL(s_Swerve);
  //   // private final GoToCoralLeftFwdSide goToCoralLeftLL = new GoToCoralLeftFwdSide(s_Swerve);
  //  // private final UpdateRobotPosition updateRobotPosition = new UpdateRobotPosition(s_Swerve);
  //  // private final UpdateTargetPosition updateTargetPosition = new UpdateTargetPosition(s_Swerve);
  //  // private final GoToCoralLeftLL goToCoralLeftLL = new GoToCoralLeftLL(s_Swerve);
  //   private final FieldCentricTargetLeft fieldCentricTargetLeft = new FieldCentricTargetLeft(s_Swerve);
  //   private final FieldCentricTargetRight fieldCentricTargetRight = new FieldCentricTargetRight(s_Swerve);
  //   private final FieldCentricTargetAlgae fieldCentricTargetAlgae = new FieldCentricTargetAlgae(s_Swerve);
  //   // private final NewFieldCentricTargetLeft newFieldCentricTargetLeft = new NewFieldCentricTargetLeft(s_Swerve);
  // // private final FieldCentricTargetCameraToTag fieldCentricTargetCameraToTag = new FieldCentricTargetCameraToTag(s_Swerve);
  //  //private final TestAutoSequence testAutoSequence = new TestAutoSequence(s_Swerve);


  // //NOTE - STANDOFF FWD IS WITHOUT THE BUMPER - ADD BUMPER DEPTH AS NEEDEDD
  //   private final TargetAllParallel targetAllParallel = new TargetAllParallel(s_Swerve, 12, 0);
  //   private final TargetAngle targetAngle =  new TargetAngle(s_Swerve);
  //   private final TargetForwardDistance targetForwardDistance = new TargetForwardDistance(s_Swerve, 9);
  //   private final TargetSideDistance targetsideDistance = new TargetSideDistance(s_Swerve, 6.5);
  //   private final TargetSideDistance targetSideDistanceChanged  = new TargetSideDistance(s_Swerve,0);
  //   //private final TargetMegaTag2 target3DMaTag2 = new TargetMegaTag2(s_Swerve);
  //   private final TargetAngleSide targetAngleSide = new TargetAngleSide(s_Swerve, 0);

  //   // Scoring
  //   private final L1_Pt1 l1_Pt1 = new L1_Pt1(elevator, coralHold, coralPivot, algaePivot);
  //   private final L1_Pt2 l1_Pt2 = new L1_Pt2(elevator, coralHold, coralPivot, algaePivot);
  //   private final L2_Pt1 l2_Pt1 = new L2_Pt1(elevator, coralHold, coralPivot, algaePivot);
  //   private final L2_Pt2 l2_Pt2 = new L2_Pt2(elevator, coralHold, coralPivot, algaePivot);
  //   private final L3_Pt1 l3_Pt1 = new L3_Pt1(elevator, coralHold, coralPivot, algaePivot);
  //   private final L3_Pt2 l3_Pt2 = new L3_Pt2(elevator, coralHold, coralPivot, algaePivot);
  //   private final L4_Pt1 l4_Pt1 = new L4_Pt1(elevator, coralHold, coralPivot, algaePivot);
  //   private final L4_Pt2 l4_Pt2 = new L4_Pt2(elevator, coralHold, coralPivot, algaePivot);
  //   private final L4_Pt2_Algae_Bump l4_Pt2_Algae_Bump = new L4_Pt2_Algae_Bump(elevator, coralPivot, coralHold, algaePivot);


    //Auto
   // private final DriveFwd driveFwd = new DriveFwd(s_Swerve, false, 10); //9
   //private final TurnOnly turnOnlyNeg90 = new TurnOnly(s_Swerve, false, -90);
   // private final DriveFwdAndSideAndTurn driveFwdAndSideAndTurn = new DriveFwdAndSideAndTurn(s_Swerve, false, 9, 0, 0);
    //private final DriveFwd driveFwd9 = new DriveFwd(s_Swerve, false, 9);//
   //private final TurnOnly turnOnly1125 = new TurnOnly(s_Swerve, false, 11.25);
   //private final TurnOnly turnOnly45 = new TurnOnly(s_Swerve, false, 45);
   // private final DriveReverse driveReverse9 = new DriveReverse(s_Swerve, true,-9);
   // private final DriveSideways driveSideways675 = new DriveSideways(s_Swerve, false, 6.5);
   // private final DriveSideways driveSidewaysNeg675 = new DriveSideways(s_Swerve, false, -6.5);

//     private final Leg1and2Practice leg1and2Practice = new Leg1and2Practice(s_Swerve, elevator, algaePivot, coralPivot, coralHold);

//     private final Leg1Left leg1Left = new Leg1Left(s_Swerve,  elevator, algaePivot, coralPivot, coralHold);
//     private final Leg2Left leg2Left = new Leg2Left(s_Swerve, coralHold, coralPivot, elevator);
//     private final Leg3Left leg3Left = new Leg3Left(s_Swerve, elevator, algaePivot, coralPivot, coralHold);
//     private final FullRunLeft fullRunLeft = new FullRunLeft(s_Swerve, elevator, algaePivot, coralPivot, coralHold);
//     private final Legs1and2Left legs1and2Left = new Legs1and2Left(s_Swerve, elevator, algaePivot, coralPivot, coralHold);

//     private final Leg1Right leg1Right = new Leg1Right(s_Swerve,  elevator, algaePivot, coralPivot, coralHold);
//     private final Leg2Right leg2Right = new Leg2Right(s_Swerve, coralHold, coralPivot, elevator);
//     private final Leg3Right leg3Right = new Leg3Right(s_Swerve, elevator, algaePivot, coralPivot, coralHold);
//     private final FullRunRight fullRunRight = new FullRunRight(s_Swerve, elevator, algaePivot, coralPivot, coralHold);
//     private final Legs1and2Right legs1and2Right = new Legs1and2Right(s_Swerve, elevator, algaePivot, coralPivot, coralHold);

//     private final CtrScore1 ctrScore1 = new CtrScore1(s_Swerve, elevator, algaePivot, algaeHold, coralPivot, coralHold);
//    // private final DriveWithPath driveWithPathLeg1 = new DriveWithPath(s_Swerve, false);
    
      
//   //Elevator
//   private final DangerManualUpDown dangerElevatorUp = new DangerManualUpDown(elevator, Constants.Elevator.ELEV_UP_SPEED);
//   private final DangerManualUpDown dangerElevatorDown = new DangerManualUpDown(elevator, Constants.Elevator.ELEV_DOWN_SPEED);
//   private final ClimbDownSequence climbDownSequence= new ClimbDownSequence(elevator, algaePivot, Constants.Elevator.ELEV_CLIMB_DOWN_SPEED);
//   private final PrepForClimb prepForClimb = new PrepForClimb(elevator,  algaePivot);
// //private final ElevMotionMagicPID motionMagicToTestLevel  = new ElevMotionMagicPID(elevator, 40);
//  private final ElevMotionMagicPID motionMagicToBottom  = new ElevMotionMagicPID(elevator, 0);



//   //AlgaeHold
//   private final AlgaeGrab algaeGrab = new AlgaeGrab(algaeHold, Constants.AlgaeHold.HOLD_SPEED1, Constants.AlgaeHold.HOLD_SPEED2);
//   private final AlgaeRelease algaeRelease = new AlgaeRelease(algaeHold, Constants.AlgaeHold.PROCESSOR_SPEED);
//   private final AlgaeL3Pickup algaeHighPickup = new AlgaeL3Pickup(elevator, algaeHold, algaePivot);
//   private final AlgaeL2Pickup algaeLowPickup = new AlgaeL2Pickup(elevator, algaeHold, algaePivot);

//   //AlgaePivot
//   private final ManualAlgaePivot algaePivotDown = new ManualAlgaePivot(algaePivot, Constants.AlgaePivot.MAN_EXT_SPEED);
//   private final ManualAlgaePivot algaePivotUp = new ManualAlgaePivot(algaePivot, Constants.AlgaePivot.MAN_RET_SPEED);
//   private final PIDAlgaePivot pidReefPickup = new PIDAlgaePivot(algaePivot, Constants.AlgaePivot.ENC_REVS_REEF_PICKUP);
//   //private final PIDAlgaePivot pidAlgaeScoreNet = new PIDAlgaePivot(algaePivot, Constants.AlgaePivot.ENC_REVS_SCORE_NET);
//   private final PIDAlgaePivot pidSafePosition = new PIDAlgaePivot(algaePivot, Constants.AlgaePivot.ENC_REVS_ELEVATOR_SAFE_POSITION);
//   private final PIDToElevSafePosition pidToElevSafePosition = new PIDToElevSafePosition(algaePivot);

//   //CoralHold
//   private final CoralGrab coralGrab = new CoralGrab(coralHold, Constants.CoralHold.HOLD_SPEED);
//   private final CoralGrabWithCounter coralGrabWithCounter = new CoralGrabWithCounter(coralHold, Constants.CoralHold.HOLD_SPEED);
//   private final CoralSeqGrabCount coralSeqGrabCount = new CoralSeqGrabCount(coralPivot, coralHold, elevator);
//   private final CoralRelease coralRelease = new CoralRelease(coralHold, Constants.CoralHold.L2_RELEASE_SPEED);
//   //private final CoralRelease coralReleaseL4 = new CoralRelease(coralHold, Constants.CoralHold.L4_RELEASE_SPEED);

//   //CoralPivot
//   private final ManualCoralPivot coralPivotDown = new ManualCoralPivot(coralPivot, Constants.CoralPivot.MAN_EXT_SPEED);
//   private final ManualCoralPivot coralPivotUp = new ManualCoralPivot(coralPivot, Constants.CoralPivot.MAN_RET_SPEED);
  //private final PIDCoralPivot pidCoraltoL1 = new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_LEVEL1);
 // private final PIDCoralPivot pidCoraltoL2 = new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_LEVEL2);
  //private final PIDCoralPivot pidCoraltoL3 = new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_LEVEL3);
  //private final PIDCoralPivot pidCoraltoL4 = new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_LEVEL4);

  
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
      
    // put in april tag coordinates (not sure where else to put it)
    // Constants.Targeting.ID_TO_POSE.put(6, new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(130.17), new Rotation2d(Units.degreesToRadians(300))));
    // Constants.Targeting.ID_TO_POSE.put(7, new Pose2d(Units.inchesToMeters(546.87), Units.inchesToMeters(158.50), new Rotation2d(Units.degreesToRadians(0))));
    // Constants.Targeting.ID_TO_POSE.put(8, new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(186.83), new Rotation2d(Units.degreesToRadians(60))));
    // Constants.Targeting.ID_TO_POSE.put(9, new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(186.83), new Rotation2d(Units.degreesToRadians(120))));
    // Constants.Targeting.ID_TO_POSE.put(10, new Pose2d(Units.inchesToMeters(481.39), Units.inchesToMeters(158.50), new Rotation2d(Units.degreesToRadians(180))));
    // Constants.Targeting.ID_TO_POSE.put(11, new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(130.17), new Rotation2d(Units.degreesToRadians(240))));
      
    // Constants.Targeting.ID_TO_POSE.put(17, new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17), new Rotation2d(Units.degreesToRadians(240))));
    // Constants.Targeting.ID_TO_POSE.put(18, new Pose2d(Units.inchesToMeters(144.00), Units.inchesToMeters(158.50), new Rotation2d(Units.degreesToRadians(180))));
    // Constants.Targeting.ID_TO_POSE.put(19, new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83), new Rotation2d(Units.degreesToRadians(120))));
    // Constants.Targeting.ID_TO_POSE.put(20, new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83), new Rotation2d(Units.degreesToRadians(60))));
    // Constants.Targeting.ID_TO_POSE.put(21, new Pose2d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.50), new Rotation2d(Units.degreesToRadians(0))));
    // Constants.Targeting.ID_TO_POSE.put(22, new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17), new Rotation2d(Units.degreesToRadians(300))));

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


//a.onTrue(turnOnly225);
//b.onTrue(turnOnlyNeg90);
//x.onTrue(turnOnly45);
//  rm.onTrue(driveSideways675);
// rb.onTrue(driveSidewaysNeg675);
//upPov.onTrue(driveFwd9);
//downPov.onTrue(driveReverse9);
// y1.onTrue(driveReverse10);
// rb1.onTrue(turnOnly90);
// lb1.onTrue(turnOnlyNeg90);

//leftPov.whileTrue(driveFwdAndSideAndTurn);

//rb.whileTrue(targetAllParallel);
//upPov.whileTrue(targetForwardDistance);
//downPov.whileTrue(targetsideDistance);

//upPov.whileTrue(leg1Right);
//a.whileTrue(legs1and2Right);
//b.whileTrue(driveWithPathLeg1);
//x.whileTrue(leg2Right);

//a.onTrue(pidCoraltoL1);
//x.onTrue(pidCoraltoL2);
//b.onTrue(pidCoraltoL3);
//rightPov.onTrue(pidCoraltoL4);


//rightPov.onTrue(pidReefPickup); //pivot Algae to position for Reef pickup
//b.onTrue(pidAlgaeScoreNet); //pivot Algae to score in Net position
//b.onTrue(pidToElevSafePosition); //if AP under elevator, move to safe position
//upPov.onTrue(pidPrepForClimb);
//downPov.onTrue(pidClimb);

//****DO NOT USE COMMANDS BELOW WHEN ALGAE PIVOT ATTACHED***
// downPov.whileTrue(dangerElevatorDown);
// upPov.whileTrue(dangerElevatorUp); 
//downPov.onTrue(dangerPidElevL1);
//leftPov.onTrue(dangerPidElevL2);
//rightPov.onTrue(dangerPidElevL3); 
//upPov.onTrue(dangerPidElevL4);

//a.whileTrue(targetsideDistance);
//x.whileTrue(targetForwardDistance);
//b.whileTrue(targetAngle);
//rb.whileTrue(targetAllParallel);

//rightPov.whileTrue(coralGrab);
//b.whileTrue(coralRelease);
//x.whileTrue(coralGrabWithCounter);

//rm.whileTrue(coralPivotUp);
//rb.whileTrue(coralPivotDown);

//a.onTrue(coralSeqGrabCount);
//lm.whileTrue(coralRelease);
 
//a1.whileTrue(algaeTarget);
//x.whileTrue(coralLeftTarget);
//b.whileTrue(coralRightTarget);

//upPov.onTrue(leg1Right);
//downPov.onTrue(leg1Left);
//a.onTrue(ctrScore1);

//leftPov.onTrue(prepForClimb);
//rightPov.onTrue(climbDownSequence);

//y1.onTrue(l1_Pt1);
//a1.onTrue(l2_Pt2e);
//x1.onTrue(l3_ptl2_Pt1);
//y1.onTrue(l4_Sco_l2_Pt2//b1.onTrue(motionMagicToBottomptl3_Pt1/lm1.whileTrue(algaeGrab);
//rm1.whileTrue(algaeRelea_l3_Pt2
//leftPov1.whileTrue(algaePivoptl4_Pt1
//rightPov1.whileTrue(algaePivotDo_l4_Pt2
//downPov1.whileTrue(dangerElevatorDown);
//upPov1.whileTrue(dangerElevatorUp);



// a.onTrue(algaeGrab).onTrue(l3_Score);
// b.onTrue(algaeGrab).onTrue(l4_Score);
  }

  public Command getAutonomousCommand() {

  // SmartDashboard.putString("autokey", "Entering getAutoCommand now");
  SmartDashboard.putString("Asking for auto sequence", "" + !autoSwitch1.get() + !autoSwitch2.get() + !autoSwitch3.get() + !autoSwitch4.get());
  Command command = null;


  //! means switch is on
  if (!autoSwitch1.get() && !autoSwitch2.get() && !autoSwitch3.get() && !autoSwitch4.get()) {

  } else if (!autoSwitch1.get() && !autoSwitch2.get() && !autoSwitch3.get() && autoSwitch4.get()) {

  } else if (!autoSwitch1.get() && !autoSwitch2.get() && autoSwitch3.get() && autoSwitch4.get()) {

  } else if (autoSwitch1.get() && !autoSwitch2.get() && !autoSwitch3.get() && !autoSwitch4.get()) {

  } else if (autoSwitch1.get() && !autoSwitch2.get() && !autoSwitch3.get() && autoSwitch4.get()) {

  } else if (autoSwitch1.get() && !autoSwitch2.get() && autoSwitch3.get() && autoSwitch4.get()) {

  } else if (autoSwitch1.get() && autoSwitch2.get() && autoSwitch3.get() && autoSwitch4.get()) {

  } 
 return command;
}

}
    /* 
    // SmartDashboard.putString("autokey", "Entering getAutoCommand now");
    Command command = null;
    AutoSwitchHelpers autoSwitchHelpers = new AutoSwitchHelpers();
    // Switch 1 is in the "ON" spot on the old auto box

    if (autoSwitchHelpers.switchesAre(true, true, true, true)){
      command = fullRunRight;
    } else if (autoSwitchHelpers.switchesAre(true, true, false, false)){
      command = leg1Right;
    } else if (autoSwitchHelpers.switchesAre(true, true, true, false)){
      command = legs1and2Right;
    }  else if (autoSwitchHelpers.switchesAre(false, true, true, true)){
        command = fullRunLeft;
    }  else if (autoSwitchHelpers.switchesAre(false, true, false, false)){
        command = leg1Left;
    }  else if (autoSwitchHelpers.switchesAre(false, true, true, false)){
        command = legs1and2Left;
    }  else if (autoSwitchHelpers.switchesAre(false, false, false, false)){
        command = fullRunCenter;
    }
   return command;
   */
 


