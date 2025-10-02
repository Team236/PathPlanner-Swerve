package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public SwerveDrivePoseEstimator m_poseEstimator;
    public double poseAngle;
    public double poseForwardDistance;
    public double poseSideDistance;

    //ll stuff
    private double pipeline = 0; 
    private double tv;
    public Pose2d poseLL; //want to use this pose after this command, after moving with odometry
    public Pose2d targetPose;

    //SmartDashboard
    private Field2d field = new Field2d();

    //targeting
    public SwerveControllerCommand currentSwerveControllerCommand;
    public Trajectory currentTrajectory;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "usb");
        //when calibrated on 3/31/25, gyro mount pose configs quarternion values were:
        //gyro.getConfigurator().apply(-0.041875,  0.012086,  0.005250, Z	-0.997314))
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants), //front left
            new SwerveModule(1, Constants.Swerve.Mod1.constants), //front right
            new SwerveModule(2, Constants.Swerve.Mod2.constants), //back left
            new SwerveModule(3, Constants.Swerve.Mod3.constants) //back right
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        /* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings, for 3D targeting. 
        The numbers used below are robot specific, and should be tuned. */
        m_poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                 mSwerveMods[0].getPosition(), //front left
                mSwerveMods[1].getPosition(), //front right
                mSwerveMods[2].getPosition(), //back left
                mSwerveMods[3].getPosition()  //back right
            },
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Math.toRadians(5)), //std deviations in X, Y (meters), and angle of the pose estimate
            VecBuilder.fill(0.5, 0.5, Math.toRadians(30))  //std deviations  in X, Y (meters) and angle of the vision (LL) measurement
        );

        SmartDashboard.putData("Field", field);

        // PATH PLANNER

        SmartDashboard.putString("Starting swerve and pathplanner", "yes");

        RobotConfig config = null; // this is a PathPlannerLib object that will store the robot config values like mass, wheel numbers, etc. that are set in the App GUI
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        // PathPlanner requires an "AutoBuilder" object to be configured in order to run any paths. This object needs access to the robot's drive/pose methods and other
        // important information. PathPlanner recommends that this is configured in the drive subsystem's constructor (because it can take a bit to load), so it's done here.
        if (config == null) {
            System.out.println("PathPlanner RobotConfig settings null, may have errored");
        } else {
            AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getChassisSpeeds,
                (speeds, feedforwards) -> driveWithChassisSpeeds(speeds),
                new PPHolonomicDriveController(
                    Constants.PathPlanner.TRANSLATION_PID_CONSTANTS, //translation
                    Constants.PathPlanner.ROTATION_PID_CONSTANTS // rotation -- both can be tuned I think
                ),
                config,
                () -> { 
                    Optional<Alliance> alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == Alliance.Red; //0 = red, 1 = blue
                    }
                    return false;
                },
                this
            );
        }
    }

//Methods start here:

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false); //closed loop auto
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    // drive method used/required for path planner. Basically just a rewritten drive() (refer to above) method 
    public void driveWithChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true); //TODO: may be worth it to check out closedLoop, especially for auto. light research indicates it may be more precise (but less responsive to input)?
        }
    }

    //Follows path that assumes starting pose is robot's current pose (by RESETTING the robots odometry to be the start pose of the path)
    public Command followPathCommand(String pathName) {
        try {
            SmartDashboard.putNumber("RobotPoseX before path start", this.getPose().getX());
            SmartDashboard.putNumber("RobotPoseY before path start", this.getPose().getY());
            SmartDashboard.putNumber("RobotPoseRotation before path start", this.getPose().getRotation().getDegrees());

            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            
            Pose2d bluePathStartingPose = path.getStartingHolonomicPose().get(); // starting pose of path, defaults to blue side coordinates
            
            if (DriverStation.getAlliance().get() == Alliance.Red) { // if red alliance, flip the path starting pose
                this.resetPose(FlippingUtil.flipFieldPose(bluePathStartingPose));
            } else  {
                this.resetPose(path.getStartingHolonomicPose().get());
            }

            SmartDashboard.putNumber("RobotPoseX AFTER", this.getPose().getX());
            SmartDashboard.putNumber("RobotPoseY AFTER", this.getPose().getY());
            SmartDashboard.putNumber("RobotPoseRotation AFTER", this.getPose().getRotation().getDegrees());

            Waypoint pathStartWaypoint = path.getWaypoints().get(0);
            SmartDashboard.putNumber("Path start  X", pathStartWaypoint.anchor().getX());
            SmartDashboard.putNumber("Path start  Y", pathStartWaypoint.anchor().getY());
            SmartDashboard.putNumber("Path start  Rotation", path.getStartingHolonomicPose().get().getRotation().getDegrees());

            Waypoint pathEndWaypoint = path.getWaypoints().get(path.getWaypoints().size() - 1);
            SmartDashboard.putNumber("Path end  X", pathEndWaypoint.anchor().getX());
            SmartDashboard.putNumber("Path end  Y", pathEndWaypoint.anchor().getY());
            SmartDashboard.putNumber("Path end  Rotation", path.getGoalEndState().rotation().getDegrees());

            FieldObject2d start = field.getObject("PathStart");
            start.setPose(pathStartWaypoint.anchor().getX(), pathStartWaypoint.anchor().getY(), path.getIdealStartingState().rotation());
            
            FieldObject2d end = field.getObject("PathEnd");
            end.setPose(pathEndWaypoint.anchor().getX(), pathEndWaypoint.anchor().getY(), path.getGoalEndState().rotation());

            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    // Barebones and default PathPlanner way of following a path. This means that if the robot is not at the start pose of the path,
    // it will attempt to move there (this is done by PathPlanner). Therefore, ONLY USE if you know robot has a pose and it is at/extremely close to start pose
    public Command followPathCommandNoReset(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    //Follows a path where end pose does not change, but it starts at the robot's current pose. Does not retain event markers (if applicable)
    public Command followPathCommandRobotStartingPose(String pathName) {
        try {
            PathPlannerPath originalPath = PathPlannerPath.fromPathFile(pathName);
            PathConstraints constraints = originalPath.getGlobalConstraints(); // just use original path's constraints by default. if this doesn't work then can reconstruct

            List<Waypoint> newWaypoints = originalPath.getWaypoints();
            newWaypoints.set(0, PathPlannerPath.waypointsFromPoses(this.getPose()).get(0)); // turns robot's current pose into waypoint, sets it as first waypoint

            PathPlannerPath path = new PathPlannerPath(
                newWaypoints, 
                constraints, 
                null, 
                originalPath.getGoalEndState()
            );
            
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    //Follows path relative to robot's current pose (shifts all poses and states to accomodate). Different than just changing the starting pose to Robot and keeping end same (as above)
    public Command followPathCommandRobotRelative(String pathName) {
        try {
            PathPlannerPath originalPath = PathPlannerPath.fromPathFile(pathName);

            PathConstraints constraints = originalPath.getGlobalConstraints(); // just use original path's constraints by default. if this doesn't work then can reconstruct
            IdealStartingState originalStartingState = originalPath.getIdealStartingState();
            GoalEndState originalGoalEndState = originalPath.getGoalEndState();
            
            Pose2d pathInitialPose = originalPath.getStartingHolonomicPose().get(); 
            List<Pose2d> newPoses = new ArrayList<>();
            newPoses.add(this.getPose()); // start with current robot pose

            int i = 0;
            for (Pose2d pose : originalPath.getPathPoses()) {
                if (i > 0) { // skip first pose since it has already been added
                    newPoses.add(this.getPose().transformBy(pose.minus(pathInitialPose))); 
                }

                i++;
            }

            PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(newPoses),
                constraints, 
                new IdealStartingState(originalStartingState.velocity(), originalStartingState.rotation().plus(getHeading().minus(pathInitialPose.getRotation()))), 
                new GoalEndState(originalGoalEndState.velocity(), originalGoalEndState.rotation().plus(getHeading().minus(pathInitialPose.getRotation())))
            );

            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    public ChassisSpeeds getChassisSpeeds() {
        SwerveModuleState[] states = getModuleStates();
        ChassisSpeeds fieldRelFromStates = Constants.Swerve.swerveKinematics.toChassisSpeeds(states);
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelFromStates, getHeading());
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        //return Rotation2d.fromDegrees(gyro.getYaw().getValue());
       return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void getLLPose() {
            // turn on the LED,  3 = force on
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
        // s_Swerve.zeroHeading(); //added this to fix the targeting going the wrong way

        //tv =1 means Limelight sees a target
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()  && (tv == 1)) { //have alliance color and see target
            if (ally.get() == Alliance.Red){
            poseLL = LimelightHelpers.getBotPose2d_wpiRed("limelight");
            //  s_Swerve.resetPose(poseLL); //do this later in ResetPose command
            }
            if (ally.get() == Alliance.Blue){
            poseLL = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
            // s_Swerve.resetPose(poseLL); //do this later in ResetPose command
            }   
        }
        //else do nothing
    }

    public void getTargetPose(Pose2d targetPose) {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.get() == Alliance.Blue){
            this.targetPose = new Pose2d(targetPose.getX(), targetPose.getY(), targetPose.getRotation().plus(new Rotation2d(Math.PI))); //do this later in ResetPose command
        }
        if (ally.get() == Alliance.Red){ // not sure why but we needed to swap these
            this.targetPose = targetPose; //do this later in ResetPose command
        }   

        // this.resetPose(targetPose);

    }

    public void resetFieldPoseWithTarget() {
        resetPose(targetPose);
    }

    public void resetLLPose() {
        if (poseLL != null) {
            resetPose(poseLL);
        }
    }

public Trajectory getTargetingTrajectory(double fwdDist1, double sideDist1, double turnAngle1, double fwdDist2, double sideDist2, double turnAngle2, boolean reversed) {
    double deltaFwd = fwdDist2-fwdDist1;
    double deltaSide = sideDist2 - sideDist1;
    double deltaAngle = turnAngle2- turnAngle1;

        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
             new Pose2d(fwdDist1, sideDist1, new Rotation2d((turnAngle1))),
            // Pass through these interior waypoints
            List.of(
                   new Translation2d((fwdDist1+0.05*deltaFwd), (sideDist1+0.05*deltaSide)), 
                   new Translation2d((fwdDist1+0.1*deltaFwd), (sideDist1+0.1*deltaSide)),
                   new Translation2d((fwdDist1+0.15*deltaFwd), (sideDist1+0.15*deltaSide)),
                   new Translation2d((fwdDist1+0.2*deltaFwd), (sideDist1+0.2*deltaSide)), 
                   new Translation2d((fwdDist1+0.25*deltaFwd), (sideDist1+0.25*deltaSide)),
                   new Translation2d((fwdDist1+0.3*deltaFwd), (sideDist1+0.3*deltaSide)),
                   new Translation2d((fwdDist1+0.35*deltaFwd), (sideDist1+0.35*deltaSide)), 
                   new Translation2d((fwdDist1+0.4*deltaFwd), (sideDist1+0.4*deltaSide)),
                   new Translation2d((fwdDist1+0.45*deltaFwd), (sideDist1+0.45*deltaSide)), 
                   new Translation2d((fwdDist1+0.5*deltaFwd), (sideDist1+0.5*deltaSide)),
                   new Translation2d((fwdDist1+0.55*deltaFwd), (sideDist1+0.55*deltaSide)),
                   new Translation2d((fwdDist1+0.6*deltaFwd), (sideDist1+0.6*deltaSide)), 
                   new Translation2d((fwdDist1+0.65*deltaFwd), (sideDist1+0.65*deltaSide)),
                   new Translation2d((fwdDist1+0.7*deltaFwd), (sideDist1+0.7*deltaSide)),
                   new Translation2d((fwdDist1+0.75*deltaFwd), (sideDist1+0.75*deltaSide)), 
                   new Translation2d((fwdDist1+0.8*deltaFwd), (sideDist1+0.8*deltaSide)),
                   new Translation2d((fwdDist1+0.85*deltaFwd), (sideDist1+0.85*deltaSide)), 
                   new Translation2d((fwdDist1+0.9*deltaFwd), (sideDist1+0.9*deltaSide)),
                   new Translation2d((fwdDist1+0.95*deltaFwd), (sideDist1+0.95*deltaSide))
                   ),  
            // End here
            new Pose2d(fwdDist2, sideDist2, new Rotation2d((turnAngle2 + Math.PI))), //add 180 because target and robot are facing opposite directions
            config);
        return exampleTrajectory;
    }

    public void MegaTag2UpdateOdometry() {
        /* Replaced below with m_poseEstimator.update(getGyroYaw(), getModulePositions()); as done in periodic for swerve odometry
        m_poseEstimator.update(
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                mSwerveMods[0].getPosition(), //front left
                mSwerveMods[1].getPosition(), //front right
                mSwerveMods[2].getPosition(), //back left
                mSwerveMods[3].getPosition()  //back right
            });
        */   m_poseEstimator.update(getGyroYaw(), getModulePositions());

        boolean useMegaTag2 = true; //set to false to use MegaTag1
        boolean doRejectUpdate = false;
        // evaluating which Megatag one or two to use based on above boolean value and 
        // only incorporate Limelight's estimates when more than one tag is visible (tagcount >= 1)
        if(useMegaTag2 == false)
        {
          LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
          if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
          {
            if(mt1.rawFiducials[0].ambiguity > .7) { doRejectUpdate = true; }
            if(mt1.rawFiducials[0].distToCamera > 3) { doRejectUpdate = true; }
          }
          if(mt1.tagCount == 0) { doRejectUpdate = true; }
          if(!doRejectUpdate) {     // if doRejectUpdate is false (or NOT true), then update the pose estimator
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
            m_poseEstimator.addVisionMeasurement(
                mt1.pose,
                mt1.timestampSeconds);
          }
        }
        else if (useMegaTag2 == true)
        {   // only incorporate Limelight's estimates when more than one tag is visible (tagcount >= 1)
            LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            if(Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
            {
                doRejectUpdate = true;
            }
            if(mt2.tagCount == 0)
            {
                doRejectUpdate = true;
            }
            if(!doRejectUpdate)   // if doRejectUpdate is false (or NOT true), then update the pose estimator
            {
                m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
                m_poseEstimator.addVisionMeasurement(
                    mt2.pose,
                    mt2.timestampSeconds);
          }
        }

        // should be functionally equivalent to everything above, just resetting swerveOdometry variable to the value of the pose estimator
        // because it is more commonly used in here
        this.resetPose(m_poseEstimator.getEstimatedPosition());
    }

    @Override
    public void periodic(){
      //  SmartDashboard.putNumber("limelight standoff fwd", LimelightHelpers.getTargetPose_CameraSpace("limelight")[2]);

    //    swerveOdometry.update(getGyroYaw(), getModulePositions());
    MegaTag2UpdateOdometry();
       SmartDashboard.putNumber("RobotPoseX", swerveOdometry.getPoseMeters().getX());
       SmartDashboard.putNumber("RobotPoseY", swerveOdometry.getPoseMeters().getY());
       SmartDashboard.putNumber("RobotPoseX (Estimator)", m_poseEstimator.getEstimatedPosition().getX());
       SmartDashboard.putNumber("RobotPoseY (Estimator)", m_poseEstimator.getEstimatedPosition().getY());
       field.setRobotPose(this.getPose());
    //    System.out.println(swerveOdometry.getPoseMeters().getX() + " " + swerveOdometry.getPoseMeters().getY() + " Rotation: " + swerveOdometry.getPoseMeters().getRotation().getDegrees());

        //for(SwerveModule mod : mSwerveMods){
         // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder degrees", mod.getCANcoder().getDegrees());
          //SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle degrees", mod.getPosition().angle.getDegrees());
          // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
          //Can't use m/s in the key!! SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity m/s", mod.getState().speedMetersPerSecond);
         //}
    
         /* 
       poseAngle = LimelightHelpers.getTargetPose_CameraSpace("limelight")[5];
       SmartDashboard.putNumber("TargetingAngle in swerve: ", poseAngle);
       poseForwardDistance = LimelightHelpers.getTargetPose_ameraSpace("limelight")[2];
      SmartDashboard.putNumber("TargetingForwardDistance in swerve: ", poseForwardDistance / 0.0254);
      poseSideDistance = LimelightHelpers.getTargetPose_CameraSpace("limelight")[0];
       SmartDashboard.putNumber("TargetingSideDistance in swerve: ", poseSideDistance / 0.0254);
       */
     }

}       

