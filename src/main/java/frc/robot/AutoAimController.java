package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoAimController {
    public enum AutoAimLocation {
        LL, LM, LR, ML, MM, MR, RL, RM, RR,
        R_LOAD, L_LOAD
    };

    private PIDController mXController, mYController;
    private ProfiledPIDController mThetaController;
    private TrajectoryConfig mTrajectoryConfig;
    private Pose2d mPoseError = new Pose2d();
    private Rotation2d mRotationError = new Rotation2d();
    private Timer mTimer = new Timer();

    private Trajectory mTrajectory;
    private Rotation2d mEndHeading;

    private Field2d field = new Field2d();

    private boolean mEnabled = false;

    /**
     * Create new Auto Aim Controller
     * 
     * @param xController x PID controller (meters)
     * @param yController y PID controller (meters)
     * @param thetaController theta PID controller (radians)
     */
    public AutoAimController(
        PIDController xController, 
        PIDController yController, 
        ProfiledPIDController thetaController, 
        TrajectoryConfig trajectoryConfig
    ) {
        mXController = xController;
        mYController = yController;
        mThetaController = thetaController;
        mTrajectoryConfig = trajectoryConfig;

        mThetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public Field2d getField() {
        return field;
    }

    /**
     * Set target points (quintic hermite spline)
     * 
     * @param points points along the path
     * @param endHeading end heading of chassis
     */
    public void setTarget(List<Pose2d> points, Rotation2d endHeading) {
        Trajectory traj = TrajectoryGenerator.generateTrajectory(
            points, 
            mTrajectoryConfig
        );

        field.getObject("traj").setTrajectory(traj);

        followPath(traj, endHeading);
    }

    /**
     * Set target points (cubic hermite spline)
     * 
     * @param startPose starting position
     * @param interPoints intermediate position
     * @param endPose end position
     * @param endHeading end heading
     */
    public void setTarget(Pose2d startPose, List<Translation2d> interPoints, Pose2d endPose, Rotation2d endHeading) {
        Trajectory traj = TrajectoryGenerator.generateTrajectory(
            startPose, 
            interPoints, 
            endPose, 
            mTrajectoryConfig
        );
        field.getObject("traj").setTrajectory(traj);
        followPath(traj, endHeading);
    }

    /**
     * Set target location
     * 
     * @param startPose starting position
     * @param location desired auto aim location
     */
    public void setTarget(Pose2d startPose, AutoAimLocation location) {
        boolean longPath = startPose.getX() > 2.85;
        boolean ultraLongPath = startPose.getX() > 5.5;
        boolean left = startPose.getY() > 3;
        Pose2d start, end;
        List<Translation2d> interPoints = new ArrayList<Translation2d>();

        Translation2d leftLongInterPoint = new Translation2d(2.6, 4.5);
        Translation2d rightLongInterPoint = new Translation2d(2.65, 1.35);
        Translation2d leftUltraInterPoint = new Translation2d(4.5, 4.7);
        Translation2d rightUltraInterPoint = new Translation2d(4.5, 1.15);

        start = new Pose2d(startPose.getX(), startPose.getY(), Rotation2d.fromDegrees(180));
        end = new Pose2d(2.323, getYOfAutoAimLocation(location), Rotation2d.fromDegrees(180));
        if(!longPath) {
            start = new Pose2d(start.getX(), start.getY(), new Rotation2d(end.getX() - start.getX(), end.getY() - start.getY()));
        }

        if(ultraLongPath) {
            if(left) {
                interPoints.add(leftUltraInterPoint);
                start = new Pose2d(start.getX(), start.getY(), new Rotation2d(leftUltraInterPoint.getX() - start.getX(), leftUltraInterPoint.getY() - start.getY()));
            } else {
                interPoints.add(rightUltraInterPoint);
                start = new Pose2d(start.getX(), start.getY(), new Rotation2d(rightUltraInterPoint.getX() - start.getX(), rightUltraInterPoint.getY() - start.getY()));
            }
        } 

        if(longPath) {
            if(left) {
                interPoints.add(leftLongInterPoint);
            } else {
                interPoints.add(rightLongInterPoint);
            }
        } 

        setTarget(start, interPoints, end, mEndHeading);
    }

    /**
     * Set enabled state of controller
     * 
     * @param enabled enabled
     */
    public void enable(boolean enabled) {
        mEnabled = enabled;
    }

    /** Update Controller */
    public void update() {
        if(!mEnabled) return;
        Trajectory.State currState = mTrajectory.sample(mTimer.get());
        field.setRobotPose(currState.poseMeters.getX(), currState.poseMeters.getY(), mEndHeading);
        SmartDashboard.putData(field);
        // ChassisSpeeds speeds = calculate(mSwerve.getPose(), currState, mEndHeading);
        // mSwerve.setOpenLoopSpeeds(speeds);
    }

    /**
     * Get positional error
     * 
     * @return positional error (meters)
     */
    public Pose2d getError() {
        return new Pose2d(mPoseError.getX(), mPoseError.getY(), mRotationError);
    }

    /**
     * Set tolerance
     * 
     * @param tolerance tolerance (meters & radians)
     */
    public void setTolerance(Pose2d tolerance) {
        mXController.setTolerance(tolerance.getX());
        mYController.setTolerance(tolerance.getY());
        mThetaController.setTolerance(tolerance.getRotation().getRadians());
    }

    /**
     * Check if robot is at target
     * 
     * @return robot at target
     */
    public boolean atTarget() {
        return mXController.atSetpoint() && mYController.atSetpoint() && mThetaController.atSetpoint();
    }

    /**
     * Calculate output speeds
     * 
     * @param currPose current robot pose (meters)
     * @param trajState desired trajectory state (meters)
     * @param desiredHeading desired final heading
     * @return output chassis speeds
     */
    private ChassisSpeeds calculate(Pose2d currPose, Trajectory.State trajState, Rotation2d desiredHeading) {
        double xFF = trajState.velocityMetersPerSecond * trajState.poseMeters.getRotation().getCos();
        double yFF = trajState.velocityMetersPerSecond * trajState.poseMeters.getRotation().getSin();
        double thetaFF =
            mThetaController.calculate(
                currPose.getRotation().getRadians(), desiredHeading.getRadians());
    
        mPoseError = trajState.poseMeters.relativeTo(currPose);
        mRotationError = desiredHeading.minus(currPose.getRotation());

        double xFeedback = mXController.calculate(currPose.getX(), trajState.poseMeters.getX());
        double yFeedback = mYController.calculate(currPose.getY(), trajState.poseMeters.getY());
    
        // Return next output.
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback, 
            yFF + yFeedback, 
            thetaFF, 
            currPose.getRotation()
        );
    }

    /**
     * Follow Trajectory
     * 
     * @param trajectory desired trajectory
     * @param endHeading desired end heading
     */
    private void followPath(Trajectory trajectory, Rotation2d endHeading) {
        mTimer.reset();
        mTimer.start();
        mTrajectory = trajectory;
        mEndHeading = endHeading;
    }

    private double getYOfAutoAimLocation(AutoAimLocation location) {
        switch(location) {
            case LL:
                return 0;
            case LM:
                return 4.5;
            case LR:
                return 4;
            case ML:
                return 3.5;
            case MM:
                return 3;
            case MR:
                return 2.5;
            case RL:
                return 2;
            case RM:
                return 1.5;
            case RR:
                return 1;
            default:
                return 0;
        }
    }
}
