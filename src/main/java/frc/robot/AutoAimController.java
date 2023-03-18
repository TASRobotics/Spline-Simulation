package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoAimController {
    public enum AutoAimLocation {
        LL, LM, LR, ML, MM, MR, RL, RM, RR,
        R_LOAD, L_LOAD
    };

    public enum LoadingLocation {
        WALL_LOAD, SCORING_LOAD
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

    public void setTarget(Pose2d startPose, LoadingLocation location, Alliance alliance) {
        List<Translation2d> interPoints = new ArrayList<Translation2d>();

        Rotation2d blueEndHeading = new Rotation2d(180);
        Rotation2d redEndHeading = new Rotation2d(0);

        double fieldLengthMeters = 16.5; 
        double wallY = 7.2;
        double scoringY = 6.0;

        double blueThresholdX = 12.0;
        double blueFinalX = 14.6;
        double finalY = location == LoadingLocation.WALL_LOAD ? wallY : scoringY;

        Pose2d endPose = new Pose2d(blueFinalX, finalY, new Rotation2d(0));
        if(alliance == Alliance.Red) {
            endPose = new Pose2d(fieldLengthMeters - blueFinalX, finalY, new Rotation2d(180));
        }

        // if(alliance == Alliance.Blue){
        //     if(startPose.getX() < blueThresholdX) {
        //         interPoints.add(new Translation2d(blueThresholdX, endPose.getY()));
        //     }
        // } else if(alliance == Alliance.Red) {
        //     if(startPose.getX() > fieldLengthMeters - blueThresholdX) {
        //         interPoints.add(new Translation2d(blueThresholdX, endPose.getY()));
        //     }
        // }

        Rotation2d endHeading = alliance == Alliance.Blue ? blueEndHeading : redEndHeading;
        setTarget(startPose, interPoints, endPose, endHeading);
    }

    /**
     * Set target location
     * 
     * @param startPose starting position
     * @param location desired auto aim location
     */
    public void setTarget(Pose2d startPose, AutoAimLocation location, Alliance alliance) {
        double fieldLengthMeters = 16.5; 
        double blueLongThreshold = 2.85;
        double blueUltraThreshold = 5.5;
        double blueLeftThreshold = 3;
        double blueScoringX = 1.85;
        Translation2d blueLeftLongInterPoint = new Translation2d(2.6, 4.5);
        Translation2d blueRightLongInterPoint = new Translation2d(2.65, 1.35);
        Translation2d blueLeftUltraInterPoint = new Translation2d(4.5, 4.7);
        Translation2d blueRightUltraInterPoint = new Translation2d(4.5, 1.15);

        Rotation2d defaultRotation = alliance == Alliance.Blue ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);
        
        boolean longPath = startPose.getX() > blueLongThreshold;
        boolean ultraLongPath = startPose.getX() > blueUltraThreshold;
        boolean left = startPose.getY() > blueLeftThreshold;
        double scoringX = blueScoringX;

        Translation2d leftLongInterPoint = blueLeftLongInterPoint;
        Translation2d rightLongInterPoint = blueRightLongInterPoint;
        Translation2d leftUltraInterPoint = blueLeftUltraInterPoint;
        Translation2d rightUltraInterPoint = blueRightUltraInterPoint;
        if(alliance == Alliance.Red) {
            longPath = startPose.getX() < fieldLengthMeters - blueLongThreshold;
            ultraLongPath = startPose.getX() < fieldLengthMeters - blueUltraThreshold;
            left = startPose.getY() < blueLeftThreshold;

            leftLongInterPoint = new Translation2d(fieldLengthMeters - blueLeftLongInterPoint.getX(), blueRightLongInterPoint.getY());
            rightLongInterPoint = new Translation2d(fieldLengthMeters - blueLeftLongInterPoint.getX(), blueLeftLongInterPoint.getY());
            leftUltraInterPoint = new Translation2d(fieldLengthMeters - blueLeftUltraInterPoint.getX(), blueRightUltraInterPoint.getY());
            rightUltraInterPoint = new Translation2d(fieldLengthMeters - blueLeftUltraInterPoint.getX(), blueLeftUltraInterPoint.getY());

            scoringX = fieldLengthMeters - blueScoringX;
        }

        Pose2d start, end;
        List<Translation2d> interPoints = new ArrayList<Translation2d>();

        start = new Pose2d(startPose.getX(), startPose.getY(), defaultRotation);
        end = new Pose2d(scoringX, getYOfAutoAimLocation(location, alliance), defaultRotation);

        // maybe check
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

        Translation2d secondToLastPoint;
        if(interPoints.size() > 0) {
            secondToLastPoint = interPoints.get(0);
        } else {
            secondToLastPoint = new Translation2d(start.getX(), start.getY());
        }

        end = new Pose2d(end.getX(), end.getY(), new Rotation2d(end.getX() - secondToLastPoint.getX(), end.getY() - secondToLastPoint.getY()));

        int rotationMult = alliance == Alliance.Blue ? 1 : -1;
        if(secondToLastPoint.getY() > end.getY()) {
            end = new Pose2d(end.getX(), end.getY(), Rotation2d.fromDegrees(end.getRotation().getDegrees() - (30 * rotationMult)));
        } else if(secondToLastPoint.getY() < end.getY()) {
            end = new Pose2d(end.getX(), end.getY(), Rotation2d.fromDegrees(end.getRotation().getDegrees() + (30 * rotationMult)));
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

    private double getYOfAutoAimLocation(AutoAimLocation location, Alliance alliance) {
        switch(location) {
            case LL:
                return alliance == Alliance.Blue ? 0 : 1;
            case LM:
                return alliance == Alliance.Blue ? 4.5 : 1.5;
            case LR:
                return alliance == Alliance.Blue ? 4 : 2;
            case ML:
                return alliance == Alliance.Blue ? 3.5 : 2.5;
            case MM:
                return 3;
            case MR:
                return alliance == Alliance.Blue ? 2.5 : 3.5;
            case RL:
                return alliance == Alliance.Blue ? 2 : 4;
            case RM:
                return alliance == Alliance.Blue ? 1.5 : 4.5;
            case RR:
                return alliance == Alliance.Blue ? 1 : 0;
            default:
                return 0;
        }
    }
}
