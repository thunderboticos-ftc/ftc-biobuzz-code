package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class Sample extends OpMode {

    private RobotMemory robotMemory;
    private Follower follower;
    private Timer pathTimer, opModeTimer, generalTimer, waitTimer, waitForShootTimer;
    private int launchs = 0;
    private int shoots = 0;

    RobotHub robotHub;

    public enum PathState {
        // START POSITION -> END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        FIRST_RELOAD,
        FIRST_GO_TO_RELOAD,
        GO_TO_SHOOT,
    };

    PathState pathState;

    private final Pose startPose = new Pose(22.78747940691927, 128.51400329489292, Math.toRadians(143));
    private final Pose shootPose = new Pose(55.13509060955519, 92.51235584843496, Math.toRadians(136));
    private final Pose artefactsCollect1StartPose = new Pose(58.5, 90.51235584843496, Math.toRadians(180));
    private final Pose artefactsCollect1FinalPose = new Pose(17.5502471169687, 90.80724876441515, Math.toRadians(180));


    private PathChain driveStartPosShootPos;
    private PathChain driveShootPosFirstReloadStartPos;
    private PathChain driveStartFirstReloadPosEndFirstReloadPos;
    private PathChain driveEndFirstReloadPosShootPos;


    public void buildPaths() {
        // Put in coordinates for starting pose > ending pose

        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosFirstReloadStartPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, artefactsCollect1StartPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), artefactsCollect1StartPose.getHeading())
                .build();

        driveStartFirstReloadPosEndFirstReloadPos = follower.pathBuilder()
                .addPath(new BezierLine(artefactsCollect1StartPose, artefactsCollect1FinalPose))
                .setLinearHeadingInterpolation(artefactsCollect1StartPose.getHeading(), artefactsCollect1FinalPose.getHeading())
                .build();

        driveEndFirstReloadPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(artefactsCollect1FinalPose, shootPose))
                .setLinearHeadingInterpolation(artefactsCollect1FinalPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                RobotHub.launcher.setVelocityPIDFOp(12.8);
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD); // Reset timer while change the path
                waitForShootTimer.resetTimer();
                break;

            case SHOOT_PRELOAD:
                RobotHub.launcher.setVelocityPIDFOp(12.8);
                if(!follower.isBusy()) {
                    if(waitForShootTimer.getElapsedTimeSeconds() > 1.5) {
                        if(launchs < 5) {
                            if (launchs > 1) {
                                RobotHub.launcher.runToTransporter(true);
                            } else {
                                RobotHub.launcher.runToTransporter(false);
                            }
                            if (generalTimer.getElapsedTimeSeconds() >= 1) {
                                launchs++;
                                RobotHub.launcher.transportToShooter(true);
                                generalTimer.resetTimer();
                            } else {
                                RobotHub.launcher.transportToShooter(false);
                            }
                        } else {
                            shoots++;
                            launchs = 0;
                            if (shoots == 1)
                                setPathState(PathState.FIRST_GO_TO_RELOAD);
                        }
                    }
                }

                break;

            case FIRST_GO_TO_RELOAD:
                RobotHub.launcher.transportToShooter(false);
                RobotHub.launcher.setVelocityPIDFOp(12.8);
                RobotHub.launcher.runToTransporter(true);
                if(!follower.isBusy()) {
                    follower.followPath(driveShootPosFirstReloadStartPos, 0.35, true);
                    setPathState(PathState.FIRST_RELOAD); // Reset timer while change the path
                    waitTimer.resetTimer();
                }

                break;

            case FIRST_RELOAD:
                RobotHub.launcher.transportToShooter(false);
                RobotHub.launcher.runToTransporter(true);
                if(!follower.isBusy()) {
                    if(waitTimer.getElapsedTimeSeconds() > 2) {
                        follower.followPath(driveStartFirstReloadPosEndFirstReloadPos, 0.25, false);
                        setPathState(PathState.GO_TO_SHOOT); // Reset timer while change the path
                    }
                }
                break;

            case GO_TO_SHOOT:
                RobotHub.launcher.setVelocityPIDFOp(12.8);
                if(!follower.isBusy()) {
                    follower.followPath(driveEndFirstReloadPosShootPos,0.6, true);
                    setPathState(PathState.SHOOT_PRELOAD); // Reset timer while change the path
                    waitForShootTimer.resetTimer();
                }

                break;

            default:
                telemetry.addLine("NO STATE COMMANDED");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {

        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        waitTimer = new Timer();
        waitForShootTimer = new Timer();
        opModeTimer.resetTimer();
        waitTimer.resetTimer();
        waitForShootTimer.resetTimer();


        RobotHub.reset();
        robotHub = RobotHub.getInstance(hardwareMap);
        robotMemory = RobotMemory.INSTANCE;

        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
        generalTimer = new Timer();
        generalTimer.resetTimer();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {

        follower.update();
        statePathUpdate();
        robotMemory.autoFinalPose = follower.getPose();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());



    }
}
