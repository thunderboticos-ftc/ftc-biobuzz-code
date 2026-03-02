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
public class BlueAutonomousCloseMain extends OpMode {

    private double timeForShoot;


    private RobotHub robotHub;
    private RobotMemory robotMemory;

    private Mapx mapx;

    private int cycle;



    private Follower follower;
    private Timer pathTimer, opModeTimer, waitForShoot, waitForStopAbsorbing;





    private enum PathState {
        START_POS_TO_SHOOT_POS,
        SHOOT_PRELOAD,
        START_FIRST_COLLECT_TO_END_FIRST_COLLECT,
        END_FIRST_COLLECT_TO_SHOOT_POS,
        END_SECOND_COLLECT_TO_START_SECOND_COLLECT,
        START_SECOND_COLLECT_TO_END_SECOND_COLLECT,
        START_SECOND_COLLECT_TO_SHOOT_POS,
        START_THIRTY_COLLECT_TO_END_THIRTY_COLLECT,
        END_THIRTY_COLLECT_TO_SHOOT_POS,

    }

    PathState pathState;



    private final Pose startPose = new Pose(22.1, 126.7, Math.toRadians(149));
    private final Pose shootPose = new Pose(49, 93, Math.toRadians(140.5));
    private final Pose startFirstCollect = new Pose(53, 92, Math.toRadians(180));
    private final Pose endFirstCollect = new Pose(21, 92, Math.toRadians(180));
    private final Pose startSecondCollect = new Pose(53, 63, Math.toRadians(185)); // y -> 66
    private final Pose endSecondCollect = new Pose(14.5, 63, Math.toRadians(185)); // y -> 66
    private final Pose startThirtyCollect = new Pose(53, 40, Math.toRadians(180));
    private final Pose endThirtyCollect = new Pose(16, 40, Math.toRadians(180));
    private final Pose finalPos = new Pose(50, 72, Math.toRadians(135));



    private PathChain startPosToShootPos;
    private PathChain shootPosToStartFirstCollectPos;
    private PathChain startFirstCollectPosToEndFirstCollectPos;
    private PathChain endFirstCollectPosToShootPos;
    private PathChain shootPosToStartSecondCollectPos;
    private PathChain startSecondCollectPosToEndSecondCollectPos;
    private PathChain endSecondCollectPosToStartSecondCollectPos;
    private PathChain startSecondCollectPosToShootPos;
    private PathChain shootPosToStartThirtyCollectPos;
    private PathChain startThirtyCollectPosToEndThirtyCollectPos;
    private PathChain endThirtyCollectPosToShootPos;
    private PathChain shootPosToFinalPos;


    public void buildPaths() {
        // Put in coordinates for starting pose > ending pose

        startPosToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        shootPosToStartFirstCollectPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, startFirstCollect))
                .setLinearHeadingInterpolation(shootPose.getHeading(), startFirstCollect.getHeading())
                .build();

        startFirstCollectPosToEndFirstCollectPos = follower.pathBuilder()
                .addPath(new BezierLine(startFirstCollect, endFirstCollect))
                .setLinearHeadingInterpolation(startFirstCollect.getHeading(), endFirstCollect.getHeading())
                .build();

        endFirstCollectPosToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(endFirstCollect, shootPose))
                .setLinearHeadingInterpolation(endFirstCollect.getHeading(), shootPose.getHeading())
                .build();

        shootPosToStartSecondCollectPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, startSecondCollect))
                .setLinearHeadingInterpolation(shootPose.getHeading(), startSecondCollect.getHeading())
                .build();

        startSecondCollectPosToEndSecondCollectPos = follower.pathBuilder()
                .addPath(new BezierLine(startSecondCollect, endSecondCollect))
                .setLinearHeadingInterpolation(startSecondCollect.getHeading(), endSecondCollect.getHeading())
                .build();

        endSecondCollectPosToStartSecondCollectPos = follower.pathBuilder()
                .addPath(new BezierLine(endSecondCollect, startSecondCollect))
                .setLinearHeadingInterpolation(endSecondCollect.getHeading(), startSecondCollect.getHeading())
                .build();

        startSecondCollectPosToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startSecondCollect, shootPose))
                .setLinearHeadingInterpolation(startSecondCollect.getHeading(), shootPose.getHeading())
                .build();

        shootPosToStartThirtyCollectPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, startThirtyCollect))
                .setLinearHeadingInterpolation(shootPose.getHeading(), startThirtyCollect.getHeading())
                .build();

        startThirtyCollectPosToEndThirtyCollectPos = follower.pathBuilder()
                .addPath(new BezierLine(startThirtyCollect, endThirtyCollect))
                .setLinearHeadingInterpolation(startThirtyCollect.getHeading(), endThirtyCollect.getHeading())
                .build();

        endThirtyCollectPosToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(endThirtyCollect, shootPose))
                .setLinearHeadingInterpolation(endThirtyCollect.getHeading(), shootPose.getHeading())
                .build();

        shootPosToFinalPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, finalPos))
                .setLinearHeadingInterpolation(shootPose.getHeading(), finalPos.getHeading())
                .build();
    }








    private void statePathUpdate() {
        switch(pathState) {
            case START_POS_TO_SHOOT_POS:
                // Going to shootPos

                follower.followPath(startPosToShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                waitForShoot.resetTimer();

                break;

            case SHOOT_PRELOAD:
                if(!follower.isBusy()) {
                    if(waitForShoot.getElapsedTimeSeconds() > 1)
                        RobotHub.launcher.runIntakeAndMidtake(false);

                    switch(cycle) {
                        case 0:
                            timeForShoot = 3;
                            break;

                        case 1:
                            timeForShoot = 2.5;
                            break;

                        case 2:
                            timeForShoot = 2.5;
                            break;

                        case 3:
                            timeForShoot = 2.75;
                            break;

                    }

                    if (waitForShoot.getElapsedTimeSeconds() > timeForShoot) {
                        RobotHub.launcher.shootArtefacts(true, 2.2);
                    }


                    if(RobotHub.launcher.shootsPerTime >= 5) {
                        RobotHub.launcher.shootArtefacts(false);
                        RobotHub.launcher.transportToShooter(false);

                        cycle++;

                        switch(cycle) {
                            case 1:
                                follower.followPath(shootPosToStartFirstCollectPos, 0.8, false);
                                setPathState(PathState.START_FIRST_COLLECT_TO_END_FIRST_COLLECT);
                                break;
                            case 2:
                                follower.followPath(shootPosToStartSecondCollectPos, false);
                                setPathState(PathState.START_SECOND_COLLECT_TO_END_SECOND_COLLECT);
                                break;

                            case 3:
                                follower.followPath(shootPosToStartThirtyCollectPos, false);
                                setPathState(PathState.START_THIRTY_COLLECT_TO_END_THIRTY_COLLECT);
                                break;

                            case 4:
                                follower.followPath(shootPosToFinalPos, false);
                                break;

                            default:
                                break;
                        }
                    }
                }

                break;

            case START_FIRST_COLLECT_TO_END_FIRST_COLLECT:
                RobotHub.launcher.transportToShooter(false);
                RobotHub.launcher.runAllIntake(true);
                if(!follower.isBusy()) {
                    follower.followPath(startFirstCollectPosToEndFirstCollectPos, 0.8, false);
                    setPathState(PathState.END_FIRST_COLLECT_TO_SHOOT_POS);
                }

                break;

            case END_FIRST_COLLECT_TO_SHOOT_POS:
                RobotHub.launcher.transportToShooter(false);
                RobotHub.launcher.runAllIntake(true);
                if(!follower.isBusy()) {
                    follower.followPath(endFirstCollectPosToShootPos, true);
                    setPathState(PathState.SHOOT_PRELOAD);
                    waitForShoot.resetTimer();
                }
                break;

            case START_SECOND_COLLECT_TO_END_SECOND_COLLECT:
                RobotHub.launcher.transportToShooter(false);
                RobotHub.launcher.runAllIntake(true);
                if(!follower.isBusy()) {
                    follower.followPath(startSecondCollectPosToEndSecondCollectPos, 0.7, false);
                    setPathState(PathState.END_SECOND_COLLECT_TO_START_SECOND_COLLECT);
                }

                break;

            case END_SECOND_COLLECT_TO_START_SECOND_COLLECT:
                RobotHub.launcher.transportToShooter(false);
                RobotHub.launcher.runAllIntake(true);
                if(!follower.isBusy()) {
                    follower.followPath(endSecondCollectPosToStartSecondCollectPos, 1, false);
                    setPathState(PathState.START_SECOND_COLLECT_TO_SHOOT_POS);
                }

                break;

            case START_SECOND_COLLECT_TO_SHOOT_POS:
                RobotHub.launcher.transportToShooter(false);
                RobotHub.launcher.runAllIntake(true);
                if(!follower.isBusy()) {
                    follower.followPath(startSecondCollectPosToShootPos, 1, true);
                    setPathState(PathState.SHOOT_PRELOAD);
                    waitForShoot.resetTimer();
                }

                break;

            case START_THIRTY_COLLECT_TO_END_THIRTY_COLLECT:
                RobotHub.launcher.transportToShooter(false);
                RobotHub.launcher.runAllIntake(true);
                if(!follower.isBusy()) {
                    follower.followPath(startThirtyCollectPosToEndThirtyCollectPos, 1, false);
                    setPathState(PathState.END_THIRTY_COLLECT_TO_SHOOT_POS);
                }

                break;

            case END_THIRTY_COLLECT_TO_SHOOT_POS:
                RobotHub.launcher.transportToShooter(false);
                RobotHub.launcher.runAllIntake(true);
                if(!follower.isBusy()) {
                    follower.followPath(endThirtyCollectPosToShootPos, true);
                    setPathState(PathState.SHOOT_PRELOAD);
                    waitForShoot.resetTimer();
                }

                break;
        }
    }





    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }



    @Override
    public void init() {
        pathState = PathState.START_POS_TO_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        waitForShoot = new Timer();
        waitForStopAbsorbing = new Timer();

        timeForShoot = 0;


        cycle = 0;


        opModeTimer.resetTimer();


        RobotHub.reset();
        robotHub = RobotHub.getInstance(hardwareMap);
        robotMemory = RobotMemory.INSTANCE;

        follower = Constants.createFollower(hardwareMap);

        mapx = new Mapx(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
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
        robotMemory.allianceSide = 0;
        mapx.update();

        //RobotHub.launcher.setVelocityPIDFOp(12.47);
        RobotHub.limelightCam.valueFilter();

        RobotHub.launcher.setVelocityBasedOnMapx(0, mapx, true);

        //RobotHub.launcher.setVelocityBasedOnCam(true, RobotHub.limelightCam);

        telemetry.addData("", RobotHub.limelightCam.valueFiltered);
    }
}
