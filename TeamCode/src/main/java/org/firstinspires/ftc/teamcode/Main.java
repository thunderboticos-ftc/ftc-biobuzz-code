package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class Main extends LinearOpMode {
    Mapx mapx;
    
    double[] initialPos;
    double initialAngle;

    int allianceSide;
    
    RobotHub robotHub;
    RobotMemory robotMemory;

    GamepadController gamepadController;
    TelemetryData telemetryData;


    @Override   
    public void runOpMode() {

        initialPos = new double[] {0, 0, 0};
        initialAngle = 0;

        RobotHub.reset();
        robotHub = RobotHub.getInstance(hardwareMap);

        robotHub.led.setState(false);

        mapx = new Mapx(hardwareMap);
        robotMemory = RobotMemory.INSTANCE;
        allianceSide = robotMemory.allianceSide;
        robotMemory.allianceSide = 0;
        robotMemory.autoFinalPose = null;

        gamepadController = new GamepadController(gamepad1, gamepad2, RobotHub.movement, RobotHub.launcher, RobotHub.limelightCam);
        telemetryData = new TelemetryData(telemetry, RobotHub.launcher, RobotHub.limelightCam);
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();  
        
        // run until the end of the match (driver presses STOP)
        while(opModeIsActive()) {
            gamepadController.moveUsingGamepad();
            gamepadController.launchUsingGamepad();
            //gamepadController.absorbUsingGamepad();
            gamepadController.transportBallToLauncher();
            gamepadController.keyStatesUpdate();
            gamepadController.correctAngleForTargetUsingGamepad(telemetry);
            gamepadController.launchUsingGamepadWithoutCam();
            gamepadController.ejectArtefacts();
            gamepadController.changeVelocityUsingGamepad();
            gamepadController.lockPosition();
            gamepadController.changeLauncherVelocityUsingGamepad();
            gamepadController.transportToShooter();
            gamepadController.rumbleOnDangerousAreas(allianceSide, mapx);
            gamepadController.goToEndLocal(allianceSide, mapx);
            gamepadController.launchUsingGamepadWithMapx(allianceSide, mapx);
            gamepadController.shootArtefactsUsingGamepad();

            RobotHub.launcher.updateLauncherVelocityAndRps();
            RobotHub.launcher.verifyLaunches();
            RobotHub.launcher.valueFilter();

            RobotHub.limelightCam.valueFilter();

            telemetry.addData("CamFilter", RobotHub.limelightCam.valueFiltered);
            telemetry.addData("Cam", RobotHub.limelightCam.getAprilTagDistance());
            telemetry.addData("DistanceMapx", mapx.getGoalDistance(allianceSide));
            telemetry.addData("Alliance", allianceSide);

            RobotHub.pinpoint.update();

            mapx.update();


            telemetry.addData("XM", mapx.positionMeters[0]);
            telemetry.addData("YM", mapx.positionMeters[1]);
            telemetry.addData("XI", mapx.positionInches[0]);
            telemetry.addData("YI", mapx.positionInches[1]);
            telemetry.addData("H", mapx.heading);
            telemetry.update();


        }
    }
}
