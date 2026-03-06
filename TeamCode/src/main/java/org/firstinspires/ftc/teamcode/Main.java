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

            RobotHub.pinpoint.update();

            mapx.update();


            telemetry.addData("Alliance", allianceSide);

            telemetry.addData("Launcher Velocity", RobotHub.launcher.getVelocity());
            telemetry.addData("Launcher Velocity Filtered", RobotHub.launcher.valueFiltered);
            telemetry.update();
        }
    }
}
