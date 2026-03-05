package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class teste extends LinearOpMode {

    DigitalChannel digital;

    @Override
    public void runOpMode() throws InterruptedException {
        digital = hardwareMap.get(DigitalChannel.class, "digital");

        digital.setMode(DigitalChannel.Mode.OUTPUT);

        waitForStart();

        while (opModeIsActive()) {
            digital.setState(true);
            sleep(2500);
            digital.setState(false);
            sleep(2500);
        }
    }
}
