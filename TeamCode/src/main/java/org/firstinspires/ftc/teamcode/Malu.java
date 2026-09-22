package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp
public class Malu extends LinearOpMode {

    DcMotor intake;
    DcMotor midtake;


    @Override
    public void runOpMode() {

        intake = hardwareMap.get(DcMotor.class,"intake");
        midtake = hardwareMap.get(DcMotor.class, "midtake");


        waitForStart();


        while(opModeIsActive()) {

            intake.setPower(0.8);
            midtake.setPower(0.8);

        }

    }


}


