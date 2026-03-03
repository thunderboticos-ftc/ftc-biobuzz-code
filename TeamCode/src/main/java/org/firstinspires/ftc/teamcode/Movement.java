
package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Movement {
    // Variables Initialization


    // HardwareMap
    private final DcMotorEx motorLeftFront;
    private final DcMotorEx motorRightFront;
    private final DcMotorEx motorLeftBack;
    private final DcMotorEx motorRightBack;
    private final Odometry odometry;
    private final ControlHub controlHub;


    // Target Angle PID
    private double lastErrorTargetAngle;
    private double lastITargetAngle;


    // Global PIDLockIn Variables
    private final double[] posTargetLockIn;
    
    private double lastErrorXLockIn;
    private double lastErrorYLockIn;
    private double lastErrorAngleLockIn;
    private double lastIXLockIn;
    private double lastIYLockIn;
    private double lastIAngleLockIn;
    
    private double initialAngleLockIn;
    
    public boolean lockedIn;

    private double headingOffset;
    private final ElapsedTime timerPIDUpdate;

    // Pinpoint
    private final GoBildaPinpointDriver pinpoint;
    
    
    public Movement(DcMotorEx motorLeftFront, DcMotorEx motorRightFront, DcMotorEx motorLeftBack, DcMotorEx motorRightBack, Odometry odometry, ControlHub controlHub, GoBildaPinpointDriver pinpoint) {
        // Values Attributions

        
        // Motors Variables
        this.motorLeftFront = motorLeftFront;
        this.motorRightFront = motorRightFront;
        this.motorLeftBack = motorLeftBack;
        this.motorRightBack = motorRightBack;
        
        this.odometry = odometry;
        
        this.controlHub = controlHub;


        // Initial PID Target Angle Values
        lastErrorTargetAngle = 0;
        lastITargetAngle = 0;
        
        // Initial PIDLockIn Values
        posTargetLockIn = new double[2];
        
        lastErrorXLockIn = 0;
        lastErrorYLockIn = 0;
        lastErrorAngleLockIn = 0;
        lastIXLockIn = 0;
        lastIYLockIn = 0;
        lastIAngleLockIn = 0;
            
        lockedIn = false;

        headingOffset = 0;

        this.timerPIDUpdate = new ElapsedTime();
        timerPIDUpdate.startTime();

        // Pinpoint
        this.pinpoint = pinpoint;


        // Motors directions
        motorLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotor.Direction.FORWARD);
        motorRightBack.setDirection(DcMotor.Direction.REVERSE);
        
        resetEncoders();
    }
    
    public void correctRobotAngleForTarget(double power, double acceptableAngleVariation, Cam cam,  Telemetry telemetry) {
        // Get aprilTag tx value;
        double txValue = cam.getAprilTagResults()[0];

        if(txValue == -100) {
            return;
        }

        double target = 3.5;


        double correctionTargetAngle;


        double[] pidValuesTargetAngle;

        pidValuesTargetAngle = PIDUpdate(txValue, target, 0.02, 0.001, lastErrorTargetAngle, lastITargetAngle);

        correctionTargetAngle = pidValuesTargetAngle[0];
        lastErrorTargetAngle = pidValuesTargetAngle[1];
        lastITargetAngle = pidValuesTargetAngle[2];

        telemetry.addData("tx", txValue);

        runMotors(-correctionTargetAngle,
                correctionTargetAngle,
                -correctionTargetAngle,
                correctionTargetAngle);
    } 

    public void moveByVector(double[] vector, double turn, double power) {
        // Get the x and y axis
        double x = vector[0];
        double y = vector[1];
        
        // Get the needed power for each motor using the direction vector
        double leftFrontPower = (y + x + turn) * power;
        double rightFrontPower = (y - x - turn) * power;
        double leftBackPower = (y - x + turn) * power;
        double rightBackPower = (y + x - turn) * power;
        
        // Normalize to be between -1 and 1
        double max = Math.max(1.0,
                Math.max(Math.abs(leftFrontPower),
                Math.max(Math.abs(rightFrontPower),
                Math.max(Math.abs(leftBackPower),
                Math.abs(rightBackPower)))));
        
        leftFrontPower /= max;
        rightFrontPower /= max;
        leftBackPower /= max;
        rightBackPower /= max;
        
        // Run the motors
        runMotors(leftFrontPower,
                        rightFrontPower,
                        leftBackPower,
                        rightBackPower);
        
    }
    
    public void turn(double angle, double power) {
        // Initialize basics variable
        double initialAngle = controlHub.getIMUYawAngle();
        double relativeAngle = 0;
        
        // Checks if it really needs to turn
        if(angle == 0) {
            return;
        }
        
        // Do the turn        
        while((angle < 0)?(relativeAngle > angle):(relativeAngle < angle)) {
            relativeAngle = controlHub.getIMUYawAngle() - initialAngle;
            if(angle > 0)
                runMotors(power, -power, power, -power);
            else
                runMotors(-power, power, -power, power);
        }
        
        stopMotors();
    }
    
    public void turnWithCorrection(double angle, double power, double powerCorrection) {
        // First Turn
        double initialAngle = controlHub.getIMUYawAngle();
        turn(angle, power);
        
        // Gets the offset angle
        double angleAfterFirstTurn = controlHub.getIMUYawAngle() - initialAngle;
        double angleCorrection = angle - angleAfterFirstTurn;
        
        // Second turn
        turn(angleCorrection, powerCorrection);
    }
    
    public void moveMetersByVector(double meters, double[] vector, double power) {
        // Variable values attribution
        double distance = 0;
        double[] initialPos = new double[] {odometry.getPosX(), odometry.getPosY()};
        double[] relativePos = new double[2];
        
        while(distance < meters) {
            // Get the x and y axis offset
            relativePos[0] = odometry.getPosX() - initialPos[0];
            relativePos[1] = odometry.getPosY() - initialPos[1];
            
            // Calculates the total distance by Pythagorean theorem
            distance = Math.sqrt(Math.pow(relativePos[0], 2) + Math.pow(relativePos[1], 2));
            moveByVector(vector, 0, power);
        }
        
        stopMotors();
    }
    
    public void PID(double meters, double[] vector, double power) {
        // Gets the correction for the motor power proportional to the error angle
        double initialAngle = controlHub.getIMUYawAngle();
        
        double x = vector[0];
        double y = vector[1];
        
        // Get the needed power for each motor using the direction vector
        double leftFrontPower = (y + x) * power;
        double rightFrontPower = (y - x) * power;
        double leftBackPower = (y - x) * power;
        double rightBackPower = (y + x) * power;
        
        // Defines the initial base values
        double distance = 0;
        double[] initialPos = new double[] {odometry.getPosX(), odometry.getPosY()};
        double[] relativePos = new double[2];
        
        double[] pidValuesAngle;
        double correctionAngle;
        double lastErrorAngle = 0;
        double lastIAngle = 0;
        
        while(distance < meters) {
            // Get the x and y axis offset
            relativePos[0] = odometry.getPosX() - initialPos[0];
            relativePos[1] = odometry.getPosY() - initialPos[1];
            
            // Calculates the total distance by Pythagorean theorem
            distance = Math.sqrt(Math.pow(relativePos[0], 2) + Math.pow(relativePos[1], 2));
            
            // Applies PID on angle values
            pidValuesAngle = PIDUpdate(controlHub.getIMUYawAngle(), initialAngle, 0.02, 0.00025, lastErrorAngle, lastIAngle);
            correctionAngle = pidValuesAngle[0];
            lastErrorAngle = pidValuesAngle[1];
            lastIAngle = pidValuesAngle[2];
            
            // Corrections on motors
            runMotors(leftFrontPower + correctionAngle,
                rightFrontPower - correctionAngle,
                leftBackPower + correctionAngle,
                rightBackPower - correctionAngle);
        }
        
        stopMotors();
    }
    
    public double[] PIDUpdate(double value, double target, double KP, double KD, double lastError, double lastI) {
        // Base variables
        double KI = 0.0001;

        KI = 0;

        double IMax = 50;

        double dt = timerPIDUpdate.seconds();
        timerPIDUpdate.reset();

        double dtMin = 0.005;
        double dtMax = 0.1;

        dt = Math.max(dtMin, Math.min(dtMax, dt));


        // Manipulates the values and get the respective correction
        double error = target - value;

        double integral;

        if(Math.abs(error) < .25)
            integral = lastI;
        else
            integral = 0;

        double proportional = error * KP;
        integral = integral + error;

        integral = Math.max(-IMax, Math.min(IMax, integral));

        double ki = integral * KI;
        double derivative = error - lastError;
        double kd = derivative * KD;
        kd = kd / dt;
        double correction = proportional + ki + kd;

        return new double[] {correction, error, integral};
    }
    
    public void PIDMax(double meters, double[] vector, double power) {
        // Gets the correction for the motor power proportional to the error angle
        double initialAngle = controlHub.getIMUYawAngle();
        
        double x = vector[0];
        double y = vector[1];
        
        double abstractX = x;
        double abstractY = y;
        
        if(x < 0)
            abstractX *= -1;
        if(y < 0)
            abstractY *= -1;

        
        // Base values
        double distance = 0;
        double[] initialPos = new double[] {odometry.getPosX(), odometry.getPosY()};
        double[] relativePos = new double[2];
        
        double[] posTarget;
        double variation;
        
        double[] pidValuesX;
        double correctionX = 0;
        double lastErrorX = 0;
        double lastIX = 0;
        
        double[] pidValuesY;
        double correctionY = 0;
        double lastErrorY = 0;
        double lastIY = 0;

        double[] pidValuesAngle;
        double correctionAngle;
        double lastErrorAngle = 0;
        double lastIAngle = 0;

        while(distance < meters) {
            // Gets the x and y axis offset
            relativePos[0] = odometry.getPosX() - initialPos[0];
            relativePos[1] = odometry.getPosY() - initialPos[1];
            
            // Calculates the total distance by Pythagorean theorem
            distance = Math.sqrt(Math.pow(relativePos[0], 2) + Math.pow(relativePos[1], 2));
            
            // Analyses the derivative and get the posTarget based on it
            if(x != 0) {
                variation = y / x;
                posTarget = new double[] {relativePos[1] / variation, relativePos[0] * variation};
            } else {
                variation = 0;
                posTarget = new double[] {0, relativePos[1]};
            }
            
            // Verify if is a diagonal move
            if(variation != 0) {
                // Analyses what is more benefic to correct
                if(posTarget[0] - relativePos[0] > posTarget[1] - relativePos[1]) {
                    // PID values for the y axis
                    pidValuesY = PIDUpdate(relativePos[1], posTarget[1], 2, 0.5, lastErrorY, lastIY);
                    correctionY = pidValuesY[0];
                    lastErrorY = pidValuesY[1];
                    lastIY = pidValuesY[2];
                    
                } else {
                    // PID values for the x axis
                    pidValuesX = PIDUpdate(relativePos[0], posTarget[0], 2, 0.05, lastErrorX, lastIX);
                    correctionX = pidValuesX[0];
                    lastErrorX = pidValuesX[1];
                    lastIX = pidValuesX[2];
                }
            } else {
                // Correct based on the direction
                if(abstractX > abstractY) {
                    // PID values for the y axis
                    pidValuesY = PIDUpdate(relativePos[1], posTarget[1], 2, 0.5, lastErrorY, lastIY);
                    correctionY = pidValuesY[0];
                    lastErrorY = pidValuesY[1];
                    lastIY = pidValuesY[2];
                    
                } else {
                    // PID values for the x axis
                    pidValuesX = PIDUpdate(relativePos[0], posTarget[0], 2, 0.05, lastErrorX, lastIX);
                    correctionX = pidValuesX[0];
                    lastErrorX = pidValuesX[1];
                    lastIX = pidValuesX[2];
                }
            }
            
            // PID values for the angle
            pidValuesAngle = PIDUpdate(controlHub.getIMUYawAngle(), initialAngle,  0.02, 0.00025, lastErrorAngle, lastIAngle);
            correctionAngle = pidValuesAngle[0];
            lastErrorAngle = pidValuesAngle[1];
            lastIAngle = pidValuesAngle[2];
            
            // Correct the motor power
            runMotors(correctionAngle + correctionY + correctionX,
                    -correctionAngle + correctionY - correctionX,
                    correctionAngle + correctionY - correctionX,
                    -correctionAngle + correctionY + correctionX);

                
        }
        
        stopMotors();
    }
    
    public void PIDLockInOp(boolean started) {
        // Verify if its needed to lock in
        if(!started) {
            lockedIn = false;
            return;
        }

        double heading = -pinpoint.getHeading(AngleUnit.DEGREES);

        // Base variable values
        double[] pidValuesY;
        double[] pidValuesX;
        double[] pidValuesAngle;
        
        double correctionY;
        double correctionX;
        double correctionAngle;

        double angle = 0;

        double[] relativePos = new double[2];
        
        // Gets the first position when locked
        if(!lockedIn) {
            this.headingOffset = heading;
            initialAngleLockIn = AngleUnit.normalizeDegrees(heading - this.headingOffset);
            posTargetLockIn[0] = pinpoint.getPosition().getX(DistanceUnit.METER);
            posTargetLockIn[1] = pinpoint.getPosition().getY(DistanceUnit.METER);

            lockedIn = true;
        }

        angle = AngleUnit.normalizeDegrees(heading - this.headingOffset);

        // Updates the relative position
        relativePos[0] = pinpoint.getPosition().getX(DistanceUnit.METER);
        relativePos[1] = pinpoint.getPosition().getY(DistanceUnit.METER);

        
        
        // PID values for the y axis
        // P -> 1.3
        // D -> 0.7
        pidValuesY = PIDUpdate(relativePos[1], posTargetLockIn[1], 1.3, 0.7, lastErrorYLockIn, lastIYLockIn);
        correctionY = pidValuesY[0];
        lastErrorYLockIn = pidValuesY[1];
        lastIYLockIn = pidValuesY[2];
        
        
        // PID values for the X axis
        // P -> 1.5
        // D -> 0.1
        pidValuesX = PIDUpdate(relativePos[0], posTargetLockIn[0], 1.5, 0.1, lastErrorXLockIn, lastIXLockIn);
        correctionX = pidValuesX[0];
        lastErrorXLockIn = pidValuesX[1];
        lastIXLockIn = pidValuesY[2];
        
        
        // PID values for the angle
        // P -> 0.02
        // D -> 0.0005
        pidValuesAngle = PIDUpdate(angle, initialAngleLockIn, 0.02, 0.0005, lastErrorAngleLockIn, lastIAngleLockIn);
        correctionAngle = pidValuesAngle[0];
        lastErrorAngleLockIn = pidValuesAngle[1];
        lastIAngleLockIn = pidValuesY[2];

        double vxField, vyField, vxRobot, vyRobot;

        vxField = correctionX;
        vyField = correctionY;

        vxRobot = vyField * Math.sin(Math.toRadians(-heading)) + vxField * Math.cos(Math.toRadians(-heading));
        vyRobot = vyField * Math.cos(Math.toRadians(-heading)) - vxField * Math.sin(Math.toRadians(-heading));


        // Correct the motors
        runMotors(correctionAngle + vxRobot - vyRobot,
            -correctionAngle + vxRobot + vyRobot,
            correctionAngle + vxRobot + vyRobot,
            -correctionAngle + vxRobot - vyRobot);

    }
    
    public void resetLeftEncoder() {
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    public void resetRightEncoder() {
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    public void resetEncoders() {
        resetLeftEncoder();
        resetRightEncoder();
    }
    
    public void runMotors(double mLFP, double mRFP, double mLBP, double mRBP) {
        // 1800 ticks for regulate all the motors
        motorLeftFront.setVelocity(mLFP * 1800);
        motorLeftBack.setVelocity(mLBP * 1800);
        motorRightFront.setVelocity(mRFP * 1800);
        motorRightBack.setVelocity(mRBP * 1800);
    }
    
    public void stopMotors() {
        motorLeftFront.setVelocity(0);
        motorRightFront.setVelocity(0);
        motorLeftBack.setVelocity(0);
        motorRightBack.setVelocity(0);
    }
}
