package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Launcher {
    // Initialize Variables
    private final DcMotorEx launcherMotor;
    private final DcMotor intakeMotor;
    private final DcMotor midtakeMotor;
    private final Servo leftServo;
    private final Servo rightServo;
    private final DcMotor led;
    
    // Launching constants
    private final double gravity;
    private final double objectiveHeight;
    private final double launcherHeight;
    
    // Launcher variables
    public double launcherRps;
    private double launcherLinearVelocity;
    
    // Launcher constants
    private final double launcherWheelCircumference;
    private final double encoderTicksPerRotation;

    // PIDF
    private double lastError;
    private double lastI;
    private final ElapsedTime timer;
    
    // Friction block constant
    private final double minVelocityToLaunch;
    
    // Launcher artefacts counter
    public double launchCounter;
    private boolean launchCounted;
    
    private double lastLauncherVelocityToCount;
    
    private final double frictionTimeInterval;
    private final double launchTimeInterval;
    private final double velocityDrop;
    
    private final ElapsedTime timerPIDUpdate;
        
    private final ElapsedTime launchCounterTimer;
    private final ElapsedTime launchCounterProcessTimer;

    public double valueFiltered;

    private final ElapsedTime filterTimer;
    public double tau;
    private boolean filterInit;

    // Servo
    private final ElapsedTime servoTimer;
    private boolean servoMoving;

    public boolean blockLed;
    private final ElapsedTime shootTimer;
    private final double servoMoveTime;
    private boolean shooting;
    public int shootsPerTime;

    public Launcher(DcMotorEx launcherMotor, DcMotor intakeMotor, DcMotor midtakeMotor, Servo leftServo, Servo rightServo, DcMotor led) {
        // Defines Variables and Constants Values
        this.launcherMotor = launcherMotor;
        this.intakeMotor = intakeMotor;
        this.midtakeMotor = midtakeMotor;
        this.leftServo = leftServo;
        this.rightServo = rightServo;
        this.led = led;

        // Launching constants
        this.gravity = 9.81;
        this.objectiveHeight = 1;
        this.launcherHeight = 0.4;


        // Launcher constants
        this.launcherWheelCircumference = 28.6;
        this.encoderTicksPerRotation = 28;

        // Launcher variables
        this.launcherRps = 0;

        this.launcherLinearVelocity = 0;

        this.lastError = 0;
        this.lastI = 0;

        // Friction block constant
        this.minVelocityToLaunch = 0;

        // PIDF
        this.timerPIDUpdate = new ElapsedTime();

        // Launchers counter
        this.launchCounter = 0;
        this.launchCounted = false;

        this.frictionTimeInterval = .1;
        this.launchTimeInterval = .130;
        this.velocityDrop = .21;

        this.lastLauncherVelocityToCount = 0;

        this.timer = new ElapsedTime();

        this.launchCounterTimer = new ElapsedTime();
        this.launchCounterProcessTimer = new ElapsedTime();

        // Filter
        this.valueFiltered = 0;
        this.filterInit = false;
        this.tau = 0.02;
        this.filterTimer = new ElapsedTime();
        filterTimer.startTime();

        // Servo
        this.servoTimer = new ElapsedTime();
        this.servoMoving = false;

        this.blockLed = false;
        this.shootTimer = new ElapsedTime();
        shootTimer.startTime();

        this.servoMoveTime = 0.175;
        this.shooting = false;

        this.shootsPerTime = 0;

        // Set motors direction
        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        launcherMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        timer.startTime();
        timerPIDUpdate.startTime();
        launchCounterTimer.startTime();
        launchCounterProcessTimer.startTime();
        servoTimer.startTime();
    }
    
    public void setVelocity(double velocity) {
        // Set the launcher velocity(meters per seconds)
        double rps = (velocity / launcherWheelCircumference) * 100;
        double ticks = rps * encoderTicksPerRotation;
        
        launcherMotor.setVelocity(ticks);
    }
    
    public void verifyLaunches() {
        updateLauncherVelocityAndRps();
        
        if(launchCounterTimer.time() >= frictionTimeInterval) {
            launchCounterTimer.reset();
            
            lastLauncherVelocityToCount = launcherLinearVelocity;
        }
        
        if(launchCounted && launchCounterProcessTimer.time() >= launchTimeInterval) {
            launchCounted = false;
        }
        
        if(launcherLinearVelocity + velocityDrop < lastLauncherVelocityToCount && !launchCounted) {
            launchCounter++;
            launchCounted = true;
            launchCounterProcessTimer.reset();
        }
    }
    
    public double getCompensateLauncherVelocity(double neededVelocity) {
        // Get the compensate launcher velocity for compensate friction

        return neededVelocity * 1.22 - 0.28;
    }
    
    public boolean isLauncherReadToGo(double velocityToReach, double errorRange) {
        // Verify if the launcher is stabilized
        updateLauncherVelocityAndRps();

        return !(velocityToReach + (errorRange / 2) < getVelocity()) && !(velocityToReach - (errorRange / 2) > getVelocity());
    }

    public void isReadyToGoLed(double velocityToReach, double errorRange) {
        boolean isReady = isLauncherReadToGo(velocityToReach, errorRange);

        if(blockLed)
            return;

        if(isReady)
            led.setPower(0.6);
        else
            turnLedOff();
    }

    public void turnLedOff() {
        if(!blockLed)
            led.setPower(0);
    }
    
    public double[] getLauncherNeededAngleAndVelocity(double objectiveDistance) {
        // This function is for 50.52º angled launcher
        // Ignores friction and air resistance
        
        if(objectiveDistance < 0) {
            return new double[] {0, 0};
        }

        double angle = 57;
        double angleRadians = Math.toRadians(angle);
        
        
        // Formula
        // v = d * sqrt((g) / (d - (1 - y0)))
        
        //double launcherVelocity = objectiveDistance * Math.sqrt(12.1334944706 / ((1.2139601223 * objectiveDistance) + this.launcherHeight - this.objectiveHeight));

        // Formula
        // v = sqrt((gravity * d²) / ((2cos²(ang)) * (d * tan(ang) - (objHeight - y0))))
        double idealVelocity = Math.sqrt((this.gravity * Math.pow(objectiveDistance, 2)) / ((2 * Math.pow(Math.cos(angleRadians), 2)) * (objectiveDistance * Math.tan(angleRadians) - (this.objectiveHeight - this.launcherHeight))));


        double vBase = (0.1 + 2.51 * idealVelocity) * 1.04;

        double k = 0.0370 * objectiveDistance * objectiveDistance - 0.2175 * objectiveDistance + 1.2425;

        double launcherVelocity = (vBase * k) * 1.01715;

        return new double[] {launcherVelocity, angle};
    
    }
    
    public void updateLauncherVelocityAndRps() {
        // Updates every second the variables
        // -launcher because for the robot launch the ball forwards, the motor needs to rotate counterclockwise
        launcherRps = launcherMotor.getVelocity() / encoderTicksPerRotation;
        
        launcherLinearVelocity = (launcherRps * launcherWheelCircumference) / 100;
    }
    
    public double getVelocity() {
        // Returns the launcher linear velocity
        return launcherLinearVelocity;
    }

    public double goldenWeightedAverage(double lighter, double heavier) {
        double phi = (1 + Math.sqrt(5)) / 2;

        double wHeavier = 1 / phi;
        double wLighter = 1 - wHeavier;


        return wLighter * lighter + wHeavier * heavier;
    }

    public void adaptFilter(double error, double tauMin, double tauMax, double errorMax) {
        double e;
        double k;

        e = Math.abs(error);
        k = Math.min(e / errorMax, 1.0);

        this.tau = tauMax - k * (tauMax - tauMin);

        valueFilter();
    }

    public void valueFilter() {
        // Resets the filter with a velocityValue
        if(!this.filterInit) {
            valueFiltered = getVelocity();
            this.filterInit = true;
            return;
        }

        // Gets the delta time
        double dt = filterTimer.time();
        filterTimer.reset();

        updateLauncherVelocityAndRps();

        // Gets alpha based on the natural exponential for a
        double alpha = 1 - Math.exp(-(dt / this.tau));
        this.valueFiltered = valueFiltered + alpha * (getVelocity() - valueFiltered);
    }

    public double[] PIDUpdate(double value, double target, double KP, double KD, double lastError, double lastI) {
        // Base variables
        double KI = 0.00001;
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

    public void setVelocityPIDFOp(double targetVelocity) {
        // Base variables
        double currentVelocity, correction, feedforward, error;
        double[] pidValues;

        final double tauMax = 0.28;
        final double tauMin = 0.03;
        final double errorMax = 0.5;
        
        // Feedforward
        // P = 0.128 * v - 0.002
        feedforward = 0.04203 * targetVelocity - 0.00474;

        double vFast = getVelocity();
        double vStable = this.valueFiltered;

        currentVelocity = goldenWeightedAverage(vFast, vStable);

        error = Math.abs(targetVelocity - currentVelocity);

        adaptFilter(error, tauMin, tauMax, errorMax);
        
        updateLauncherVelocityAndRps();

        
        // PID values based on the launcher velocity
        pidValues = PIDUpdate(currentVelocity, targetVelocity, 0.3, 0.09, lastError, lastI);
        correction = pidValues[0];
        lastError = pidValues[1];
        lastI = pidValues[2];


        // Motor correction
        launcherMotor.setPower(feedforward + correction);
    }

    public void setVelocityBasedOnCam(boolean start, Cam cam) {

        // Gets the target distance and needed velocity
        double targetDistance = cam.getAprilTagDistance() / 100;
        cam.tau = 0.1;
        targetDistance = cam.valueFiltered / 100;
        double neededVelocity = getLauncherNeededAngleAndVelocity(targetDistance)[0];
        double realVelocity = neededVelocity;

        // Sets the launcher to the correct amount of power
        if(start) {
            setVelocityPIDFOp(realVelocity);
        } else {
            runLauncher(false);
        }
    }


    public void setVelocityBasedOnMapx(int side, Mapx mapx, boolean start) {

        // Gets the target distance and needed velocity
        double targetDistance = mapx.getGoalDistance(side);
        double neededVelocity = getLauncherNeededAngleAndVelocity(targetDistance)[0];
        double realVelocity = neededVelocity;


        // Sets the launcher to the correct amount of power
        if(start) {
            setVelocityPIDFOp(realVelocity);
        } else {
            runLauncher(false);
        }
    }


    public void setVelocityPIDF(double defaultVelocity, Cam cam, double seconds, double waitForShooterSeconds, LinearOpMode linearOpMode) {
        double lastError = 0;
        double lastI = 0;
        double currentVelocity;
        double correction;
        double[] pidValues;
        
        double targetVelocity;
        double targetDistance;
        double neededVelocity;
        double realVelocity;

        double error, e, k;
        final double tauMax = 0.25;
        final double tauMin = 0.03;
        final double errorMax = 0.5;

        double feedforward;
        
        timer.reset();
        
        while(timer.time() <= seconds && linearOpMode.opModeIsActive()) {
            targetDistance = cam.getAprilTagDistance() / 100;
            
            neededVelocity = getLauncherNeededAngleAndVelocity(targetDistance)[0];
            realVelocity = getCompensateLauncherVelocity(neededVelocity);
            
            targetVelocity = realVelocity;
            
            if(targetDistance < 0) {
                targetVelocity = defaultVelocity;
            }
            
            // P = 0.128 * v - 0.002
            feedforward = 0.128 * targetVelocity - 0.002;

            currentVelocity = this.valueFiltered;

            error = Math.abs(targetVelocity - currentVelocity);
            e = Math.abs(error);
            k = Math.min(e / errorMax, 1.0);

            this.tau = tauMax - k * (tauMax - tauMin);

            valueFilter();
            
            updateLauncherVelocityAndRps();
            pidValues = PIDUpdate(currentVelocity, targetVelocity, 1, 0.1, lastError, lastI);
            
            correction = pidValues[0];
            lastError = pidValues[1];
            lastI = pidValues[2];
            
            launcherMotor.setPower(feedforward + correction);

            runToShooter(timer.time() >= waitForShooterSeconds);
        }
        
        launcherMotor.setPower(0);
    }
    
    public void runLauncher(boolean start, double power) {
        // Run launcher with variable power
        if(start)
            launcherMotor.setPower(power);
        else
            launcherMotor.setPower(0);
    }
    
    public void runLauncher(boolean start) {
        // Run launcher with static power
        if(start)
            launcherMotor.setPower(1);
        else
            launcherMotor.setPower(0);
    }
    
    public void runIntakeAndMidtake(boolean start, double power) {
        // Run absorber with variable power
        if(start)
            intakeMotor.setPower(power);
        else
            intakeMotor.setPower(0);
    }

    public void runIntakeAndMidtake(boolean start) {
        // Run absorber with static power
        if(start) {
            intakeMotor.setPower(1);
            midtakeMotor.setPower(-1);
        } else {
            intakeMotor.setPower(0);
            midtakeMotor.setPower(0);
        }
    }
    public void runAllIntake(boolean start) {
        // Run absorber with static power
        if(start) {
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
        }
    }

    public void runAllIntake(boolean start, boolean usesMidtake) {
        // Run absorber with static power
        if(start) {
            intakeMotor.setPower(1);
            if(usesMidtake)
                midtakeMotor.setPower(-1);
        } else {
            intakeMotor.setPower(0);
            if(usesMidtake)
                midtakeMotor.setPower(0);
        }
    }

    
    public void runToShooter(boolean start) {
        updateLauncherVelocityAndRps();
        
        if(launcherLinearVelocity < minVelocityToLaunch) {
            runIntakeAndMidtake(false);
            
            return;
        }

        runIntakeAndMidtake(start);
    }
    
    public void runToTransporter(boolean start) {
        runIntakeAndMidtake(start);
    }

    public void transportToShooter(boolean move) {
        double startEsq = 0.26;
        double endEsq = 0.5;

        double startDir = 0.38;
        double endDir = 0.17;

        if(move && !servoMoving) {
            servoMoving = true;
            servoTimer.reset();
        }

        if(this.servoMoving) {
            leftServo.setPosition(endEsq);
            rightServo.setPosition(endDir);

            if(servoTimer.time() > this.servoMoveTime) {
                this.servoMoving = false;

                leftServo.setPosition(startEsq);
                rightServo.setPosition(startDir);
            }

        } else {
            leftServo.setPosition(startEsq);
            rightServo.setPosition(startDir);
        }
    }

    public void shootArtefacts(boolean shoot) {
        if(!shoot) {
            midtakeMotor.setPower(0);
            this.shootsPerTime = 0;
        }

        if(!shoot && this.shooting) {
            transportToShooter(false);
            this.shooting = false;
        }

        if(shoot && !this.shooting) {
            this.shooting = true;
            shootTimer.reset();
            midtakeMotor.setPower(-1);
        }

        if(this.shooting) {
            if(shootTimer.time() > this.servoMoveTime * 2.2 || (shootsPerTime == 0 && shootTimer.time() > this.servoMoveTime * 1.5)) {
                shootTimer.reset();
                this.shootsPerTime++;
                transportToShooter(true);
                midtakeMotor.setPower(-1);
            } else {
                transportToShooter(false);
                if(this.shootsPerTime == 0) {
                    midtakeMotor.setPower(-1);
                    runAllIntake(false, false);
                } else {
                    midtakeMotor.setPower(1);
                    runAllIntake(true, false);
                }
            }
        }
    }
    public void shootArtefacts(boolean shoot, double kT) {
        if(!shoot) {
            midtakeMotor.setPower(0);
            this.shootsPerTime = 0;
        }

        if(!shoot && this.shooting) {
            transportToShooter(false);
            this.shooting = false;
        }

        if(shoot && !this.shooting) {
            this.shooting = true;
            shootTimer.reset();
            midtakeMotor.setPower(-1);
        }

        if(this.shooting) {
            if(shootTimer.time() > this.servoMoveTime * kT || (shootsPerTime == 0 && shootTimer.time() > this.servoMoveTime * 1.5)) {
                shootTimer.reset();
                this.shootsPerTime++;
                transportToShooter(true);
                midtakeMotor.setPower(-1);
            } else {
                transportToShooter(false);
                if(this.shootsPerTime == 0) {
                    midtakeMotor.setPower(-1);
                    runAllIntake(false, false);
                } else {
                    midtakeMotor.setPower(1);
                    runAllIntake(true, false);
                }
            }
        }
    }
}