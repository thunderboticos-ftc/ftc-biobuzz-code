package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class GamepadController {
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final Movement movement;
    private final Launcher launcher;
    private final Cam cam;
    
    // Gamepad1 variables
    public boolean isA1Pressed;
    public boolean isB1Pressed;
    public boolean isY1Pressed;
    private boolean lastLeftBumper1;
    private boolean lastRightBumper1;
    public boolean leftBumper1WasPressed;
    public boolean rightBumper1WasPressed;
    
    // Gamepad2 variables
    public boolean isA2Pressed;
    public boolean isB2Pressed;
    public boolean isY2Pressed;
    private boolean lastLeftBumper2;
    private boolean lastRightBumper2;
    public boolean leftBumper2WasPressed;
    public boolean rightBumper2WasPressed;
    
    // Movement variables
    private double movementVelocity; // 0 - 1
    private final double velocityJump; // 0 - 1

    // Launcher variables

    public double launcherVelocity;
    private final double launcherVelocityJump;


    public double velocityUsing;


    private double launcherErrorRange;

    private RobotMemory robotMemory;



    public GamepadController(Gamepad gamepad1, Gamepad gamepad2, Movement movement, Launcher launcher, Cam cam) {
        // Hardware map
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.movement = movement;
        this.launcher = launcher;
        this.cam = cam;
        
        // Gamepad1 variables
        this.isA1Pressed = false;
        this.isB1Pressed = false;
        this.isY1Pressed = false;
        this.lastLeftBumper1 = false;
        this.lastRightBumper1 = false;
        this.leftBumper1WasPressed = false;
        this.rightBumper1WasPressed = false;
        
        // Gamepad2 variables
        this.isA2Pressed = false;
        this.isB2Pressed = false;
        this.isY2Pressed = false;
        this.lastLeftBumper2 = false;
        this.lastRightBumper2 = false;
        this.leftBumper2WasPressed = false;
        this.rightBumper2WasPressed = false;
        
        
        // Movement variables
        this.movementVelocity = 1;
        this.velocityJump = 0.2;

        // Launcher variables
        this.launcherVelocity = 5;
        this.launcherVelocityJump = 0.1;


        this.velocityUsing = 0;

        launcherErrorRange = 1;

        this.robotMemory = RobotMemory.INSTANCE;

    }

    public void rumbleOnDangerousAreas(int side, Mapx1 mapx) {
        // side:
        // 0 -> Red
        // 1 -> Blue

        // Positions need to be in inches

        double[][] dangerousPositions;

        if(side == 0) {
            // Red side

            dangerousPositions = new double[][] {{141, 65},
                    {141, 47.5},
                    {141, 30},
                    {18.75, 72},
                    {105.25, 33.25}};
        } else {
            // Blue side

            dangerousPositions = new double[][] {{3, 65},
                    {3, 47.5},
                    {3, 30},
                    {125.25, 72},
                    {38.75, 33.25}};
        }

        // Change the robotPos using the spacial position in inches
        double[] robotPos = mapx.positionInches;

        double dist;
        double xDistance;
        double yDistance;
        double rumbleSensibility;
        double kS = 1.0 / 3.0;
        double maxSensibility = 300;
        double shortestDist = -1;
        int rumble;

        for(double[] dangerousPos : dangerousPositions) {

            xDistance = Math.abs(dangerousPos[0] - robotPos[0]);
            yDistance = Math.abs(dangerousPos[1] - robotPos[1]);


            dist = (xDistance * xDistance) + (yDistance * yDistance);

            if(shortestDist < 0 || dist < shortestDist)
                shortestDist = dist;
        }

        if(shortestDist < 900) {
            rumbleSensibility = -kS * shortestDist + maxSensibility;
        } else {
            rumbleSensibility = 0;
        }

        try {
            rumble = (int) Math.round(rumbleSensibility);

        } catch(Exception e) {

            rumble = 0;
        }

        gamepad1.rumble(rumble);
        gamepad2.rumble(rumble);
    }

    public void correctAngleForTargetUsingGamepad() {
        double power = gamepad1.left_trigger;
        if(isB1Pressed) {
            movement.correctRobotAngleForTarget(power, 0.2, cam);
        }        
    }
    
    public void moveUsingGamepad() {
        // Get the gamepad values
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double power = movementVelocity;
        
        // Normalize to make the y goes up when the stick is going up
        y *= -1;
        
        if(isA1Pressed)
            return;
        
        movement.moveByVector(new double[] {x, y}, turn, power);
    }
    
    public void changeVelocityUsingGamepad() {
        // Get the current state of the bumpers
        boolean leftBumper = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;
        
        // Do nothing if both bumpers being pressed
        if(leftBumper && rightBumper)
            return;
        
        // Verify if want a "turbo" velocity jump or a normal
        if(isY1Pressed) {
            // "Turbo" velocity jump
            
            // Decreases velocity
            if(leftBumper && movementVelocity - velocityJump >= 0) {
                movementVelocity -= velocityJump;
            }
            
            // Increases velocity
            if(rightBumper && movementVelocity + velocityJump <= 1) {
                movementVelocity += velocityJump;
            }
        } else {
            // Normal velocity jump
            
            // Decreases velocity
            if(leftBumper1WasPressed && movementVelocity - velocityJump >= 0) {
                movementVelocity -= velocityJump;
            }
            
            // Increases velocity
            if(rightBumper1WasPressed && movementVelocity + velocityJump <= 1) {
                movementVelocity += velocityJump;
            }
        }
    }

    public void changeLauncherVelocityUsingGamepad() {
        // Get the current state of the bumpers
        boolean leftBumper = gamepad2.left_bumper;
        boolean rightBumper = gamepad2.right_bumper;

        // Do nothing if both bumpers being pressed
        if(leftBumper && rightBumper)
            return;

        // Verify if want a "turbo" velocity jump or a normal
        if(isY2Pressed) {
            // "Turbo" velocity jump

            // Decreases velocity
            if(leftBumper && launcherVelocity - launcherVelocityJump >= 0) {
                launcherVelocity -= launcherVelocityJump;
            }

            // Increases velocity
            if(rightBumper && launcherVelocity + launcherVelocityJump <= 5000) {
                launcherVelocity += launcherVelocityJump;
            }
        } else {
            // Normal velocity jump

            // Decreases velocity
            if(leftBumper2WasPressed && launcherVelocity - launcherVelocityJump >= 0) {
                launcherVelocity -= launcherVelocityJump;
            }

            // Increases velocity
            if(rightBumper2WasPressed && launcherVelocity + launcherVelocityJump <= 5000) {
                launcherVelocity += launcherVelocityJump;
            }
        }
    }

    public void launchUsingGamepad() {
        // Base variables
        boolean rightBumper = gamepad2.right_bumper;
        
        // Gets the target distance and needed velocity
        double targetDistance = cam.getAprilTagDistance() / 100;
        double neededVelocity = launcher.getLauncherNeededAngleAndVelocity(targetDistance)[0];
        double realVelocity = neededVelocity;

        // Do nothing if it conflicts with other method
        if(gamepad2.right_trigger > 0) {
            return;
        }
        
        // Sets the launcher to the correct amount of power
        if(rightBumper) {
            if(targetDistance < 0) {
                velocityUsing = 0;
                launcher.runLauncher(false);
                launcher.turnLedOff();
            } else {
                if(velocityUsing == 0)
                    velocityUsing = realVelocity;

                launcher.setVelocityPIDFOp(velocityUsing);
                launcher.isReadyToGoLed(velocityUsing, this.launcherErrorRange);
            }
        } else {
            velocityUsing = 0;
            launcher.runLauncher(false);
            launcher.turnLedOff();
        }
    }
    
    public void launchUsingGamepadWithoutCam() {
        boolean rightBumper = gamepad2.right_bumper;
        
        // Do not if it conflicts with other method
        if(rightBumper) {
            return;    
        }
        
        // Do the launch with a static velocity if the trigger is pressed
        if(gamepad2.right_trigger > 0) {
            launcher.setVelocityPIDFOp(this.launcherVelocity);
            launcher.isReadyToGoLed(this.launcherVelocity, this.launcherErrorRange);
        } else {
            launcher.setVelocity(0);
            launcher.turnLedOff();
        }
    }
    
    public void ejectArtefacts() {
        // Base variable
        double power = gamepad2.left_trigger;
        
        // Do nothing if it conflicts if other methods
        if(isB2Pressed || isA2Pressed) 
            return;
        
        // Ejects the artefacts
        if(power > 0) {
            launcher.runIntake(true, -1);
        } else {
            launcher.runIntake(false);
        }
    }
    
    public void transportToShooter() {
        // Run the ball to the absorber
        launcher.transportToShooter(gamepad2.a);
    }

    public void transportBallToLauncher() {
        // Makes the ball go to the launcher
        if(isA2Pressed)
            return;

        launcher.runToShooter(isB2Pressed);
    }
    
    public void lockPosition() {
        // Lock robot position if a is pressed
        movement.PIDLockInOp(isA1Pressed);
    }

    public void goToEndLocal(int side, Mapx1 mapx) {
        double[] goToPos = new double[2];
        double goToHeading = 0;

        if(side == 0) {
            goToPos = new double[] {38.444810543657326, 33.39044481054367};
            goToHeading = 0;

        } else if(side == 1) {
            goToPos = new double[] {105.34431630971994, 33.62108731466229};
            goToHeading = 180;

        }

        mapx.goToGoal(goToPos, goToHeading, false, gamepad1.b);
    }
    
    public void keyStatesUpdate() {
        boolean leftBumper1 = gamepad1.left_bumper;
        boolean rightBumper1 = gamepad1.right_bumper;

        boolean leftBumper2 = gamepad2.left_bumper;
        boolean rightBumper2 = gamepad2.right_bumper;
        
        // Verify if y1 is currently being pressed
        if(gamepad1.yWasPressed()) {
            isY1Pressed = true;
        } else if(gamepad1.yWasReleased()) {
            isY1Pressed = false;
        }
        
        
        // Verify if a1 is currently being pressed
        if(gamepad1.aWasPressed()) {
            isA1Pressed = true;
        } else if(gamepad1.aWasReleased()) {
            isA1Pressed = false;
        }
        
        
        
        
        // Verify if y2 is currently being pressed
        if(gamepad2.yWasPressed()) {
            isY2Pressed = true;
        } else if(gamepad2.yWasReleased()) {
            isY2Pressed = false;
        }
    
        // Verify if a2 is currently being pressed
        if(gamepad2.aWasPressed()) {
            isA2Pressed = true;
        } else if(gamepad2.aWasReleased()) {
            isA2Pressed = false;
        }
        
        // Verify if b2 is currently being pressed
        if(gamepad2.bWasPressed()) {
            isB2Pressed = true;
        } else if(gamepad2.bWasReleased()) {
            isB2Pressed = false;
        }
        
        // Verify if b1 is currently being pressed
        isB1Pressed = gamepad1.bWasPressed();
        
        
        if(lastLeftBumper1) {
            leftBumper1WasPressed = false;
        } else if(leftBumper1) {
            leftBumper1WasPressed = true;
        }
        
        if(lastRightBumper1) {
            rightBumper1WasPressed = false;
        } else if(rightBumper1) {
            rightBumper1WasPressed = true;
        }


        if(lastLeftBumper2) {
            leftBumper2WasPressed = false;
        } else if(leftBumper2) {
            leftBumper2WasPressed = true;
        }

        if(lastRightBumper2) {
            rightBumper2WasPressed = false;
        } else if(rightBumper2) {
            rightBumper2WasPressed = true;
        }
        
        lastLeftBumper1 = leftBumper1;
        lastRightBumper1 = rightBumper1;

        lastLeftBumper2 = leftBumper2;
        lastRightBumper2 = rightBumper2;
    }


}
