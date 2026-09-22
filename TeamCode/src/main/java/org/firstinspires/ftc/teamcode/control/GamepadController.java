package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.Cam;
import org.firstinspires.ftc.teamcode.subsystem.Mapx;
import org.firstinspires.ftc.teamcode.subsystem.Movement;


public class GamepadController {
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final Movement movement;
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



    public double velocityUsing;




    public GamepadController(Gamepad gamepad1, Gamepad gamepad2, Movement movement, Cam cam) {
        // Hardware map
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.movement = movement;
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


        this.velocityUsing = 0;

    }

    public void moveUsingGamepad() {
        // Get the gamepad values
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double power = movementVelocity;
        
        // Normalize to make the y goes up when the stick is going up
        y *= -1;
        
        if(isA1Pressed || isB1Pressed)
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

    public void lockPosition() {
        // Lock robot position if a is pressed
        movement.PIDLockInOp(isA1Pressed);
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
        if(gamepad1.bWasPressed()) {
            isB1Pressed = true;
        } else if(gamepad1.bWasReleased()) {
            isB1Pressed = false;
        }
        
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
