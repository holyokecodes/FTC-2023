package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.ServoEx;


@TeleOp(name="TheNemo", group="Linear Opmode")

public class TheNemo extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //declare encoder variables
    static final int MAX_ENCODER = 0;
    static final int MIN_ENCODER = -3210;
    
    @Override
    public void runOpMode() {

        // telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //set up revIMU
        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();

        //set up servos
        SimpleServo handMoverServo = new SimpleServo (hardwareMap, "handMover", 0, 180);

        //set up motors
        Motor frontLeftMotor = new Motor (hardwareMap, "frontLeft");
        Motor frontRightMotor = new Motor (hardwareMap, "frontRight");
        Motor backLeftMotor = new Motor (hardwareMap, "backLeft");
        Motor backRightMotor = new Motor (hardwareMap, "backRight");
        Motor armLifterMotor = new Motor (hardwareMap, "armLifter");

        armLifterMotor.setRunMode(Motor.RunMode.PositionControl);
        armLifterMotor.setPositionCoefficient(.05);
        armLifterMotor.setPositionTolerance(5);
        
        // Setup mecanum
        MecanumDrive driveBase = new MecanumDrive (frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
            
        GamepadEx controller1 = new GamepadEx(gamepad1);
        
        TriggerReader rightTrigger = new TriggerReader(controller1, GamepadKeys.Trigger.RIGHT_TRIGGER);
        ButtonReader leftBumper = new ButtonReader(controller1, GamepadKeys.Button.LEFT_BUMPER);
        ButtonReader rightBumper = new ButtonReader(controller1, GamepadKeys.Button.RIGHT_BUMPER);
        ButtonReader xButton = new ButtonReader(controller1, GamepadKeys.Button.X);
        ButtonReader aButton = new ButtonReader(controller1, GamepadKeys.Button.A);
        ButtonReader yButton = new ButtonReader(controller1, GamepadKeys.Button.Y);
        ButtonReader dPadUp = new ButtonReader(controller1, GamepadKeys.Button.DPAD_UP);        
        ButtonReader dPadRight = new ButtonReader(controller1, GamepadKeys.Button.DPAD_RIGHT);
        ButtonReader dPadDown = new ButtonReader(controller1, GamepadKeys.Button.DPAD_DOWN);
        ButtonReader dPadLeft = new ButtonReader(controller1, GamepadKeys.Button.DPAD_LEFT);
        
        //declare variables
        double speed = 1.0;
        boolean handOpen = true;
        double openAngle = 35;
        double closeAngle = 102;
        int targetPosition = -50;
        
        handMoverServo.turnToAngle(openAngle);
        
       armLifterMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
       
        // Wait for the game to start
        waitForStart();
        runtime.reset();
        
        

        //reset encoder
        armLifterMotor.resetEncoder();


        // run until the end of the match
        while (opModeIsActive()) {

            //speed modifier
            rightTrigger.readValue();

            if(rightTrigger.isDown()) {
                speed = 0.25;
            } else {
                speed = 1.0;
            }

            //hand opener and closer
            xButton.readValue();
            if(xButton.wasJustPressed()) {
                handOpen = !handOpen; //switch true or false
                handMoverServo.turnToAngle(handOpen ? openAngle : closeAngle); //if true, first if false, second
            }

            //precise hand mover
            aButton.readValue();
            yButton.readValue();
            
            //set angle to servo angle
            double angle = handMoverServo.getAngle();
            
            if(yButton.isDown() && angle > openAngle) {
                handMoverServo.rotateByAngle(-.5);
            }

            if(aButton.isDown() && angle < closeAngle) {
                handMoverServo.rotateByAngle(.5);
            }
            
            //arm raiser
            telemetry.addData("handOpen",handOpen);
             
            dPadUp.readValue();
            dPadRight.readValue();
            dPadDown.readValue();
            dPadLeft.readValue();
            
            telemetry.addData("dPadDown",dPadDown.isDown());

            
            
            if(dPadDown.isDown()) {
                 targetPosition = -50;
            }
            
             if(dPadLeft.isDown()) {
                 targetPosition = -1354;
            }
            
             if(dPadUp.isDown()) {
                 targetPosition = -2226; 
            }
            
             if(dPadRight.isDown()) {
                 targetPosition = -3210;
            }
            
            leftBumper.readValue();
            rightBumper.readValue();

            if(leftBumper.isDown()) {
                //Lower arm
                targetPosition += 20;
            } else if(rightBumper.isDown()) {
                //Raise arm
                targetPosition += -20;
            
            
            }
            //arm lifter safety
            if(targetPosition > MAX_ENCODER) {
                targetPosition = MAX_ENCODER;
            }
            
            if(targetPosition < MIN_ENCODER) {
                targetPosition = MIN_ENCODER;
            }

            //actually move arm
            armLifterMotor.setTargetPosition(targetPosition);

            //brake motor
            if (armLifterMotor.atTargetPosition()){
                armLifterMotor.stopMotor();
            } else {
                armLifterMotor.set(0.275);
            }
            
         
            //telemetry.addData("targetPosition", targetPosition);
            //telemetry.addData("encoderPosition", armLifterMotor.getCurrentPosition());
            
            double strafeSpeed = -controller1.getLeftX() * speed;
            double fowardSpeed = controller1.getLeftY() * speed;
            double rotateSpeed =  -controller1.getRightX() * speed;
            double heading = -imu.getRotation2d().getDegrees();
            
            telemetry.addData("strafeSpeed", strafeSpeed);
            telemetry.addData("fowardSpeed", fowardSpeed);
            telemetry.addData("rotateSpeed", rotateSpeed);
            
            driveBase.driveFieldCentric(strafeSpeed, fowardSpeed, rotateSpeed,heading, false);
            
            telemetry.addData("Servo Angle", handMoverServo.getAngle());

            telemetry.update();

        }
    }
}
