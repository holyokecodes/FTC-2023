package org.firstinspires.ftc.teamcode;

// imports
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

    // declare encoder variables
    static final int MAX_ENCODER = 0;
    static final int MIN_ENCODER = -3210;

    @Override
    public void runOpMode() {

        // telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // set up revIMU
        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();

        // set up servos
        SimpleServo handMoverServo = new SimpleServo (hardwareMap, "handMover", 0, 180);

        //set up motors
        Motor frontLeftMotor = new Motor (hardwareMap, "frontLeft");
        Motor frontRightMotor = new Motor (hardwareMap, "frontRight");
        Motor backLeftMotor = new Motor (hardwareMap, "backLeft");
        Motor backRightMotor = new Motor (hardwareMap, "backRight");
        Motor armLifterMotor = new Motor (hardwareMap, "armLifter");

        armLifterMotor.setRunMode(Motor.RunMode.PositionControl);
        armLifterMotor.setPositionCoefficient(.05);
        armLifterMotor.setPositionTolerance(3);

        // Setup mecanum
        MecanumDrive driveBase = new MecanumDrive (frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

        // setup controllers
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
        ButtonReader bButton = new ButtonReader(controller1, GamepadKeys.Button.B);

        // declare variables
        double speed = 1.0;
        boolean handOpen = true;
        int targetPosition = -72;
        double closeAngle = 90;
        double openAngle = 25;

        

        // set behavior to brake
        armLifterMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        
        //praise shrek (very important)
        telemetry.addData("Shrek Is With You", " ");
        telemetry.update();
        
        // Wait for the game to start
        waitForStart();
        runtime.reset();
        handMoverServo.turnToAngle(openAngle);
        armLifterMotor.setTargetPosition(targetPosition);
        armLifterMotor.resetEncoder();
        
        // run until the end of the match
        while (opModeIsActive()) {
            
            
            
                
            
            
            // if(bButton.wasJustPressed()) {
            //     handMoverServo.turnToAngle(openAngle);
            //     while(handMoverServo.getAngle()  != openAngle && opModeIsActive()) {};
            //     handMoverServo.turnToAngle(closeAngle);
            //     while(handMoverServo.getAngle() != closeAngle && opModeIsActive()) {};
            //     targetPosition = -200;
            //     armLifterMotor.setTargetPosition(targetPosition);
            //     targetPosition = -50;
            //     armLifterMotor.setTargetPosition(targetPosition);
                
                

            // }
            
            
            // speed modifier

            if(rightTrigger.isDown()) {
                speed = 0.25;
            } else {
                speed = 1.0;
            }

            // hand opener and closer
            xButton.readValue();
            if(xButton.wasJustPressed()) {
                handOpen = !handOpen; //switch true or false
                handMoverServo.turnToAngle(handOpen ? openAngle : closeAngle); //if true, first.  if false, second
            }

            // read controller
            rightTrigger.readValue();
            leftBumper.readValue();
            rightBumper.readValue();
            aButton.readValue();
            yButton.readValue();
            dPadUp.readValue();
            dPadRight.readValue();
            dPadDown.readValue();
            dPadLeft.readValue();
            bButton.readValue();


            // precise hand mover
            double angle = handMoverServo.getAngle();

            if(yButton.isDown() && angle > openAngle) {
                handMoverServo.rotateByAngle(-.5);
            }

            if(aButton.isDown() && angle < closeAngle) {
                handMoverServo.rotateByAngle(.5);
            }



            if(leftBumper.isDown()) {
                // Lower arm
                targetPosition += 20;
            } else if(rightBumper.isDown()) {
                // Raise arm
                targetPosition += -20;
            }

            // arm presets
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
                targetPosition = -3200;
            }

            if(bButton.isDown()) {
                targetPosition = -400;
            }
            

            // arm lifter safety
            if(targetPosition > MAX_ENCODER) {
                targetPosition = MAX_ENCODER;
            }

            if(targetPosition < MIN_ENCODER) {
                targetPosition = MIN_ENCODER;
            }

            //actually move arm
            armLifterMotor.setTargetPosition(targetPosition);

            // brake motor
            if (armLifterMotor.atTargetPosition()){
                armLifterMotor.stopMotor();
            } else {
                armLifterMotor.set(0.275);
            }


            // set variables
            double strafeSpeed = -controller1.getLeftX() * speed;
            double fowardSpeed = controller1.getLeftY() * speed;
            double rotateSpeed =  -controller1.getRightX() * speed;
            double heading = -imu.getRotation2d().getDegrees();

            // move robot
            driveBase.driveFieldCentric(strafeSpeed, fowardSpeed, rotateSpeed,heading, false);

            // telemetry

            telemetry.addData("targetPosition", targetPosition);
            telemetry.addData("encoderPosition", armLifterMotor.getCurrentPosition());
            // telemetry.addData("handOpen",handOpen);
            telemetry.addData("Servo Angle", handMoverServo.getAngle());
            // telemetry.addData("dPadDown",dPadDown.isDown());
            // telemetry.addData("strafeSpeed", strafeSpeed);
            // telemetry.addData("fowardSpeed", fowardSpeed);
            // telemetry.addData("rotateSpeed", rotateSpeed);
            telemetry.addData("Heading", heading);
            telemetry.addData("RobotAngle", imu.getHeading());

            
            //update telemetry
            telemetry.update();

        }
    }
}
