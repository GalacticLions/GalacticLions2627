// TeleOp using PedroPathing instead of manual field-centric drive
package org.firstinspires.ftc.team5898;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team5898.Constants.CannonConstants;
import org.firstinspires.ftc.team5898.LimelightUtils.VisualServoing;
import org.firstinspires.ftc.team5898.pedroPathing.Constants;
import com.pedropathing.follower.Follower;

@Disabled
@TeleOp(name = "Beta TeleOP (PedroPathing)")
public class Beta_TeleOP_Pedro extends OpMode {

    // PedroPathing follower
    Follower follower;

    // Mechanism hardware
    Limelight3A limelight;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    DcMotorEx topLauncher, bottomLauncher;
    VisualServoing visualServoing;
    CRServo backLeftServo, backRightServo;
    Servo stopperServo;
    DcMotor frontIntake;

    // Mechanism constants
    double intakePower;
    double LAUNCH_VELOCITY;
    double kP, kI, kD, kF;

    // State machine for mechanisms
    enum States { IDLE, INTAKE, LAUNCH, SPOOL_UP, VISUAL_SERVOING }
    private States robotState = States.IDLE;

    // Button state tracking
    boolean prevA1 = false, prevB2 = false, prevA2 = false;

    @Override
    public void init() {
        // 1) PedroPathing init
        follower = Constants.createFollower(hardwareMap);
        // startingPose can be set if you want a specific field-relative starting position
        follower.setStartingPose(null); // you can pass a saved Pose here if desired
        follower.update(); // update once so pose is valid

        // 2) Mechanism constants
        intakePower = CannonConstants.IntakePower;
        kP = CannonConstants.kP;
        kI = CannonConstants.kI;
        kD = CannonConstants.kD;
        kF = CannonConstants.kF;
        LAUNCH_VELOCITY = CannonConstants.FRONT_LAUNCH_VELOCITY;

        // 3) Limelight init
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        telemetry.setMsTransmissionInterval(11);
        limelight.setPollRateHz(90);
        limelight.start();

        // 4) Drive motors – still need them for mechanisms, but Pedro handles their powers
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        topLauncher = hardwareMap.get(DcMotorEx.class, "TLaunch");
        bottomLauncher = hardwareMap.get(DcMotorEx.class, "BLaunch");
        frontIntake = hardwareMap.get(DcMotor.class, "Intake");

        // Directions (only needed for mechanisms; Pedro handles drive motors internally)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        topLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomLauncher.setDirection(DcMotorSimple.Direction.FORWARD);

        // Mode setup for launchers (RUN_USING_ENCODER + PIDF)
        topLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topLauncher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(kP, kI, kD, kF)
        );
        bottomLauncher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(kP, kI, kD, kF)
        );

        topLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Intake & servos
        backLeftServo = hardwareMap.get(CRServo.class, "LServo");
        backRightServo = hardwareMap.get(CRServo.class, "RServo");
        stopperServo = hardwareMap.get(Servo.class, "STPR");

        frontIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightServo.setDirection(DcMotorSimple.Direction.FORWARD);

        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Visual servoing helper
        visualServoing = new VisualServoing(limelight, frontLeft, frontRight, backRight, backLeft, telemetry);

        telemetry.addData("Status", "Initialized (PedroPathing TeleOp)");
        telemetry.update();
    }

    @Override
    public void start() {
        // Tell Pedro we’re in TeleOp and we want brake mode on the drive motors
        follower.startTeleopDrive(true); // true = use BRAKE mode in TeleOp【turn1fetch0】
    }

    @Override
    public void loop() {
        // 1) Update PedroPathing (this updates pose, odometry, drive powers)
        follower.update();

        // 2) Update button states
        boolean a1Pressed = gamepad1.a;
        boolean b2Pressed = gamepad2.b;
        boolean a2Pressed = gamepad2.a;
        boolean rightTrigger2 = gamepad2.right_trigger > 0.3;

        boolean a1JustPressed = a1Pressed && !prevA1;
        boolean b2JustPressed = b2Pressed && !prevB2;
        boolean a2JustPressed = a2Pressed && !prevA2;

        // 3) Mechanism state machine (same logic, but drive is handled by Pedro)
        switch (robotState) {
            case IDLE:
                // Drive is handled by Pedro via setTeleOpDrive below, so no manual motor powers here

                // Stop all mechanisms
                backLeftServo.setPower(0);
                backRightServo.setPower(0);
                frontIntake.setPower(0);
                topLauncher.setPower(0);
                bottomLauncher.setPower(0);
                stopperServo.setPosition(1); // Close stopper

                // State transitions
                if (b2JustPressed) {
                    robotState = States.INTAKE;
                } else if (a1JustPressed) {
                    robotState = States.VISUAL_SERVOING;
                }
                break;

            case INTAKE:
                // Drive still Pedro-controlled

                // Intake action
                backLeftServo.setPower(intakePower);
                backRightServo.setPower(intakePower);
                frontIntake.setPower(0.7);
                stopperServo.setPosition(1); // Stopper closed

                // State transitions
                if (b2JustPressed) {
                    robotState = States.IDLE; // Press B again to stop
                } else if (rightTrigger2) {
                    robotState = States.SPOOL_UP; // Trigger to spool up
                }
                break;

            case SPOOL_UP:
                // Drive still Pedro-controlled

                // Launcher wheel acceleration
                topLauncher.setVelocity(LAUNCH_VELOCITY);
                bottomLauncher.setVelocity(LAUNCH_VELOCITY);

                // Stop intake while waiting to launch
                backLeftServo.setPower(0);
                backRightServo.setPower(0);
                frontIntake.setPower(0);
                stopperServo.setPosition(1);

                // State transitions
                if (!rightTrigger2) {
                    robotState = States.IDLE; // Release Trigger -> IDLE
                } else if (a2JustPressed) {
                    robotState = States.LAUNCH; // Press A to launch
                }
                break;

            case LAUNCH:
                // Drive still Pedro-controlled

                // Maintain launcher wheel velocity
                topLauncher.setVelocity(LAUNCH_VELOCITY);
                bottomLauncher.setVelocity(LAUNCH_VELOCITY);

                // Open stopper and feed the ring
                stopperServo.setPosition(0.5); // Move stopper away
                backLeftServo.setPower(0.7);
                backRightServo.setPower(0.7);

                // State transitions
                if (!a2Pressed) {
                    robotState = States.SPOOL_UP; // Release A -> back to SPOOL_UP
                }
                if (!rightTrigger2) {
                    robotState = States.IDLE; // Release Trigger -> IDLE
                }
                break;

            case VISUAL_SERVOING:
                // In VISUAL_SERVOING we still want Pedro drive, but we can override
                // with visualServoing if it sets its own motor powers.
                visualServoing.visualServo();

                // State transition
                if (a1JustPressed) {
                    robotState = States.IDLE;
                }
                break;
        }

        // 4) Update previous button states
        prevA1 = a1Pressed;
        prevB2 = b2Pressed;
        prevA2 = a2Pressed;

        // 5) TeleOp drive input to PedroPathing
        //    This is the standard TeleOp pattern from the docs.
        if (robotState != States.VISUAL_SERVOING) {
            // Gamepad input ( Pedro uses: forward, strafe, turn, isRobotCentric, headingOffset )
            double forward = -gamepad1.left_stick_y;
            double strafe  = -gamepad1.left_stick_x;
            double turn    = -gamepad1.right_stick_x;

            // Example: field-centric (what you had before), so isRobotCentric = false
            // If you want robot-centric, set the last parameter to true.
            // headingOffset = 0 means "forward" on the field is 0 radians.
            follower.setTeleOpDrive(forward, strafe, turn, false, 0.0f);
        } else {
            // In VISUAL_SERVOING, visualServoing is setting motor powers directly.
            // You can choose to still apply a small turn correction here, or leave it fully to visualServoing.
            // For simplicity, we let visualServoing control the drive motors.
        }

        // 6) Telemetry
        telemetry.addData("State", robotState);
        telemetry.addData("Launcher Velocity", topLauncher.getVelocity());
        telemetry.addData("PedroPose", follower.getPose());
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}