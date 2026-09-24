package org.firstinspires.ftc.team5898;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Disabled
@Autonomous(name="Auto_StraferBase", group="Starter Code", preselectTeleOp = "Strafer Tele Op - Test Code")
public class Auto_StraferBase extends LinearOpMode{
    // variable declaration & setup
    DcMotor frontleft, frontright, backleft, backright, motorArmTilt, motorBeltDrive;
    Servo servoWrist;
    CRServo servoClaw;

    // motor counts per rotation (ticks/pulses per rotation)
    // check motor specs from manufacturer
    // 537.7 is for GoBilda 312 RPM Yellow Jacket motor
    double cpr = 537.7;

    // adjust gearRatio if you have geared up or down your motors
    double gearRatio = 1;

    // wheel diameter in inches
    // 3.779 is for the GoBilda mecanum wheels
    double diameter = 3.779;

    //counts per inch: cpr * gear ratio / (pi * diameter (in inches))
    double cpi = (cpr * gearRatio)/(Math.PI * diameter);

    // use calibrate auto to check this number before proceeding
    double bias = 1.0; // adjust based on calibration opMode
    
    double strafeBias = 0.9;//change to adjust only strafing movement
    //
    double conversion = cpi * bias;
    IMU imu;

    @Override
    public void runOpMode(){


        // wait for Start to be pressed
        waitForStart();
        initGyro();



    }







    /**
     * Use to make the robot go forward a number of inches
     * @param inches distance to travel in inches
     * @param speed has a range of [0,1]
     */
    public void forward(double inches, double speed){ moveToPosition(inches, speed); }

    /**
     * Use to make the robot go backward a number of inches
     * @param inches distance to travel in inches
     * @param speed has a range of [0,1]
     */
    public void back(double inches, double speed){ moveToPosition(-inches, speed); }

    /**
    Rotate the robot left
    @param degrees the amount of degrees to rotate
    @param speed has a range of [0,1]
     */
    public void turnLeft(double degrees, double speed){ turnWithGyro(degrees, -speed); }

    /**
    Rotate the robot right
    @param degrees the amount of degrees to rotate
    @param speed has a range of [0,1]
     */
    public void turnRight(double degrees, double speed){ turnWithGyro(degrees, speed); }

    /**
    Strafe left
    @param inches the distance in inches to strafe
    @param speed has a range of [0,1]
     */
    public void strafeLeft(double inches, double speed){ strafeToPosition(-inches, speed); }

    /**
    Strafe right
    @param inches the distance in inches to strafe
    @param speed has a range of [0,1]
     */
    public void strafeRight(double inches, double speed){ strafeToPosition(inches, speed); }







    /*
    This function's purpose is simply to drive forward or backward.
    To drive backward, simply make the inches input negative.
     */
    public void moveToPosition(double inches, double speed){
        int move = (int)(Math.round(inches*conversion));
        backleft.setTargetPosition(backleft.getCurrentPosition() + move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);

        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){
            telemetry.addData("Busy...", "");
            telemetry.update();
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
    }

    /*
    This function uses the Hub IMU Integrated Gyro to turn a precise number of degrees (+/- 5).
    Degrees should always be positive, make speedDirection negative to turn left.
     */
    public void turnWithGyro(double degrees, double speedDirection){
        // Create an object to receive the IMU angles
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        //Initialize

        double yaw = robotOrientation.getYaw(AngleUnit.DEGREES); //make this negative?
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();

        double first;
        double second;

        // turning right
        if (speedDirection > 0){
            if (degrees > 10){first = (degrees - 10) + devertify(yaw);}
            else{first = devertify(yaw);}
            second = degrees + devertify(yaw);
        }

        // turning left
        else{
            if (degrees > 10){first = devertify(-(degrees - 10) + devertify(yaw));}
            else{first = devertify(yaw);}
            second = devertify(-degrees + devertify(yaw));
        }

        // Go to position
        double firsta = convertify(first - 5);
        double firstb = convertify(first + 5);
        turnWithEncoder(speedDirection);

        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES); //make this negative?
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }
        else{
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES); //make this negative?
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }


        double seconda = convertify(second - 5);//175
        double secondb = convertify(second + 5);//-175
        turnWithEncoder(speedDirection / 3);

        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES); //make this negative?
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES); //make this negative?
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);
        }

        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /*
    This function uses the encoders to strafe left or right.
    Negative input for inches results in left strafing.
     */
    public void strafeToPosition(double inches, double speed){
        int move = (int)(Math.round(inches * cpi * strafeBias));
        backleft.setTargetPosition(backleft.getCurrentPosition() - move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() - move);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);

        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){
            telemetry.addData("Working...", " ");
            telemetry.update();}
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
    }

    /*
    These functions are used in the turnWithGyro function to ensure inputs
    are interpreted properly.
     */
    public double devertify(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }
    public double convertify(double degrees){
        if (degrees > 360){
            degrees = degrees - 360;
        }
        else if(degrees < -180){
            degrees = 360 + degrees;
        }
        else if(degrees > 179){
            degrees = -(360 - degrees);
        }
        return degrees;
    }

    /*
    This function is called at the beginning of the program to activate
    the IMU Integrated Gyro.
     */
    public void initGyro(){
        // Check the orientation of the Rev Hub
        // more info on ftc-docs.firstinspires.org
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
    }

    /*
    This function is used in the turnWithGyro function to set the
    encoder mode and turn.
     */
    public void turnWithEncoder(double input){
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        frontleft.setPower(input);
        backleft.setPower(input);
        frontright.setPower(-input);
        backright.setPower(-input);
    }
    public void belt(int ticks, double speed){
        // ticks are how many ticks around the motor it should move
        motorBeltDrive.setTargetPosition(motorBeltDrive.getCurrentPosition() + ticks);
        motorBeltDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBeltDrive.setPower(speed);
        while (motorBeltDrive.isBusy()){
            telemetry.addData("Changing belt...", " ");
            telemetry.addData("Belt position: ", motorBeltDrive.getCurrentPosition());
            telemetry.update();}
        motorBeltDrive.setPower(0);
    }
    public void tilt(int ticks, double speed){
        // ticks are how many ticks around the motor it should move
        motorArmTilt.setTargetPosition(motorArmTilt.getCurrentPosition() + ticks);
        motorArmTilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmTilt.setPower(speed);
        while (motorArmTilt.isBusy()){
            telemetry.addData("Busy...", " ");
            telemetry.update();}
        motorArmTilt.setPower(0);
    }
    public void claw(double seconds){
        servoClaw.setPower(1);
    }
    public void wrist(double position, double speed){
        servoWrist.setPosition(Servo.Direction.values().length);
    }

}
