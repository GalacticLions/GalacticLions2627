package org.firstinspires.ftc.team5898;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.ImuParameters;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import page.j5155.advantagescope.AdvantageScopeLite;

@Disabled
@Autonomous(name="Limelight Localization", group="limelight")
public class LimelightLocalization extends OpMode
{
    Limelight3A limelight;
    IMU imu;
    IMU.Parameters imuParameters;
    int[] validIDs = {20,24};

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(BNO055IMUNew.class,"imu");
        imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu.initialize(imuParameters);
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

    }
    @Override
    public void start(){

    }
    @Override
    public void loop() {
        Orientation robotOrientation = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        );
        double robotYaw = robotOrientation.firstAngle;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)
        } else {
            telemetry.addData("Limelight", "No Targets (April Tags)");
        }


        limelight.updateRobotOrientation(robotYaw);
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double robotX = botpose_mt2.getPosition().x;
                double robotY = botpose_mt2.getPosition().y;
                double robotZ = botpose_mt2.getPosition().z;
                double robotHeading = robotOrientation.firstAngle;
                telemetry.addData("Robot Position", String.format("X: %.2f, Y: %.2f, Z: %.2f", robotX, robotY, robotZ));
                telemetry.addData("Robot Heading", String.format("%.2f°", robotHeading));
            } else {
                telemetry.addData("Robot Position", "No Target");
            }
        }
        telemetry.update();
    }
    @Override
    public void stop(){

    }
}
