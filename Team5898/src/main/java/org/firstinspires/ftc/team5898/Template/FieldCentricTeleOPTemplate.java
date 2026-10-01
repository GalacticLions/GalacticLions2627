package org.firstinspires.ftc.team5898.Template;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.team5898.Hardware.Robot;

/**
 * Template for a field-centric TeleOp.
 *
 * gamepad1 (driver):
 *   left stick   drive / strafe
 *   right stick  turn
 *   guide        reset heading
 */

@Disabled
public class FieldCentricTeleOPTemplate extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void loop() {
        // Reset heading so the current facing becomes "forward"
        if (gamepad1.guide) {
            robot.resetHeading();
        }

        // Drive relative to the field; stick Y is inverted
        robot.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }

    @Override
    public void stop() {
        robot.drive(0, 0, 0);
    }
}
