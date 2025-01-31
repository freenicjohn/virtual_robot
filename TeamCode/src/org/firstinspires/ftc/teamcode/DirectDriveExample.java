package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "SimpleTeleOp")
public class DirectDriveExample extends LinearOpMode {

    public DcMotor motorRightFront = null;
    public DcMotor motorRightBack = null;
    public DcMotor motorLeftBack = null;
    public DcMotor motorLeftFront = null;

    @Override
    public void runOpMode() {

        motorLeftBack = hardwareMap.dcMotor.get("back_left_motor");
        motorLeftFront = hardwareMap.dcMotor.get("front_left_motor");
        motorRightFront = hardwareMap.dcMotor.get("front_right_motor");
        motorRightBack = hardwareMap.dcMotor.get("back_right_motor");

        // Calling setPower with a positive value should rotate the wheel forward
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0) {
                directDriveControl(0.5);
            } else {
                directDriveControl();
            }

            sleep(20);  // Sleeping here allows the CPU to catch up with other tasks
        }
    }

    private void directDriveControl() {
        directDriveControl(1.0);
    }

    private void directDriveControl(double speedMultiplier) {
        double max;

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double r = gamepad1.right_stick_x;

        // Apply exponential scaling to the joystick values
        y = Math.signum(y) * Math.pow(y, 2.0);
        x = Math.signum(x) * Math.pow(x, 2.0);

        // Check out https://github.com/freenicjohn/FtcRobotController/blob/10ea65b1cae2b63770d78149a75bdefd3b8b9877/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/docs/OmniWheelControlDerivation.md
        // for the derivation of these equations.
        double powerRightFront = -x + y - r;
        double powerRightBack  =  x + y - r;
        double powerLeftBack   = -x + y + r;
        double powerLeftFront  =  x + y + r;

        max = Math.max(Math.max(Math.abs(powerLeftFront), Math.abs(powerRightBack)),
                Math.max(Math.abs(powerLeftBack), Math.abs(powerLeftFront)));

        // If any individual motor power is greater than 1.0, scale all values to fit in the range [-1.0, 1.0]
        if (max > 1.0) {
            powerRightFront  /= max;
            powerRightBack   /= max;
            powerLeftBack    /= max;
            powerLeftFront   /= max;
        }

        motorRightFront.setPower(powerRightFront * speedMultiplier);
        motorRightBack.setPower(powerRightBack * speedMultiplier);
        motorLeftBack.setPower(powerLeftBack * speedMultiplier);
        motorLeftFront.setPower(powerLeftFront * speedMultiplier);
    }
}
