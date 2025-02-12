package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "DirectDriveExample")
public class DirectDriveExample extends LinearOpMode {

    public DcMotor motorRightFront = null;
    public DcMotor motorRightBack = null;
    public DcMotor motorLeftBack = null;
    public DcMotor motorLeftFront = null;
    public SparkFunOTOS myOtos = null;

    @Override
    public void runOpMode() {

        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        motorLeftBack = hardwareMap.dcMotor.get("back_left_motor");
        motorLeftFront = hardwareMap.dcMotor.get("front_left_motor");
        motorRightFront = hardwareMap.dcMotor.get("front_right_motor");
        motorRightBack = hardwareMap.dcMotor.get("back_right_motor");

        // Calling setPower with a positive value should rotate the wheel forward
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        configureOtos();

        waitForStart();

        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();

            if (gamepad1.right_trigger > 0) {
                directDriveControl(0.5);
            } else {
                directDriveControl(1.0);
            }

            telemetry.addData("Position", "X: %.2f, Y: %.2f, H: %.2f", pos.x, pos.y, pos.h);
            telemetry.update();
            sleep(20);  // Sleeping here allows the CPU to catch up with other tasks
        }
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

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.RADIANS);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 2, 0);
        myOtos.setOffset(offset);
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);
        myOtos.calibrateImu();
        myOtos.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.update();
    }
}
