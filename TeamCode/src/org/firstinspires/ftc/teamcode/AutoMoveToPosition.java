package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name = "AutoMoveToPosition")
public class AutoMoveToPosition extends LinearOpMode {

    public DcMotor motorRightFront = null;
    public DcMotor motorRightBack = null;
    public DcMotor motorLeftBack = null;
    public DcMotor motorLeftFront = null;
    public SparkFunOTOS myOtos;
    public Position desiredPosition = new Position(DistanceUnit.INCH, 39.21, 38.37, 0, 0);
    public YawPitchRollAngles desiredOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 33.53, 0, 0, 0);

    // Tuning variables for position control - go too fast with too tight of tolerances and things will get wacky
    final double Y_GAIN  =  0.02  ;
    final double X_GAIN =  0.03 ;
    final double R_GAIN   =  0.01  ;
    final double MAX_AUTO_Y = 0.5;
    final double MAX_AUTO_X = 0.5;
    final double MAX_AUTO_R  = 0.5;
    final double Y_TOLERANCE = 0.1;
    final double X_TOLERANCE = 0.1;
    final double R_TOLERANCE = 0.001;

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
            telemetry.addLine("Desired X: " + desiredPosition.x);
            telemetry.addLine("Desired Y: " + desiredPosition.y);
            telemetry.addLine("Desired Heading: " + desiredOrientation.getYaw(AngleUnit.DEGREES));
            telemetry.addLine();
            telemetry.addLine("X: " + myOtos.getPosition().x);
            telemetry.addLine("Y: " + myOtos.getPosition().y);
            telemetry.addLine("Heading: " + myOtos.getPosition().h);
            telemetry.addLine();

            double  xError = desiredPosition.x - myOtos.getPosition().x;  // Instead of otos, we should get position and yaw from the AprilTag detection
            double  yError = desiredPosition.y - myOtos.getPosition().y;
            double  rError = desiredOrientation.getYaw(AngleUnit.DEGREES) - myOtos.getPosition().h;

            // If everything is working, you should see errors getting smaller and smaller
            telemetry.addLine("X Error: " + xError);
            telemetry.addLine("Y Error: " + yError);
            telemetry.addLine("Heading Error: " + rError);

            // If we are within the error tolerance for any axis, stop moving in that direction
            xError = Math.abs(xError) < X_TOLERANCE ? 0 : xError;
            yError = Math.abs(yError) < Y_TOLERANCE ? 0 : yError;
            rError = Math.abs(rError) < R_TOLERANCE ? 0 : rError;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            xError  = Range.clip(xError * X_GAIN, -MAX_AUTO_X, MAX_AUTO_X);
            yError   = Range.clip(yError * Y_GAIN, -MAX_AUTO_Y, MAX_AUTO_Y) ;
            rError = Range.clip(rError * R_GAIN, -MAX_AUTO_R, MAX_AUTO_R);

            moveRobot(xError, yError, rError);

            telemetry.update();
            sleep(20);  // Sleeping here allows the CPU to catch up with other tasks
        }
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        motorLeftFront.setPower(leftFrontPower);
        motorRightFront.setPower(rightFrontPower);
        motorLeftBack.setPower(leftBackPower);
        motorRightBack.setPower(rightBackPower);
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

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
