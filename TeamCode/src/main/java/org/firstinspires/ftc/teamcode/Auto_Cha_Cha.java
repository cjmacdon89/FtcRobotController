package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="Auto_Cha_Cha", group="")
public class Auto_Cha_Cha extends LinearOpMode {

    ElapsedTime timmer = new ElapsedTime();
    DcMotor LB_Drive;
    DcMotor RB_Drive;
    DcMotor LF_Drive;
    DcMotor RF_Drive;
    DcMotor Elevator;
    Servo Claw;
    double power = 0.25;
    double park = 0;
    double distanceSensor = 500;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;

    static final double FEET_PER_METER = 3.28084;

    double current_position = 0;
    double Elevator_Position = 0;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Motor Mapping
        LB_Drive = hardwareMap.get(DcMotor.class, "LB_Drive");
        RB_Drive = hardwareMap.get(DcMotor.class, "RB_Drive");
        LF_Drive = hardwareMap.get(DcMotor.class, "LF_Drive");
        RF_Drive = hardwareMap.get(DcMotor.class, "RF_Drive");
        Elevator = hardwareMap.get(DcMotor.class, "Elevator");
        Claw = hardwareMap.get(Servo.class, "Claw");

        //setting the direction of motors
        LB_Drive.setDirection(DcMotor.Direction.FORWARD);
        RB_Drive.setDirection(DcMotor.Direction.REVERSE);
        LF_Drive.setDirection(DcMotor.Direction.FORWARD);
        RF_Drive.setDirection(DcMotor.Direction.FORWARD);
        Claw.setPosition(0.38);

        // Setup Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU.
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
        telemetry.addData("Mode", "waiting for start");
        telemetry.update();
        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        Claw.setPosition(0.38);
        // wait for start button.

        telemetry.addData("Mode", "running");
        telemetry.update();

        right_drive(750, power, 0);
        stop_robot();

        //Lyrics
        //To the right now

        right_drive(50, power, 0);
        stop_robot();

        //To the left

        left_drive(50, power, 0);
        stop_robot();

        //Take it back now y'all

        backward_drive(50, power, 0);
        stop_robot();

        //One hop this time

        Elevator_Position = 1900;
        Elevator.setTargetPosition((int) Elevator_Position);
        Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        current_position = Elevator.getCurrentPosition();
        Elevator.setPower(1.0);

        sleep(500);
        Elevator.setPower(0);

        //One hop this time

        Elevator_Position = 1900;
        Elevator.setTargetPosition((int) Elevator_Position);
        Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        current_position = Elevator.getCurrentPosition();
        Elevator.setPower(1.0);

        sleep(500);
        Elevator.setPower(0);


        //Right foot two stomps

        turnright_drive(50, power, 0);
        stop_robot();
        turnleft_drive(50, power, 0);
        stop_robot();

        turnright_drive(50, power, 0);
        stop_robot();
        turnleft_drive(50, power, 0);
        stop_robot();

        //Left foot two stomps

        turnleft_drive(50, power, 0);
        stop_robot();
        turnright_drive(50, power, 0);
        stop_robot();

        turnleft_drive(50, power, 0);
        stop_robot();
        turnright_drive(50, power, 0);
        stop_robot();

        //Slide to the left

        left_drive(50, power, 0);
        stop_robot();

        //Slide to the right

        right_drive(50, power, 0);
        stop_robot();

        //Criss cross

        turnleft_drive(50, power, 0);
        stop_robot();
        turnright_drive(50, power, 0);
        stop_robot();

        turnleft_drive(50, power, 0);
        stop_robot();
        turnright_drive(50, power, 0);
        stop_robot();

        //Criss cross

        turnleft_drive(50, power, 0);
        stop_robot();
        turnright_drive(50, power, 0);
        stop_robot();

        turnleft_drive(50, power, 0);
        stop_robot();
        turnright_drive(50, power, 0);
        stop_robot();

        //Ch cha real smooth

    }

    // Function to get angle change from gyro
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .02;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    private double checkDirection_90() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction = 0, angle, gain = .02;

        angle = getAngle();

        if (angle == -90);
        else
            correction = -angle + 90;        // reverse sign of angle for correction.


        correction = correction * gain;

        return correction;
    }

    private double checkDirection_180() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .015;

        angle = getAngle_180();

        if (angle == 180)
            correction = 0;             // no adjustment.
        else
            correction = 180 - angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    private double getAngle_180() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = (angles.firstAngle) - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void forward_drive(double run_time, double power, double direction) {
        timmer.reset();
        while (timmer.milliseconds() < run_time) {
            if (direction == 0) {
                correction = checkDirection();
            } else {
                correction = checkDirection_90();
            }
            LB_Drive.setPower(-power + correction);
            RB_Drive.setPower(-power - correction);
            LF_Drive.setPower(-power + correction);
            RF_Drive.setPower(-power - correction);
        }
    }

    private void backward_drive(double run_time, double power, double direction) {
        timmer.reset();
        while (timmer.milliseconds() < run_time) {
            if (direction == 0) {
                correction = checkDirection();
            } else {
                correction = checkDirection_90();
            }
            LB_Drive.setPower(power + correction);
            RB_Drive.setPower(power - correction);
            LF_Drive.setPower(power + correction);
            RF_Drive.setPower(power - correction);
        }
    }

    private void right_drive(double run_time, double power, double direction) {
        timmer.reset();
        while (timmer.milliseconds() < run_time) {
            if (direction == 0) {
                correction = checkDirection();
            } else {
                correction = checkDirection_90();
            }
            LB_Drive.setPower(power + correction);
            RB_Drive.setPower(-power - correction);
            LF_Drive.setPower(-power + correction);
            RF_Drive.setPower(power - correction);
        }
    }

    private void left_drive(double run_time, double power, double direction) {
        timmer.reset();
        while (timmer.milliseconds() < run_time) {
            if (direction == 0) {
                correction = checkDirection();
            } else {
                correction = checkDirection_90();
            }
            LB_Drive.setPower(-power + correction);
            RB_Drive.setPower(power - correction);
            LF_Drive.setPower(power - correction);
            RF_Drive.setPower(-power - correction);
        }
    }

    private void turnright_drive(double run_time, double power, double direction) {
        timmer.reset();
        while (timmer.milliseconds() < run_time) {
            LB_Drive.setPower((power * -1));
            RB_Drive.setPower((power * 1));
            LF_Drive.setPower((power * -1));
            RF_Drive.setPower((power * 1));
        }
    }

    private void turnleft_drive(double run_time, double power, double direction) {
        timmer.reset();
        while (timmer.milliseconds() < run_time && opModeIsActive()) {
            LB_Drive.setPower((power * 1));
            RB_Drive.setPower((power * -1));
            LF_Drive.setPower((power * 1));
            RF_Drive.setPower((power * -1));
        }
    }

    private void right_drive_time(double run_time, double power, double direction) {
        timmer.reset();
        while (timmer.milliseconds() < run_time) {
            if (direction == 0) {
                correction = checkDirection();
            } else {
                correction = checkDirection_90();
            }
            LB_Drive.setPower(power + correction);
            RB_Drive.setPower(-power - correction);
            LF_Drive.setPower(-power + correction);
            RF_Drive.setPower(power - correction);
        }
    }

    private void left_drive_time(double run_time, double power, double direction) {
        timmer.reset();
        while (timmer.milliseconds() < run_time && opModeIsActive()) {
            if (direction == 0) {
                correction = checkDirection();
            } else {
                correction = checkDirection_90();
            }
            LB_Drive.setPower(-power + correction);
            RB_Drive.setPower(power - correction);
            LF_Drive.setPower(power + correction);
            RF_Drive.setPower(-power - correction);
        }
    }

    private void forward_drive_time(double run_time, double power, double direction) {
        timmer.reset();
        while (timmer.milliseconds() < run_time && opModeIsActive()) {
            if (direction == 0) {
                correction = checkDirection();
            } else {
                correction = checkDirection_90();
            }
            LB_Drive.setPower(-power + correction);
            RB_Drive.setPower(-power - correction);
            LF_Drive.setPower(-power + correction);
            RF_Drive.setPower(-power - correction);
        }
    }

    private void stop_robot() {
        LB_Drive.setPower(0);
        RB_Drive.setPower(0);
        LF_Drive.setPower(0);
        RF_Drive.setPower(0);
    }
}

