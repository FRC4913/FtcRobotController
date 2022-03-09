package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "Auto")
@Disabled
public class RoadRunnerTest extends LinearOpMode {
    public static double DISTANCE = 50;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();
        Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(DISTANCE)
                .build();
        Trajectory trajectory3 = drive.trajectoryBuilder(new Pose2d())
                .back(DISTANCE)
                .build();
        Trajectory trajectory4 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(DISTANCE)
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectory(trajectory1);
            drive.followTrajectory(trajectory2);
            drive.followTrajectory(trajectory3);
            drive.followTrajectory(trajectory4);
        }
    }

}