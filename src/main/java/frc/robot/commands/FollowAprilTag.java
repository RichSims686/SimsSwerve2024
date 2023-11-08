// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

public class FollowAprilTag extends SequentialCommandGroup {
    private static final LoggedTunableNumber targetDistance = new LoggedTunableNumber("FollowAprilTag/TargetDistance",
            2.0);

    private Pose2d averagePose = new Pose2d();
    private final double beta = 0.1; 

    public FollowAprilTag(Drive drive, AprilTagVision aprilTagVision) {
        addCommands(
            Commands.runOnce(() -> aprilTagVision.setDemoTagMode(true))
                .andThen(            
                    new DriveToPose(
                        drive,
                        true,
                        () -> {
                            Pose2d fieldToRobot = drive.getPose();
                            Optional<Transform2d> robotToTag = aprilTagVision.getRobotToDemoTag();
                            if (robotToTag.isPresent()) {
                                Pose2d fieldToTag = fieldToRobot.transformBy(robotToTag.get());
                                Pose2d driveToPose = fieldToTag.transformBy(
                                    new Transform2d(
                                        new Translation2d(targetDistance.get(), 0.0), 
                                        new Rotation2d(Math.PI)));

                                // unwrap angular adjustment
                                double prevAngle = averagePose.getRotation().getRadians();
                                double newAngle = averagePose.getRotation().getRadians();
                                double deltaAngle = prevAngle - newAngle;
                                if (deltaAngle > Math.PI) {
                                    deltaAngle -= 2*Math.PI;
                                } else if (deltaAngle < -Math.PI) {
                                    deltaAngle += 2*Math.PI;
                                }
                                newAngle = prevAngle - deltaAngle;

                                if (averagePose.equals(new Pose2d())) {
                                    averagePose = driveToPose;
                                } else {
                                    averagePose = new Pose2d(
                                        new Translation2d(
                                            (1-beta)*averagePose.getX() + beta*driveToPose.getX(),
                                            (1-beta)*averagePose.getY() + beta*driveToPose.getY()),
                                        new Rotation2d(
                                            ((1-beta)*prevAngle + beta*newAngle)
                                        ));
                                }
                                Logger.recordOutput("AprilTagVision/TagPose", fieldToTag);
                                Logger.recordOutput("AprilTagVision/DriveToPoseAvg", averagePose);                                
                                Logger.recordOutput("AprilTagVision/DriveToPose", driveToPose);
                                return driveToPose;
                            } else {
                                // stay where you are if you don't see the demo tag
                                Logger.recordOutput("AprilTagVision/TagPose", new double[] {});
                                Logger.recordOutput("AprilTagVision/DriveToPoseAvg", new double[] {});                                
                                Logger.recordOutput("AprilTagVision/DriveToPose", new double[] {});
                                return drive.getPose();
                            }
                        }
                    )
                )
                .finallyDo((boolean interrupted) -> aprilTagVision.setDemoTagMode(true)
            )
        );
    }
}