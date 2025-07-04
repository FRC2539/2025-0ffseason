// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static frc.robot.constants.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {

    private final VisionConsumer consumer;
    private final VisionIO[] io;

    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    private final double STD_DEV_FACTOR_MT2A = 0.0000206;
    private final double STD_DEV_FACTOR_MT2C = 0.000469;

    private final double STD_DEV_FACTOR_MT1A = 0.001401;
    private final double STD_DEV_FACTOR_MT1C = 0;

    private final double ANGULAR_STD_DEV_MT1A = 0.0264;
    private final double ANGULAR_STD_DEV_MT1C = 0.5;

    private final double ANGULAR_STD_DEV_MT2 = Double.POSITIVE_INFINITY;

    private double linearStdDev;
    private double angularStdDev;
    private final double HEIGHT_CONSTANT_CORAL = 1.0;

    public VisionSubsystem(VisionConsumer consumer, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] =
                    new Alert(
                            "Vision camera " + Integer.toString(i) + " is disconnected.",
                            AlertType.kWarning);
        }
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.tx();
    }

    public TargetObservation getLastTargetObersevation(int camera) {
        return inputs[camera].latestTargetObservation;
    }

    public int[] getTagIDs(int cameraIndex) {
        return inputs[cameraIndex].tagIds;
    }

    public Optional<PoseObservation> getNewestPoseObservation(int cameraIndex) {
        PoseObservation[] observations = inputs[cameraIndex].poseObservations;
        if (observations.length != 0) {
            return Optional.of(observations[0]);
        } else {
            return Optional.empty();
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : inputs[cameraIndex].poseObservations) {
                // Check whether to reject pose
                boolean rejectPose =
                        observation.tagCount() == 0 // Must have at least one tag
                                || (observation.tagCount() == 1
                                        && observation.ambiguity()
                                                > maxAmbiguity) // Cannot be high ambiguity
                                || Math.abs(observation.pose().getZ())
                                        > maxZError // Must have realistic Z coordinate

                                // Must be within the field boundaries
                                || observation.pose().getX() < 0.0
                                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                                || observation.pose().getY() < 0.0
                                || observation.pose().getY() > aprilTagLayout.getFieldWidth()
                                || observation.type() == PoseObservationType.MEGATAG_1;

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                // C refers to a constant that is added. A refers to a scalar constant.
                // Like this -> ((A * calculations) + c)

                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev =
                            (STD_DEV_FACTOR_MT2A
                                            * (Math.pow(observation.averageTagDistance(), 2.0)
                                                    / observation.tagCount()))
                                    + STD_DEV_FACTOR_MT2C;
                    angularStdDev = ANGULAR_STD_DEV_MT2;
                } else {
                    linearStdDev =
                            (STD_DEV_FACTOR_MT1A
                                            * (Math.pow(observation.averageTagDistance(), 2.0)
                                                    / observation.tagCount()))
                                    + STD_DEV_FACTOR_MT1C;
                    angularStdDev =
                            (ANGULAR_STD_DEV_MT1A
                                            * (Math.pow(observation.averageTagDistance(), 2.0)
                                                    / observation.tagCount()))
                                    + ANGULAR_STD_DEV_MT1C;
                }

                final double stdDevFactor = 10;

                linearStdDev = linearStdDevBaseline * stdDevFactor;
                angularStdDev = angularStdDevBaseline * stdDevFactor;
                // if (observation.type() == PoseObservationType.MEGATAG_2) {
                //     linearStdDev *= linearStdDevMegatag2Factor; //
                //     angularStdDev *= angularStdDevMegatag2Factor;
                // }
                if (cameraIndex < cameraStdDevFactors.length) {
                    linearStdDev *= cameraStdDevFactors[cameraIndex];
                    angularStdDev *= cameraStdDevFactors[cameraIndex];
                }

                // Send vision observation
                consumer.accept(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }

            // Log camera datadata
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
                    tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput(
                "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPoses",
                allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted",
                allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesRejected",
                allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
        Logger.recordOutput("Vision/Summary/CoralVerticality", isCoralVertical(0));
        Logger.recordOutput(
                "Vision/Summary/CoralVerticality/CoralHorizontal",
                inputs[0].latestTargetObservation.getTargetHorizontalExtentPixels());
        Logger.recordOutput(
                "Vision/Summary/CoralVerticality/CoralVertical",
                inputs[0].latestTargetObservation.getTargetVerticalExtentPixels());
        Logger.recordOutput("Vision/Summary/X", getTagIDs(2));
        Logger.recordOutput("Vision/Summary/Y", getLastTargetObersevation(2).ty());
    }

    // is the coral vertical
    public boolean isCoralVertical(int cameraIndex) {
        double targetHorizontalExtentPixels =
                Math.abs(
                        inputs[cameraIndex].latestTargetObservation
                                .getTargetHorizontalExtentPixels());
        double targetVerticalExtentPixels =
                Math.abs(
                        inputs[cameraIndex].latestTargetObservation
                                .getTargetVerticalExtentPixels());
        // inputs[cameraIndex].latestTargetObservation.tx()
        if (targetVerticalExtentPixels > (targetHorizontalExtentPixels * HEIGHT_CONSTANT_CORAL)) {
            return true;
        } else {
            return false;
        }
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
                Pose2d visionRobotPoseMeters,
                double timestampSeconds,
                Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
