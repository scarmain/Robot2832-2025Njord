package frc.robot.vision;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.livoniawarriors.UtilFunctions;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagCamera {
  public static final Time MAX_LATENCY = Milliseconds.of(250);
  public static final Time RUNNING_DISCONECT_TIME = Seconds.of(5);
  public static final Time STARTUP_DISCONECT_TIME = Seconds.of(30);

  /** Latency alert to use when high latency is detected. */
  public final Alert latencyAlert;

  /** Latency alert to use when camera is disconnected. */
  public final Alert disconnectAlert;

  /** Camera instance for comms. */
  public final PhotonCamera camera;
  /** Pose estimator for camera. */
  public final PhotonPoseEstimator poseEstimator;
  /** Standard Deviation for single tag readings for pose estimation. */
  private final Matrix<N3, N1> singleTagStdDevs;
  /** Standard deviation for multi-tag readings for pose estimation. */
  private final Matrix<N3, N1> multiTagStdDevs;
  /** Transform of the camera rotation and translation relative to the center of the robot */
  public final Transform3d robotToCamTransform;
  /** Current standard deviations used. */
  public Matrix<N3, N1> curStdDevs;
  /** Estimated robot pose. */
  public Optional<EstimatedRobotPose> estimatedRobotPose;
  /** Simulated camera instance which only exists during simulations. */
  public PhotonCameraSim cameraSim;
  /** Results list to be updated periodically and cached to avoid unnecessary queries. */
  public List<PhotonPipelineResult> resultsList;
  /** Last read from the camera timestamp to prevent lag due to slow data fetches. */
  @AutoLogOutput(key = "Camera {name}/lastReadTimestamp")
  private double lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);

  private double lastTagTimestamp;

  @AutoLogOutput(key = "Camera {name}/distToTag12")
  double distTo12;

  @AutoLogOutput(key = "Camera {name}/distToTag18")
  double distTo18;

  @AutoLogOutput(key = "Camera {name}/numTagsSeen")
  int numTagsSeen;

  @SuppressWarnings("unused")
  private final String name;

  private IntegerSubscriber heartbeatSub;
  private double lastHeartbeatTimestamp;

  // this is needed to save the result of the listener so we don't lose the handle
  @SuppressWarnings("unused")
  private int heartbeatHandle;

  private CameraType cameraType;

  /**
   * Construct a Photon Camera class with help. Standard deviations are fake values, experiment and
   * determine estimation noise on an actual robot.
   *
   * @param name Name of the PhotonVision camera found in the PV UI.
   * @param robotToCamRotation {@link Rotation3d} of the camera.
   * @param robotToCamTranslation {@link Translation3d} relative to the center of the robot.
   * @param singleTagStdDevs Single AprilTag standard deviations of estimated poses from the camera.
   * @param multiTagStdDevsMatrix Multi AprilTag standard deviations of estimated poses from the
   *     camera.
   */
  public AprilTagCamera(
      String name,
      Rotation3d robotToCamRotation,
      Translation3d robotToCamTranslation,
      Matrix<N3, N1> singleTagStdDevs,
      Matrix<N3, N1> multiTagStdDevsMatrix) {
    this.name = name;
    AutoLogOutputManager.addObject(this);

    latencyAlert =
        new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);
    disconnectAlert = new Alert("'" + name + "' Camera is disconnected.", AlertType.kError);
    lastHeartbeatTimestamp = -1;
    cameraType = CameraType.APRILTAGS;

    camera = new PhotonCamera(name);

    resultsList = new ArrayList<>();
    estimatedRobotPose = Optional.empty();

    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

    poseEstimator =
        new PhotonPoseEstimator(
            Vision.fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, robotToCamTransform);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    this.singleTagStdDevs = singleTagStdDevs;
    this.multiTagStdDevs = multiTagStdDevsMatrix;

    // add a listener to listen for heartbeat changes
    heartbeatSub = camera.getCameraTable().getIntegerTopic("heartbeat").subscribe(0);
    heartbeatHandle =
        NetworkTableInstance.getDefault()
            .addListener(
                heartbeatSub,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> {
                  lastHeartbeatTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
                });

    if (Robot.isSimulation()) {
      SimCameraProperties cameraProp = new SimCameraProperties();
      // A 640 x 480 camera with a 100 degree diagonal FOV.
      cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
      // Approximate detection noise with average and standard deviation error in pixels.
      cameraProp.setCalibError(0.25, 0.08);
      // Set the camera image capture framerate (Note: this is limited by robot loop rate).
      cameraProp.setFPS(60);
      // The average and standard deviation in milliseconds of image data latency.
      // these have been lowered from 35 to stop flickering as the camera is only rendered on new
      // images
      cameraProp.setAvgLatencyMs(14);
      cameraProp.setLatencyStdDevMs(5);

      cameraSim = new PhotonCameraSim(camera, cameraProp);
      cameraSim.enableDrawWireframe(true);
    }
  }

  /**
   * Add camera to {@link VisionSystemSim} for simulated photon vision.
   *
   * @param systemSim {@link VisionSystemSim} to use.
   */
  public void addToVisionSim(VisionSystemSim systemSim) {
    if (Robot.isSimulation()) {
      systemSim.addCamera(cameraSim, robotToCamTransform);
    }
  }

  /**
   * Get the result with the least ambiguity from the best tracked target within the Cache. This may
   * not be the most recent result!
   *
   * @return The result in the cache with the least ambiguous best tracked target. This is not the
   *     most recent result!
   */
  public Optional<PhotonPipelineResult> getBestResult() {
    if (resultsList.isEmpty()) {
      return Optional.empty();
    }

    PhotonPipelineResult bestResult = resultsList.get(0);
    double amiguity = bestResult.getBestTarget().getPoseAmbiguity();
    double currentAmbiguity = 0;
    for (PhotonPipelineResult result : resultsList) {
      currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
      if (currentAmbiguity < amiguity && currentAmbiguity > 0) {
        bestResult = result;
        amiguity = currentAmbiguity;
      }
    }
    return Optional.of(bestResult);
  }

  /**
   * Get the latest result from the current cache.
   *
   * @return Empty optional if nothing is found. Latest result if something is there.
   */
  public Optional<PhotonPipelineResult> getLatestResult() {
    return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(0));
  }

  /**
   * Get the estimated robot pose. Updates the current robot pose estimation, standard deviations,
   * and flushes the cache of results.
   *
   * @return Estimated pose.
   */
  public Optional<EstimatedRobotPose> updateCamera() {
    // timeout alerts
    Time currentTime = Microseconds.of(NetworkTablesJNI.now());
    // check for startup timeout
    if (lastHeartbeatTimestamp < 0 && currentTime.gt(STARTUP_DISCONECT_TIME)) {
      disconnectAlert.set(true);
    }
    // check for running timeout
    else if (lastHeartbeatTimestamp > 0
        && (Seconds.of(lastHeartbeatTimestamp).plus(RUNNING_DISCONECT_TIME)).lt(currentTime)) {
      disconnectAlert.set(true);
    } else {
      disconnectAlert.set(false);
    }

    if (cameraType == CameraType.APRILTAGS) {
      // check pose
      updateUnreadResults();
    } else if (cameraType == CameraType.ALGAE) {
      // color detection
      colorDetection();
    } else {
      // do no processing
    }

    return estimatedRobotPose;
  }

  List<ColorMatchResult> colorTargets = new ArrayList<ColorMatchResult>();

  private void colorDetection() {
    // Query the latest result from PhotonVision
    PhotonPipelineResult result = camera.getLatestResult();
    List<PhotonTrackedTarget> targets = result.getTargets();
    colorTargets = new ArrayList<ColorMatchResult>();
    for (PhotonTrackedTarget target : targets) {
      // TODO: Get actual frame sizes
      colorTargets.add(new ColorMatchResult(target, 960, 720));
    }
  }

  public List<ColorMatchResult> getColorTargets() {
    return colorTargets;
  }

  /**
   * Update the latest results, cached with a maximum refresh rate of 1req/15ms. Sorts the list by
   * timestamp.
   */
  private void updateUnreadResults() {
    double mostRecentTimestamp = 0;
    double currentTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
    double debounceTime = Milliseconds.of(15).in(Seconds);

    if ((resultsList.isEmpty() || (currentTimestamp - mostRecentTimestamp >= debounceTime))
        && (currentTimestamp - lastReadTimestamp) >= debounceTime) {
      resultsList =
          Robot.isReal()
              ? camera.getAllUnreadResults()
              : cameraSim.getCamera().getAllUnreadResults();
      lastReadTimestamp = currentTimestamp;
      resultsList.sort(
          (PhotonPipelineResult a, PhotonPipelineResult b) -> {
            return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
          });
      if (!resultsList.isEmpty()) {
        updateEstimatedGlobalPose();
      }
    }
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
   * {@link Cameras#updateEstimationStdDevs}
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  private void updateEstimatedGlobalPose() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var change : resultsList) {
      visionEst = poseEstimator.update(change);
      updateEstimationStdDevs(visionEst, change.getTargets());

      // check if the camera is lagging
      if (change.metadata.getLatencyMillis() > MAX_LATENCY.in(Milliseconds)) {
        latencyAlert.set(true);
      } else {
        latencyAlert.set(false);
      }

      for (var tag : change.targets) {
        lastTagTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
        if (tag.fiducialId == 12) {
          distTo12 = UtilFunctions.getDistance(tag.bestCameraToTarget);
        }
        if (tag.fiducialId == 7) {
          distTo18 = UtilFunctions.getDistance(tag.bestCameraToTarget);
        }
      }
      numTagsSeen = change.targets.size();
    }
    estimatedRobotPose = visionEst;
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = singleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = singleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) {
          continue;
        }
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = singleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (estimatedPose.get().strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
          // estStdDevs = multiTagStdDevs;
        }
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) {
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          // the closer we are, the better we make our standard devs
          estStdDevs = singleTagStdDevs.times(avgDist / 4);
        }
        curStdDevs = estStdDevs;
        SmartDashboard.putNumber("Std Dev", estStdDevs.get(0, 0));
      }
    }
  }

  @AutoLogOutput(key = "Camera {name}/hasTarget")
  public boolean hasTarget() {
    return Microseconds.of(NetworkTablesJNI.now()).in(Seconds) - lastTagTimestamp < 0.25;
  }

  @AutoLogOutput(key = "Camera {name}/isConnected")
  public boolean isConnected() {
    // if we have never seen the camera yet, or if the disconnect alert goes active
    return !(lastHeartbeatTimestamp < 0 || disconnectAlert.get());
  }

  public void setPipeLine(CameraType type) {
    if (type == CameraType.DRIVER_CAM) {
      camera.setDriverMode(true);
    } else {
      camera.setDriverMode(false);

      if (type == CameraType.APRILTAGS) {
        camera.setPipelineIndex(0);
      } else if (type == CameraType.ALGAE) {
        camera.setPipelineIndex(1);
      }
    }
  }

  public CameraType getPipeLineType() {
    return cameraType;
  }
}
