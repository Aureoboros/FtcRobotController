// ========== APRILTAG VISION ALIGNMENT & POSITIONING CODE ==========
// This code isolates all the AprilTag vision processing, alignment, and navigation logic

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.List;

// ========== CONSTANTS ==========

// AprilTag IDs
private static final int RED_APRILTAG_ID = 24;
private static final int BLUE_APRILTAG_ID = 20;
private static final int[] OBELISK_APRILTAG_IDS = {21, 22, 23};

// Field Positions (in feet)
private static final double TAG_20_X = -4.86;  // Blue Goal X
private static final double TAG_20_Y = -4.64;  // Blue Goal Y
private static final double TAG_24_X = 4.86;   // Red Goal X
private static final double TAG_24_Y = 4.64;   // Red Goal Y
private static final double OBELISK_X = 0.0;   // Obelisk X
private static final double OBELISK_Y = 6.0;   // Obelisk Y

// Target Positions (based on alliance) - in inches
private static final double BLUE_TARGET_X_INCHES = 0.0;   // inches
private static final double BLUE_TARGET_Y_INCHES = 3.0;   // inches
private static final double RED_TARGET_X_INCHES = 0.0;    // inches
private static final double RED_TARGET_Y_INCHES = -3.0;   // inches

// Target Positions in feet (for calculations)
private static final double BLUE_TARGET_X = BLUE_TARGET_X_INCHES / 12.0;  // feet
private static final double BLUE_TARGET_Y = BLUE_TARGET_Y_INCHES / 12.0;   // feet
private static final double RED_TARGET_X = RED_TARGET_X_INCHES / 12.0;    // feet
private static final double RED_TARGET_Y = RED_TARGET_Y_INCHES / 12.0;    // feet

// Alignment Constants
private static final double ANGLE_TOLERANCE_DEGREES = 3.0;
private static final double POSITION_TOLERANCE_INCHES = 8.0;
private static final double AUTO_ALIGN_ROTATION_SPEED = 0.15;

// Navigation Speed Constants
private static final double BRAKING_ZONE_INCHES = 18.0;
private static final double AUTO_NAV_MAX_SPEED = 0.25;
private static final double AUTO_NAV_MID_SPEED = 0.15;
private static final double AUTO_NAV_MIN_SPEED = 0.08;
private static final double SLOWDOWN_DISTANCE_FEET = 6.0;
private static final double BRAKING_DISTANCE_FEET = 3.0;

// ========== STATE VARIABLES ==========
private enum Alliance {NONE, RED, BLUE}
private Alliance selectedAlliance = Alliance.NONE;

private AprilTagProcessor aprilTag;
private VisionPortal visionPortal;

private boolean autoAligning = false;
private boolean autoNavigatingToZero = false;

// Robot position tracking (field coordinates)
private double robotX = 0.0;  // feet
private double robotY = 0.0;  // feet
private double robotHeading = 0.0;  // radians

// ========== INITIALIZATION ==========

public void initializeVision() {
    // Initialize AprilTag processor
    aprilTag = new AprilTagProcessor.Builder().build();

    // Initialize vision portal with camera
    visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(
                org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, 
                "Webcam 1"))
            .addProcessor(aprilTag)
            .build();
}

public void closeVision() {
    visionPortal.close();
}

// ========== APRILTAG DETECTION HELPERS ==========

/**
 * Get AprilTag detection by ID
 * @param targetId The ID of the AprilTag to find
 * @return AprilTagDetection object if found, null otherwise
 */
private AprilTagDetection getAprilTagDetection(int targetId) {
    List<AprilTagDetection> detections = aprilTag.getDetections();
    for (AprilTagDetection detection : detections) {
        if (detection.id == targetId) {
            return detection;
        }
    }
    return null;
}

/**
 * Get any visible Obelisk tag (21, 22, or 23)
 * @return AprilTagDetection object if found, null otherwise
 */
private AprilTagDetection getAnyObeliskTag() {
    List<AprilTagDetection> detections = aprilTag.getDetections();
    for (AprilTagDetection detection : detections) {
        for (int obeliskId : OBELISK_APRILTAG_IDS) {
            if (detection.id == obeliskId) {
                return detection;
            }
        }
    }
    return null;
}

// ========== POSITION CALCULATION ==========

/**
 * Calculate robot position from Obelisk tag at known position
 * @param obeliskTag Detected Obelisk AprilTag
 * @param imu IMU for heading reference
 */
private void calculatePositionFromObelisk(AprilTagDetection obeliskTag, IMU imu) {
    calculatePositionFromTag(obeliskTag, OBELISK_X, OBELISK_Y, imu);
}

/**
 * Calculate robot position from any AprilTag at known field position
 * Uses trigonometry to determine robot's field coordinates
 * 
 * @param tag Detected AprilTag
 * @param tagX Known X position of tag on field (feet)
 * @param tagY Known Y position of tag on field (feet)
 * @param imu IMU for heading reference
 */
private void calculatePositionFromTag(AprilTagDetection tag, double tagX, double tagY, IMU imu) {
    // Range: distance from camera to tag (convert inches to feet)
    double rangeToTag = tag.ftcPose.range / 12.0;

    // Bearing: horizontal angle to the tag (convert degrees to radians)
    double bearingToTag = Math.toRadians(tag.ftcPose.bearing);

    // Get current robot heading from IMU
    double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

    // Calculate absolute angle from robot to tag (in field coordinates)
    double absoluteAngleToTag = currentHeading + bearingToTag;

    // Calculate robot position using inverse trigonometry
    // Robot is 'rangeToTag' away at angle 'absoluteAngleToTag' from tag
    robotX = tagX - rangeToTag * Math.sin(absoluteAngleToTag);
    robotY = tagY - rangeToTag * Math.cos(absoluteAngleToTag);
    
    // Note: robotHeading comes from IMU, not calculated here
    // This assumes camera is at robot center (add offset compensation if needed)
}

// ========== AUTO-ALIGNMENT ==========

/**
 * Auto-align to alliance-specific AprilTag
 * Returns movement commands (x, y, rx)
 * 
 * @param imu IMU for heading reference
 * @return double[] {x, y, rx, status} - movement commands and status code
 *         status: 0 = aligned, 1 = aligning, 2 = searching
 */
public double[] performAutoAlignment(IMU imu) {
    int targetTag = (selectedAlliance == Alliance.RED) ? RED_APRILTAG_ID : BLUE_APRILTAG_ID;
    AprilTagDetection detection = getAprilTagDetection(targetTag);

    double x = 0, y = 0, rx = 0;
    double status = 2;  // 2 = searching

    if (detection != null) {
        double yawError = detection.ftcPose.yaw;
        
        if (Math.abs(yawError) < ANGLE_TOLERANCE_DEGREES) {
            // Aligned!
            rx = 0;
            status = 0;  // 0 = aligned
        } else {
            // Rotate to align
            rx = Math.signum(yawError) * AUTO_ALIGN_ROTATION_SPEED;
            status = 1;  // 1 = aligning
        }
    } else {
        // Tag not found - search by rotating
        rx = (selectedAlliance == Alliance.RED) ? 
            AUTO_ALIGN_ROTATION_SPEED * 0.3 : 
            -AUTO_ALIGN_ROTATION_SPEED * 0.3;
        status = 2;  // 2 = searching
    }

    return new double[]{x, y, rx, status};
}

// ========== AUTO-NAVIGATION TO TARGET ==========

/**
 * Navigate to target position based on alliance
 * Returns movement commands (x, y, rx)
 * 
 * @param imu IMU for heading reference
 * @return double[] {x, y, rx, distance} - movement commands and distance to target
 */
public double[] performAutoNavigation(IMU imu) {
    // Determine target based on alliance
    double targetX = (selectedAlliance == Alliance.BLUE) ? BLUE_TARGET_X : RED_TARGET_X;
    double targetY = (selectedAlliance == Alliance.BLUE) ? BLUE_TARGET_Y : RED_TARGET_Y;
    
    // Update position from visible tags
    AprilTagDetection navTag20 = getAprilTagDetection(BLUE_APRILTAG_ID);
    AprilTagDetection navTag24 = getAprilTagDetection(RED_APRILTAG_ID);
    AprilTagDetection navObeliskTag = getAnyObeliskTag();
    
    if (navTag20 != null) {
        calculatePositionFromTag(navTag20, TAG_20_X, TAG_20_Y, imu);
    } else if (navTag24 != null) {
        calculatePositionFromTag(navTag24, TAG_24_X, TAG_24_Y, imu);
    } else if (navObeliskTag != null) {
        calculatePositionFromObelisk(navObeliskTag, imu);
    }
    
    // Calculate distance and angle to target
    double deltaX = targetX - robotX;
    double deltaY = targetY - robotY;
    double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    double distanceInches = distanceToTarget * 12.0;
    double distanceFeet = distanceToTarget;

    double x = 0, y = 0, rx = 0;

    if (distanceInches < POSITION_TOLERANCE_INCHES) {
        // Reached target!
        return new double[]{0, 0, 0, distanceInches};
    }

    // Calculate angle to target (in field coordinates)
    double angleToTarget = Math.atan2(deltaX, deltaY);
    
    // ========== PROGRESSIVE SPEED CONTROL ==========
    double speed = AUTO_NAV_MAX_SPEED;
    
    if (distanceInches < 12.0) {
        // Final braking zone (< 12 inches)
        speed = AUTO_NAV_MIN_SPEED * 0.3;
    } else if (distanceInches < BRAKING_ZONE_INCHES) {
        // Aggressive braking (12-18 inches)
        double brakingRatio = (distanceInches - 12.0) / (BRAKING_ZONE_INCHES - 12.0);
        speed = AUTO_NAV_MIN_SPEED * (0.3 + 0.4 * brakingRatio);
    } else if (distanceFeet < BRAKING_DISTANCE_FEET) {
        // Moderate braking (18 inches - 3 feet)
        double brakingRatio = (distanceFeet - BRAKING_ZONE_INCHES / 12.0) / 
                             (BRAKING_DISTANCE_FEET - BRAKING_ZONE_INCHES / 12.0);
        speed = AUTO_NAV_MIN_SPEED * 0.7 + 
               (AUTO_NAV_MID_SPEED - AUTO_NAV_MIN_SPEED * 0.7) * brakingRatio;
    } else if (distanceFeet < SLOWDOWN_DISTANCE_FEET) {
        // Gradual slowdown (3-6 feet)
        double slowdownRatio = (distanceFeet - BRAKING_DISTANCE_FEET) / 
                              (SLOWDOWN_DISTANCE_FEET - BRAKING_DISTANCE_FEET);
        speed = AUTO_NAV_MID_SPEED + 
               (AUTO_NAV_MAX_SPEED - AUTO_NAV_MID_SPEED) * slowdownRatio;
    }
    
    // Additional slowdown when tags are visible (better detection)
    if (navTag20 != null || navTag24 != null) {
        speed *= 0.9;
    } else if (navObeliskTag == null) {
        speed *= 0.7;
    }

    // Ensure minimum speed (robot must move!)
    double minSpeed;
    if (distanceInches > 36.0) {
        minSpeed = AUTO_NAV_MID_SPEED;
    } else if (distanceInches > 24.0) {
        minSpeed = AUTO_NAV_MIN_SPEED * 1.5;
    } else if (distanceInches > 12.0) {
        minSpeed = AUTO_NAV_MIN_SPEED;
    } else {
        minSpeed = AUTO_NAV_MIN_SPEED * 0.6;
    }
    speed = Math.max(speed, minSpeed);

    // ========== CONVERT TO ROBOT FRAME ==========
    // Calculate robot-relative angle
    double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    robotHeading = heading;
    
    double robotRelativeAngle = angleToTarget - heading;
    // Normalize to [-π, π]
    while (robotRelativeAngle > Math.PI) robotRelativeAngle -= 2 * Math.PI;
    while (robotRelativeAngle < -Math.PI) robotRelativeAngle += 2 * Math.PI;
    
    // Calculate movement components
    x = Math.sin(robotRelativeAngle) * speed;  // Strafe
    y = Math.cos(robotRelativeAngle) * speed;  // Forward/back
    
    // ========== HEADING CORRECTION ==========
    double headingError = angleToTarget - heading;
    while (headingError > Math.PI) headingError -= 2 * Math.PI;
    while (headingError < -Math.PI) headingError += 2 * Math.PI;
    
    // Proportional rotation gain (reduces near target)
    double rotationGain = 0.12;
    if (distanceInches < 24.0) rotationGain = 0.06;
    if (distanceInches < 12.0) rotationGain = 0.03;
    
    rx = headingError * rotationGain;
    
    // Cap rotation speed
    double maxRotationSpeed = 0.12;
    if (distanceInches < 12.0) maxRotationSpeed = 0.06;
    rx = Math.max(Math.min(rx, maxRotationSpeed), -maxRotationSpeed);
    
    return new double[]{x, y, rx, distanceInches};
}

// ========== POSITION UPDATE ==========

/**
 * Update robot position from visible AprilTags
 * Call this periodically to maintain accurate position
 * 
 * @param imu IMU for heading reference
 * @return true if position was updated from a tag, false otherwise
 */
public boolean updatePositionFromTags(IMU imu) {
    AprilTagDetection tag20 = getAprilTagDetection(BLUE_APRILTAG_ID);
    AprilTagDetection tag24 = getAprilTagDetection(RED_APRILTAG_ID);
    AprilTagDetection obeliskTag = getAnyObeliskTag();
    
    if (tag20 != null) {
        calculatePositionFromTag(tag20, TAG_20_X, TAG_20_Y, imu);
        return true;
    } else if (tag24 != null) {
        calculatePositionFromTag(tag24, TAG_24_X, TAG_24_Y, imu);
        return true;
    } else if (obeliskTag != null) {
        calculatePositionFromObelisk(obeliskTag, imu);
        return true;
    }
    
    // Update heading even if no tags visible
    robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    return false;
}

// ========== GETTERS ==========

public double getRobotX() { return robotX; }
public double getRobotY() { return robotY; }
public double getRobotHeading() { return robotHeading; }
public Alliance getAlliance() { return selectedAlliance; }
public boolean isAligning() { return autoAligning; }
public boolean isNavigating() { return autoNavigatingToZero; }

// ========== SETTERS ==========

public void setAlliance(Alliance alliance) { selectedAlliance = alliance; }
public void setAutoAligning(boolean aligning) { autoAligning = aligning; }
public void setAutoNavigating(boolean navigating) { autoNavigatingToZero = navigating; }
public void resetPosition() { 
    robotX = 0.0; 
    robotY = 0.0; 
    robotHeading = 0.0; 
}
