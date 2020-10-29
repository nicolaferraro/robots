package fusewarhw2020;

import java.awt.*;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import robocode.AdvancedRobot;
import robocode.BulletHitEvent;
import robocode.BulletMissedEvent;
import robocode.HitByBulletEvent;
import robocode.HitRobotEvent;
import robocode.HitWallEvent;
import robocode.RobotDeathEvent;
import robocode.Rules;
import robocode.ScannedRobotEvent;
import robocode.util.Utils;

// API help : https://robocode.sourceforge.io/docs/robocode/robocode/Robot.html

/**
 * TwiceInARow - a robot by (your name here)
 */
public class TwiceInARow extends AdvancedRobot {

    private static final boolean DEBUG = true;

    private static final int MIN_BORDER_DISTANCE = 110;
    private static final int ABSOLUTE_STEP = 5000;
    private static final int RAMMING_DEVIATION_MIN_ENEMIES = 10;
    private static final int RAMMING_DEVIATION_MIN = 0;
    private static final int RAMMING_DEVIATION_MAX = 40;
    private static final int FIRE_BEARING_DISTANCE = 3;
    private static final double ENEMY_SPEED_OVERESTIMATE = 1.05;
    private static final double WORST_ENEMY_TOLERANCE = 0.3;
    private static final boolean DEVIATION_FROM_CENTER = false;

    private int direction = ABSOLUTE_STEP;
    private Map<String, RobotProfile> profiles = new HashMap<String, RobotProfile>();
    private int totalHits = 0;
    private int totalMisses = 0;

    public TwiceInARow() {
    }

    public void run() {

        setBodyColor(Color.BLACK);
        setGunColor(Color.RED);
        setRadarColor(Color.BLACK);
        setBulletColor(Color.RED);
        setScanColor(Color.BLACK);

        setAdjustRadarForRobotTurn(true);
        setAdjustGunForRobotTurn(true);
        setAdjustRadarForGunTurn(true);

        boolean farFromBorder = farFromBorder();

        setAhead(direction);
        setTurnRadarRight(360);

        while (true) {
            boolean nowFarFromBorder = farFromBorder();
            if (!nowFarFromBorder && farFromBorder) {
                reverseDirection();
            }
            farFromBorder = nowFarFromBorder;

            if (getRadarTurnRemaining() == 0.0) {
                setTurnRadarRight(360);
            }

            execute();
        }
    }

    /**
     * onHitWall:  There is a small chance the robot will still hit a wall
     */
    public void onHitWall(HitWallEvent e) {
        // Bounce off!
        reverseDirection();
    }

    public void reverseDirection() {
        this.direction = -direction;
        setAhead(this.direction);
    }

    private double rammingDeviation() {
        int enemies = Math.min(getOthers(), RAMMING_DEVIATION_MIN_ENEMIES);
        double ratio = 1.0 - (enemies * 1.0 / RAMMING_DEVIATION_MIN_ENEMIES);
        System.out.println("AAA: " + enemies + " - " + ratio);
        return RAMMING_DEVIATION_MIN + ratio * (RAMMING_DEVIATION_MAX - RAMMING_DEVIATION_MIN);
    }

    private boolean isPersonalEnemy(String robotName) {
        RobotProfile targetProfile = getProfile(robotName);
        if (targetProfile.dead) {
            return false;
        }
        RobotProfile worstEnemy = null;
        for (RobotProfile p : this.profiles.values()) {
            if (p.dead) {
                continue;
            }
            if (worstEnemy == null || p.hitsByRobot > worstEnemy.hitsByRobot) {
                worstEnemy = p;
            }
        }
        return worstEnemy == null || targetProfile.hitsByRobot >= worstEnemy.hitsByRobot * (1 - WORST_ENEMY_TOLERANCE);
    }

    public void onScannedRobot(ScannedRobotEvent e) {
        if (e.isSentryRobot()) {
            return;
        }
        
        if(!isPersonalEnemy(e.getName())) {
            return;
        }

        
        double bearingDegrees = getHeading() + e.getBearing();
        double bearingDegreesFromRadar = Utils.normalRelativeAngleDegrees(bearingDegrees - getRadarHeading());

        double bulletsOutcomes = totalHits + totalMisses;
        double hitPrecision = 1.0;
        if (bulletsOutcomes > 0) {
            hitPrecision = ((double) totalHits) / bulletsOutcomes;
        }
        double wantedBulletPower = Rules.MAX_BULLET_POWER - (1.0 - hitPrecision) * 0.5;
        double bulletPower = Math.max(Rules.MIN_BULLET_POWER, Math.min(e.getEnergy()/4, wantedBulletPower));
        
        RobotProfile p = getProfile(e.getName());
        Double lastEnergy = p.energy;
        if (lastEnergy != null) {
            double diff = lastEnergy - e.getEnergy();
            if (p.goAwayStrategy && diff >= 0.1 && diff <= 3.0) {
                reverseDirection();
            }
        }
        getProfile(e.getName()).energy = e.getEnergy();
        
        double bulletSpeed = (20.0 - 3.0 * bulletPower);

        int maxIterations = 50;
        int iterations = 0;
        double time = 0.0;
        while ((++iterations) <= maxIterations) {
            Point enemyPos = getFuturePoint(e, time);
            double oldTime = time;
            time = Point2D.distance(getX(), getY(), enemyPos.getX(), enemyPos.getY()) / bulletSpeed;
            if (Utils.isNear(time, oldTime)) {
                break;
            }
        }

        Point predictedPoint = getFuturePoint(e, time);
        double predictedX = predictedPoint.getX();
        double predictedY = predictedPoint.getY();

        if (DEBUG) {
            System.out.println("Iterations=" + iterations + ", time=" + time + ", Predicted x=" + predictedX + ", y=" + predictedY);
        }

        double theta = Utils.normalAbsoluteAngle(Math.atan2(predictedX - getX(), predictedY - getY()));
        
        double absoluteBearing = getHeadingRadians() + e.getBearingRadians();
        setTurnRadarRightRadians(Utils.normalRelativeAngle(absoluteBearing - getRadarHeadingRadians()));
        double gunDiffRadians = Utils.normalRelativeAngle(theta - getGunHeadingRadians());
        setTurnGunRightRadians(gunDiffRadians);

        double approachDeviation = rammingDeviation();
        if (DEBUG) {
            System.out.println("Ramming: " + approachDeviation);
        }

        if (this.direction > 0) {
            setTurnRight(Utils.normalRelativeAngleDegrees(deviate(e.getBearing() + 90 - approachDeviation)));
        } else {
            setTurnRight(Utils.normalRelativeAngleDegrees(deviate(e.getBearing() + 90 + approachDeviation)));
        }

        if (Math.toDegrees(gunDiffRadians) <= FIRE_BEARING_DISTANCE) { 
            if (getGunHeat() == 0 && getEnergy() >= bulletPower) {
                fire(bulletPower);
            }
        }

        if (bearingDegreesFromRadar == 0) {
            scan();
        }
    }

    private Point getFuturePoint(ScannedRobotEvent e, double time) {
        double absoluteBearing = getHeadingRadians() + e.getBearingRadians();
        double enemyX = getX() + e.getDistance() * Math.sin(absoluteBearing);
        double enemyY = getY() + e.getDistance() * Math.cos(absoluteBearing);

        double velocity = e.getVelocity() * ENEMY_SPEED_OVERESTIMATE;

        double predictedX = enemyX + Math.sin(e.getHeadingRadians()) * velocity * time;
        double predictedY = enemyY + Math.cos(e.getHeadingRadians()) * velocity * time;

        if (predictedX < 18.0
            || predictedY < 18.0
            || predictedX > getBattleFieldWidth() - 18.0
            || predictedY > getBattleFieldHeight() - 18.0) {
            predictedX = Math.min(Math.max(18.0, predictedX),
                                    getBattleFieldWidth() - 18.0);
            predictedY = Math.min(Math.max(18.0, predictedY),
                                    getBattleFieldHeight() - 18.0);
        }

        return new Point(predictedX, predictedY);
    }

    private double deviate(double bearing) {
        if (!DEVIATION_FROM_CENTER) {
            return bearing;
        }

        double minDistance = 15;
        if (Math.abs(getX() - getBattleFieldWidth()/2) <= minDistance && Math.abs(getY() - getBattleFieldHeight()/2) <= minDistance) {
            // Keep the bearing when close to the center
            return bearing;
        }

        double maxDeviationDegrees = Math.min(45, getOthers() * 5);

        double centerBearing = getBearingDegrees(getBattleFieldWidth() / 2, getBattleFieldHeight() / 2);
        
        double bearingDiff = bearing - centerBearing;

        double modified = bearing;
        if (bearingDiff >=0 && bearingDiff <= maxDeviationDegrees) {
            modified = centerBearing + maxDeviationDegrees; 
        } else if (bearingDiff <=0 && bearingDiff >= -maxDeviationDegrees){
            modified = centerBearing - maxDeviationDegrees;
        }
        if (DEBUG && Math.abs(modified - bearing) >= 0.1) {
            System.out.println("original=" + bearing + ", modified=" + modified + ", num=" + getOthers() + ", max=" + maxDeviationDegrees + ", current=" + (modified - bearing));
        }
        return modified;
    }
    
    double getBearingDegrees(double x, double y) {
        double b = Math.PI/2 - Math.atan2(y - this.getY(), x - this.getX());
        double brad = Utils.normalRelativeAngle(b - this.getHeadingRadians());
        return Utils.normalRelativeAngleDegrees(brad / Math.PI * 180);
    }


    @Override
    public void onHitByBullet(HitByBulletEvent e) {
        RobotProfile p = getProfile(e.getName());
        p.hitsByRobot++;
        p.goAwayStrategy = !p.goAwayStrategy;
    }

    @Override
    public void onRobotDeath(RobotDeathEvent e) {
        RobotProfile p = getProfile(e.getName());
        p.dead = true;
    }

    @Override
    public void onBulletHit(BulletHitEvent event) {
        this.totalHits++;
    }

    @Override
    public void onBulletMissed(BulletMissedEvent event) {
        this.totalMisses++;
    }

    

    /**
     * onHitRobot:  Back up!
     */
    public void onHitRobot(HitRobotEvent e) {
        // If we're moving the other robot, reverse!
        if (e.isMyFault()) {
            reverseDirection();
        }
    }

    private boolean farFromBorder() {
        return getX() > MIN_BORDER_DISTANCE &&
               getY() > MIN_BORDER_DISTANCE &&
               getBattleFieldWidth() - getX() > MIN_BORDER_DISTANCE &&
               getBattleFieldHeight() - getY() > MIN_BORDER_DISTANCE;
    }

    private RobotProfile getProfile(String name) {
        RobotProfile profile = this.profiles.get(name);
        if (profile == null) {
            profile = new RobotProfile();
            this.profiles.put(name, profile);
        }
        return profile;
    }

    private static class RobotProfile {
        private Double energy;
        private boolean goAwayStrategy = true;
        private int hitsByRobot;
        private boolean dead;
    }

    private static class Point {
        private double x,y;
        Point(double x, double y) {
            this.x = x;
            this.y = y;
        }
        public double getX() {
            return x;
        }
        public double getY() {
            return y;
        }
    }
}
