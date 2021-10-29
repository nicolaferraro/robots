package warhw2021;

import java.awt.*;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import robocode.AdvancedRobot;
import robocode.BulletHitEvent;
import robocode.HitByBulletEvent;
import robocode.HitRobotEvent;
import robocode.HitWallEvent;
import robocode.RobotDeathEvent;
import robocode.Rules;
import robocode.ScannedRobotEvent;
import robocode.util.Utils;

// API help : https://robocode.sourceforge.io/docs/robocode/robocode/Robot.html

/**
 * Daje - a robot by Nicola Ferraro
 */
public class DajeX extends AdvancedRobot {

    private static final boolean DEBUG = true;

    private static final int MIN_BORDER_DISTANCE = 110;
    private static final int ABSOLUTE_STEP = 5000;
    private static final int RAMMING_DEVIATION_MIN_ENEMIES = 10;
    private static final int RAMMING_DEVIATION_MIN = -10;
    private static final int RAMMING_DEVIATION_MAX = 30;
    private static final double RAMMING_ENERGY_RATIO = 0.5;
    private static final int FIRE_BEARING_DISTANCE = 3;
    private static final double ENEMY_SPEED_OVERESTIMATE = 1.05;
    private static final double WORST_ENEMY_TOLERANCE = 0.55;
    private static final double DEVIATION_FROM_CENTER_MAX = 10;
    private static final double DEVIATION_FROM_CENTER_MULTIPLIER = 3.5;
    private static final double BULLET_PRECISION_SAVING = 1.0;
    private static final double BULLET_ENERGY_RATIO_SAVING = 8.0;
    private static final double BULLET_MIN_ATTEMPTS_ENERGY = 5.0;
    private static final int INITIALIZATION_MAX_TIME = 33;
    private static final double INITIALIZATION_MAX_DAMAGE = 2.9;
    private static final double INITIALIZATION_MAX_ENERGY_LOSS = 8.0;
    private static final double DANGER_FACTOR = 0.4;

    private int direction = ABSOLUTE_STEP;
    private Map<String, RobotProfile> profiles = new HashMap<String, RobotProfile>();
    private Double initialEnergy;

    public DajeX() {
    }

    public void run() {

        setBodyColor(Color.RED);
        setGunColor(Color.YELLOW);
        setRadarColor(Color.YELLOW);
        setBulletColor(Color.YELLOW);
        setScanColor(Color.YELLOW);

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
        double myEnergy = getEnergy();
        double avgEnergy = avgEnergy();
        double energyRatio = 1.0;
        if (myEnergy > 0 && avgEnergy > 0) {
            // Stay a bit away if enemies are stronger
            energyRatio = myEnergy / (avgEnergy * RAMMING_ENERGY_RATIO);
            energyRatio = Math.min(energyRatio, 1.0);
            energyRatio = Math.max(energyRatio, 0.0);
        }
        return RAMMING_DEVIATION_MIN + ratio * energyRatio * (RAMMING_DEVIATION_MAX - RAMMING_DEVIATION_MIN);
    }

    private double avgEnergy() {
        double energySum = 0;
        int num = 0;
        for (RobotProfile p : this.profiles.values()) {
            if (p.dead) {
                continue;
            }
            if (p.energy != null) {
                energySum += p.energy;
                num++;
            }
        }
        if (num == 0) {
            return 0.0;
        }
        return energySum / num;
    }

    private double danger(String robotName) {
        int numMoreDangerous = 0;
        RobotProfile prof = this.profiles.get(robotName);
        if (prof == null || prof.energy == null) {
            if (DEBUG) {
                out.println("Danger " + robotName + ": " + 1.0 + " (empty profile or energy)");
            }
            return 1.0;
        }

        for (RobotProfile p : this.profiles.values()) {
            if (p.dead) {
                continue;
            }
            if (DEBUG && p.energy == null) {
                out.println("WARN: Cannot compute full danger because " + robotName + " has no energy");
            }
            if (p.name != null && p.name.equals(robotName)) {
                continue;
            }
            if (p.energy != null && p.energy > prof.energy) {
                numMoreDangerous ++;
            }
        }
        int total = this.profiles.size();
        if (total == 0) {
            if (DEBUG) {
                out.println("Danger " + robotName + ": " + 1.0 + " (no data)");
            }
            return 1.0;
        }
        double percMoreDangerous = numMoreDangerous * 1.0 / total;
        double danger = 1.0 - percMoreDangerous;
        if (DEBUG) {
            out.println("Danger " + robotName + ": " + danger);
        }
        return danger;
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
            if (worstEnemy == null || correctedEnergyLoss(p) > correctedEnergyLoss(worstEnemy)) {
                worstEnemy = p;
            }
        }
        boolean ok = worstEnemy == null || correctedEnergyLoss(targetProfile) >= correctedEnergyLoss(worstEnemy) * (1 - WORST_ENEMY_TOLERANCE);
        if (DEBUG && worstEnemy != null) {
            out.println("Worst enemy: " + worstEnemy.name + " (" + correctedEnergyLoss(worstEnemy) + "), current target: " + robotName + " (" + correctedEnergyLoss(targetProfile) + ") - targeting: " + ok);
        }
        return ok;
    }

    private double correctedEnergyLoss(RobotProfile profile) {
        double danger = danger(profile.name);
        return profile.energyLoss * (1 + danger*DANGER_FACTOR);
    }

    public void onScannedRobot(ScannedRobotEvent e) {
        if (e.isSentryRobot()) {
            return;
        }

        getProfile(e.getName()).energy = e.getEnergy();

        if (initialize(e)) {
            return;
        }
        
        if(!isPersonalEnemy(e.getName())) {
            return;
        }

        RobotProfile p = getProfile(e.getName());
        
        double bearingDegrees = getHeading() + e.getBearing();
        double bearingDegreesFromRadar = Utils.normalRelativeAngleDegrees(bearingDegrees - getRadarHeading());

        double hitPrecision = 1.0;
        if (p.attemptsEnergy >= BULLET_MIN_ATTEMPTS_ENERGY) {
            hitPrecision = p.hitsEnergy / p.attemptsEnergy;
        }
        
        double bulletPower = Rules.MAX_BULLET_POWER - (1.0 - hitPrecision) * BULLET_PRECISION_SAVING;
        bulletPower = Math.min(getEnergy() / BULLET_ENERGY_RATIO_SAVING, bulletPower);
        bulletPower = Math.min(Rules.MAX_BULLET_POWER, bulletPower);
        bulletPower = Math.max(Rules.MIN_BULLET_POWER, Math.min(e.getEnergy()/4, bulletPower));
        
        Double lastEnergy = p.energy;
        if (lastEnergy != null) {
            double diff = lastEnergy - e.getEnergy();
            if (p.goAwayStrategy && diff >= 0.1 && diff <= 3.0) {
                reverseDirection();
            }
        }
        
        double bulletSpeed = (20.0 - 3.0 * bulletPower);

        int maxIterations = 50;
        int iterations = 0;
        double time = 0.0;
        while ((++iterations) <= maxIterations) {
            Point enemyPos = getFuturePoint(e, time);
            if (Utils.isNear(time, 0.0)) {
                p.lastLocation = enemyPos;
            }
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
            out.println("Iterations=" + iterations + ", time=" + time + ", Predicted x=" + predictedX + ", y=" + predictedY);
        }

        double theta = Utils.normalAbsoluteAngle(Math.atan2(predictedX - getX(), predictedY - getY()));
        
        double absoluteBearing = getHeadingRadians() + e.getBearingRadians();
        setTurnRadarRightRadians(Utils.normalRelativeAngle(absoluteBearing - getRadarHeadingRadians()));
        double gunDiffRadians = Utils.normalRelativeAngle(theta - getGunHeadingRadians());
        setTurnGunRightRadians(gunDiffRadians);

        double approachDeviation = rammingDeviation();
        if (DEBUG) {
            out.println("Ramming: " + approachDeviation);
        }

        if (this.direction > 0) {
            setTurnRight(Utils.normalRelativeAngleDegrees(deviate(e.getBearing() + 90 - approachDeviation)));
        } else {
            setTurnRight(Utils.normalRelativeAngleDegrees(deviate(e.getBearing() + 90 + approachDeviation)));
        }

        if (Math.toDegrees(gunDiffRadians) <= FIRE_BEARING_DISTANCE) { 
            if (getGunHeat() == 0 && getEnergy() >= bulletPower) {
                p.attemptsEnergy+=bulletPower;
                if (DEBUG) {
                    out.println("Fire against " + p.name + "(" + bulletPower + ")");
                }
                fire(bulletPower);
            }
        }

        if (bearingDegreesFromRadar == 0) {
            scan();
        }
    }

    private boolean initialize(ScannedRobotEvent e) {
        if (initialEnergy == null) {
            initialEnergy = getEnergy();
        }

        if (getTime() > INITIALIZATION_MAX_TIME) {
            return false;
        }

        if (getEnergy() <= initialEnergy - INITIALIZATION_MAX_ENERGY_LOSS) {
            return false;
        }

        double sumDamage = 0;
        for (RobotProfile p : this.profiles.values()) {
            sumDamage += p.energyLoss;
        }
        if (sumDamage > INITIALIZATION_MAX_DAMAGE) {
            return false;
        }

        if (DEBUG) {
            out.println("Initialization phase (" + getTime() + " / " + INITIALIZATION_MAX_TIME + ")...");
        }

        RobotProfile sp = getProfile(e.getName());
        Point l = getFuturePoint(e, 0.0);
        sp.lastLocation = l;

        List<Point> others = new ArrayList<>();
        for (RobotProfile p : this.profiles.values()) {
            if (p.dead || p.lastLocation == null) {
                continue;
            }
            others.add(p.lastLocation);
        }
        if (others.size() == 0) {
            others.add(new Point(getX(), getY()));
        }

        double sumx=0, sumy=0;
        for (Point loc : others) {
            sumx += loc.x;
            sumy += loc.y;
        }
        Point center = new Point(sumx / others.size(), sumy / others.size());

        double escapex, escapey;
        if (center.x < getBattleFieldWidth() / 2) {
            escapex = center.x + getBattleFieldWidth()/2;
        } else {
            escapex = center.x - getBattleFieldWidth()/2;
        }
        if (center.y < getBattleFieldHeight() / 2) {
            escapey = center.y + getBattleFieldHeight()/2;
        } else {
            escapey = center.y - getBattleFieldHeight()/2;
        }
        
        escapex = Math.min(Math.max(MIN_BORDER_DISTANCE, escapex), getBattleFieldWidth() - MIN_BORDER_DISTANCE);
        escapey = Math.min(Math.max(MIN_BORDER_DISTANCE, escapey), getBattleFieldHeight() - MIN_BORDER_DISTANCE);
        
        double bearing = getBearingDegrees(escapex, escapey);
        setTurnRight(bearing);
        setTurnRadarRight(360);
        if (DEBUG) {
            double distance = Math.sqrt(Math.pow(getX() - escapex, 2) + Math.pow(getY() - escapey, 2));
            out.println("Escape location x=" + escapex + ", y=" + escapey + ", distance=" + distance);
        }

        // Set closest target as next
        RobotProfile nextTarget = null;
        double nextTargetDistance = Double.MAX_VALUE;
        for (RobotProfile p : this.profiles.values()) {
            p.energyLoss = 0; // reset
            if (p.lastLocation == null) {
                continue;
            }
            double d = Math.sqrt(Math.pow(getX() - p.lastLocation.x, 2) + Math.pow(getY() - p.lastLocation.y, 2));
            if (d < nextTargetDistance) {
                nextTargetDistance = d;
                nextTarget = p;
            }
        }
        if (nextTarget != null) {
            nextTarget.energyLoss=1.0;
        }

        return true;
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
        double minDistance = 15;
        if (Math.abs(getX() - getBattleFieldWidth()/2) <= minDistance && Math.abs(getY() - getBattleFieldHeight()/2) <= minDistance) {
            // Keep the bearing when close to the center
            return bearing;
        }

        double maxDeviationDegrees = Math.min(DEVIATION_FROM_CENTER_MAX, getOthers() * DEVIATION_FROM_CENTER_MULTIPLIER);

        double centerBearing = getBearingDegrees(getBattleFieldWidth() / 2, getBattleFieldHeight() / 2);
        
        double bearingDiff = bearing - centerBearing;

        double modified = bearing;
        if (bearingDiff >=0 && bearingDiff <= maxDeviationDegrees) {
            modified = centerBearing + maxDeviationDegrees; 
        } else if (bearingDiff <=0 && bearingDiff >= -maxDeviationDegrees){
            modified = centerBearing - maxDeviationDegrees;
        }
        if (DEBUG && Math.abs(modified - bearing) >= 0.1) {
            out.println("Deviation original=" + bearing + ", modified=" + modified + ", num=" + getOthers() + ", max=" + maxDeviationDegrees + ", current=" + (modified - bearing));
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
        p.energyLoss+=e.getPower();
        p.goAwayStrategy = !p.goAwayStrategy;
        reverseDirection();
    }

    @Override
    public void onRobotDeath(RobotDeathEvent e) {
        RobotProfile p = getProfile(e.getName());
        p.dead = true;
    }

    @Override
    public void onBulletHit(BulletHitEvent event) {
        getProfile(event.getName()).hitsEnergy+=event.getBullet().getPower();
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
            profile = new RobotProfile(name);
            this.profiles.put(name, profile);
        }
        return profile;
    }

    private static class RobotProfile {
        private String name;
        private Double energy;
        private boolean goAwayStrategy = true;
        private double energyLoss;
        private boolean dead;
        private double attemptsEnergy;
        private double hitsEnergy;
        private Point lastLocation;
        
        private RobotProfile(String name) {
            this.name = name;
        }
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
