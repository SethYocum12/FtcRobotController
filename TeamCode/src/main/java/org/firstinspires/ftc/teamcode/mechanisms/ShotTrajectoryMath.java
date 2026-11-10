package org.firstinspires.ftc.teamcode.mechanisms;

import java.util.List;

/** Stationary projectile geometry. Units are inches and seconds; +Z is up. */
public final class ShotTrajectoryMath {
    public static final double GRAVITY = 386.08858;
    private static final double EPS = 1e-9;

    private ShotTrajectoryMath() { }

    public static final class Vec3 {
        public final double x, y, z;
        public Vec3(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
        public Vec3 plus(Vec3 other) { return new Vec3(x + other.x, y + other.y, z + other.z); }
        public Vec3 minus(Vec3 other) { return new Vec3(x - other.x, y - other.y, z - other.z); }
        public Vec3 times(double scale) { return new Vec3(x * scale, y * scale, z * scale); }
        public double dot(Vec3 other) { return x * other.x + y * other.y + z * other.z; }
        public Vec3 cross(Vec3 other) {
            return new Vec3(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
        }
        public double length() { return Math.sqrt(dot(this)); }
        public boolean finite() { return Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(z); }
    }

    /** Rigid right-handed rotation, applied to column vectors. */
    public static final class Rotation {
        private final double[] m;
        public Rotation(double... entries) {
            if (entries.length != 9) throw new IllegalArgumentException("Need nine rotation entries");
            m = entries.clone();
        }
        public static Rotation yawPitchRoll(double yaw, double pitch, double roll) {
            double cy = Math.cos(yaw), sy = Math.sin(yaw);
            double cp = Math.cos(pitch), sp = Math.sin(pitch);
            double cr = Math.cos(roll), sr = Math.sin(roll);
            return new Rotation(cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr,
                    sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, -sp, cp*sr, cp*cr);
        }
        public Vec3 apply(Vec3 v) {
            return new Vec3(m[0]*v.x+m[1]*v.y+m[2]*v.z,
                    m[3]*v.x+m[4]*v.y+m[5]*v.z, m[6]*v.x+m[7]*v.y+m[8]*v.z);
        }
        public boolean valid() {
            Vec3 x = apply(new Vec3(1,0,0)), y = apply(new Vec3(0,1,0)), z = apply(new Vec3(0,0,1));
            return x.finite() && y.finite() && z.finite()
                    && Math.abs(x.length()-1) < 1e-6 && Math.abs(y.length()-1) < 1e-6
                    && Math.abs(z.length()-1) < 1e-6 && Math.abs(x.dot(y)) < 1e-6
                    && Math.abs(x.dot(z)) < 1e-6 && Math.abs(y.dot(z)) < 1e-6
                    && x.cross(y).dot(z) > 0.999999;
        }
    }

    /** Convex aperture: bottom-center origin, horizontal right axis, face-up axis. */
    public static final class Opening {
        public final Vec3 bottom, right, up, outward;
        private final double[][] polygon;

        public Opening(Vec3 bottom, Vec3 right, Vec3 up, double[][] polygon) {
            this.bottom = bottom;
            this.right = right;
            this.up = up;
            this.outward = right.cross(up);
            this.polygon = new double[polygon.length][];
            for (int i = 0; i < polygon.length; i++) this.polygon[i] = polygon[i].clone();
        }

        /** FIRST Arena TU06 Figure 9-11: pentagonal face, not its 20x14 bounding rectangle. */
        public static Opening bioBuzz(Vec3 bottom, Vec3 right, Vec3 up) {
            return new Opening(bottom, right, up,
                    new double[][]{{-10,0},{10,0},{10,7.61},{0,14},{-10,7.61}});
        }

        public boolean valid() {
            if (!bottom.finite() || !right.finite() || !up.finite()
                    || Math.abs(right.length()-1) > 1e-6 || Math.abs(up.length()-1) > 1e-6
                    || Math.abs(right.dot(up)) > 1e-6 || polygon.length < 3) return false;
            double area = 0;
            for (int i = 0; i < polygon.length; i++) {
                double[] a = polygon[i], b = polygon[(i+1)%polygon.length];
                if (a.length != 2 || b.length != 2 || !Double.isFinite(a[0]) || !Double.isFinite(a[1])
                        || Math.hypot(b[0]-a[0], b[1]-a[1]) < EPS) return false;
                area += a[0]*b[1]-b[0]*a[1];
                for (double[] point : polygon) {
                    if (point.length != 2 || (b[0]-a[0])*(point[1]-a[1])
                            -(b[1]-a[1])*(point[0]-a[0]) < -EPS) return false;
                }
            }
            return area > EPS;
        }
    }

    public static final class Prediction {
        public final boolean fits;
        public final String reason;
        public final double flightSeconds, horizontalInches, clearanceInches;
        public final Vec3 crossing;

        private Prediction(boolean fits, String reason, double seconds, double distance,
                           double clearance, Vec3 crossing) {
            this.fits = fits;
            this.reason = reason;
            this.flightSeconds = seconds;
            this.horizontalInches = distance;
            this.clearanceInches = clearance;
            this.crossing = crossing;
        }
    }

    private static Prediction invalid(String reason) {
        return new Prediction(false, reason, Double.NaN, Double.NaN, Double.NaN, null);
    }

    public static Prediction predict(Vec3 release, Vec3 launchVelocity, Opening opening,
                                     double ballRadius, double errorMargin,
                                     double minimumDistance, double maximumDistance) {
        if (!release.finite() || !launchVelocity.finite() || !opening.valid()
                || !Double.isFinite(ballRadius) || ballRadius <= 0
                || !Double.isFinite(errorMargin) || errorMargin < 0
                || !Double.isFinite(minimumDistance) || !Double.isFinite(maximumDistance)
                || minimumDistance < 0 || maximumDistance <= minimumDistance) return invalid("Invalid shot model input");
        Vec3 gravity = new Vec3(0, 0, -GRAVITY);
        double a = 0.5 * gravity.dot(opening.outward);
        double b = launchVelocity.dot(opening.outward);
        double c = release.minus(opening.bottom).dot(opening.outward);
        if (c <= EPS) return invalid("Shooter is behind the opening plane");
        double[] roots;
        if (Math.abs(a) < EPS) {
            if (Math.abs(b) < EPS) return invalid("Trajectory is parallel to opening");
            roots = new double[]{-c/b};
        } else {
            double discriminant = b*b-4*a*c;
            if (discriminant < 0) return invalid("Trajectory never reaches opening plane");
            double q = -0.5*(b + Math.copySign(Math.sqrt(discriminant), b));
            roots = Math.abs(q) < EPS ? new double[]{-b/(2*a)} : new double[]{q/a, c/q};
        }
        double time = Double.POSITIVE_INFINITY;
        for (double root : roots) {
            if (Double.isFinite(root) && root > EPS && root < time && root <= 3
                    && launchVelocity.plus(gravity.times(root)).dot(opening.outward) < -EPS) time = root;
        }
        if (!Double.isFinite(time)) return invalid("No forward entry into opening");
        Vec3 displacement = launchVelocity.times(time).plus(gravity.times(0.5*time*time));
        Vec3 crossing = release.plus(displacement);
        Vec3 relative = crossing.minus(opening.bottom);
        double x = relative.dot(opening.right), y = relative.dot(opening.up);
        double distance = Math.hypot(displacement.x, displacement.y);
        if (distance < minimumDistance || distance > maximumDistance) {
            return new Prediction(false, "Outside calibrated shooting distances", time, distance, Double.NaN, crossing);
        }
        Vec3 entryVelocity = launchVelocity.plus(gravity.times(time));
        double normalSpeed = Math.abs(entryVelocity.dot(opening.outward));
        double clearance = Double.POSITIVE_INFINITY;
        for (int i = 0; i < opening.polygon.length; i++) {
            double[] p = opening.polygon[i], q = opening.polygon[(i+1)%opening.polygon.length];
            double dx = q[0]-p[0], dy = q[1]-p[1], length = Math.hypot(dx,dy);
            double edgeDistance = (dx*(y-p[1])-dy*(x-p[0]))/length;
            Vec3 inward = opening.right.times(-dy/length).plus(opening.up.times(dx/length));
            // The swept ball needs extra rim clearance when entering the face obliquely.
            double ratio = entryVelocity.dot(inward)/normalSpeed;
            double required = ballRadius*Math.sqrt(1+ratio*ratio) + errorMargin;
            clearance = Math.min(clearance, edgeDistance-required);
        }
        boolean fits = Double.isFinite(clearance) && clearance >= 0;
        return new Prediction(fits, fits ? "Predicted clearance through opening" : "Predicted ball/rim collision",
                time, distance, clearance, crossing);
    }

    /** An observed crossing point, measured in the vertical plane of the launch direction. */
    public static final class FitSample {
        public final double forwardInches, heightAboveReleaseInches;
        public FitSample(double forwardInches, double heightAboveReleaseInches) {
            this.forwardInches = forwardInches;
            this.heightAboveReleaseInches = heightAboveReleaseInches;
        }
    }

    public static final class Fit {
        public final double horizontalSpeed, verticalSpeed, rmsError, maximumError, minDistance, maxDistance;
        private Fit(double horizontal, double vertical, double rms, double maximum, double min, double max) {
            horizontalSpeed = horizontal; verticalSpeed = vertical; rmsError = rms;
            maximumError = maximum; minDistance = min; maxDistance = max;
        }
    }

    /** Fits z = a*d - b*d^2 from measured heights; a scoring yes/no is not a height sample. */
    public static Fit fit(List<FitSample> samples) {
        if (samples.size() < 3) throw new IllegalArgumentException("Need measured crossings at three distances");
        double min = Double.POSITIVE_INFINITY, max = 0, scale = 0;
        int distinct = 0;
        for (int i = 0; i < samples.size(); i++) {
            FitSample s = samples.get(i);
            if (!Double.isFinite(s.forwardInches) || s.forwardInches <= 0
                    || !Double.isFinite(s.heightAboveReleaseInches)) throw new IllegalArgumentException("Invalid crossing sample");
            min = Math.min(min, s.forwardInches); max = Math.max(max, s.forwardInches); scale += s.forwardInches;
            boolean newDistance = true;
            for (int j = 0; j < i; j++) if (Math.abs(s.forwardInches-samples.get(j).forwardInches) < 0.5) newDistance = false;
            if (newDistance) distinct++;
        }
        if (distinct < 3 || max-min < 6) throw new IllegalArgumentException("Distances are too similar to fit curvature");
        scale /= samples.size();
        double aa=0, ab=0, bb=0, ay=0, by=0;
        for (FitSample s : samples) {
            double d=s.forwardInches/scale, d2=d*d;
            aa+=d*d; ab+=d*d2; bb+=d2*d2;
            ay+=d*s.heightAboveReleaseInches; by+=d2*s.heightAboveReleaseInches;
        }
        double determinant=aa*bb-ab*ab;
        if (determinant <= 1e-8*aa*bb) throw new IllegalArgumentException("Ill-conditioned trajectory samples");
        double slope=(ay*bb-by*ab)/determinant/scale;
        double curvature=-(by*aa-ay*ab)/determinant/(scale*scale);
        if (!Double.isFinite(curvature) || curvature <= 1e-10 || !Double.isFinite(slope) || slope <= 0) {
            throw new IllegalArgumentException("Samples do not describe an upward gravity trajectory");
        }
        double horizontal=Math.sqrt(GRAVITY/(2*curvature)), vertical=slope*horizontal;
        if (!Double.isFinite(horizontal) || !Double.isFinite(vertical)) {
            throw new IllegalArgumentException("Non-finite fitted launch speed");
        }
        double squaredError=0, maximumError=0;
        for (FitSample s : samples) {
            double residual=s.heightAboveReleaseInches-(slope*s.forwardInches-curvature*s.forwardInches*s.forwardInches);
            squaredError+=residual*residual; maximumError=Math.max(maximumError,Math.abs(residual));
        }
        return new Fit(horizontal,vertical,Math.sqrt(squaredError/samples.size()),maximumError,min,max);
    }
}
