package frc.robot;


public class test {
    public static void main(String[] args) {
        double x_dist = 72.0; // 2d distance in inches
        double y_dist = 56.4 - 14; // Z of target minus robot-to-turret transoform Z
        double h = 15.6 + 20.0; // height of funnel plus clearance of funnel 
        double r = 24; // just funnel radius
        double A1 = x_dist * x_dist;
        double B1 = x_dist;
        double D1 = y_dist;
        double A2 = -x_dist * x_dist + (x_dist - r) * (x_dist - r);
        double B2 = -r;
        double D2 = h;
        double Bm = -B2 / B1;
        double A3 = Bm * A1 + A2;
        double D3 = Bm * D1 + D2;
        double a = D3 / A3;
        double b = (D1 - A1 * a) / B1;
        double theta = Math.atan(b);

        double g = 386; // gravity inches/sec^2
        double v0 = Math.sqrt(-g / (2 * a * (Math.cos(theta)) * (Math.cos(theta)) ));

        System.out.println("Theta in radians: " + theta);
        System.out.println("Exit velocity in in/s: " + v0);
    }
}
