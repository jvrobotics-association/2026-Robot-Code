package frc.robot;

class ShotData {
  public double vel;
  public double deg;

  ShotData(double vel, double rad) {
    this.vel = vel;
    this.deg = (rad * (180 / Math.PI));
  }
}

public class test {

  public static void main(String[] args) {
    for (int i = 0; i < (15 * 12); i++) {
      double inches = i;
      double vel = math(inches).vel;
      double deg = math(inches).deg;

      System.out.println(
          "Shot from " + inches + " inches: speed: " + vel + " in/s, degrees: " + deg + " degrees");
    }
  }

  public static ShotData math(double inches) {
    double x_dist = inches; // 2d distance in inches
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
    double v0 = Math.sqrt(-g / (2 * a * (Math.cos(theta)) * (Math.cos(theta))));

    return new ShotData(v0, theta);
  }
}
