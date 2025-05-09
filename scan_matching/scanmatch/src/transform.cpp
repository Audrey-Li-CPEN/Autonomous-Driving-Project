#include "scan_matching_skeleton/transform.h"
#include <cmath>
#include <Eigen/Geometry>
#include <complex>
#include <iostream>

using namespace std;

void transformPoints(const vector<Point> &points, Transform &t, vector<Point> &transformed_points)
{
  transformed_points.clear();
  for (size_t i = 0; i < points.size(); i++)
  {
    transformed_points.push_back(t.apply(points[i]));
  }
}

int solve_deg2(double a, double b, double c, double &x1, double &x2)
{
  double delta = b * b - 4 * a * c;

  if (delta < 0)
    return 0;

  double inv_2a = 0.5 / a;

  if (delta == 0)
  {
    x1 = -b * inv_2a;
    x2 = x1;
    return 1;
  }

  double sqrt_delta = sqrt(delta);
  x1 = (-b + sqrt_delta) * inv_2a;
  x2 = (-b - sqrt_delta) * inv_2a;
  return 2;
}

/// Reference : Eric W. Weisstein. "Cubic Equation." From MathWorld--A Wolfram Web Resource.
/// http://mathworld.wolfram.com/CubicEquation.html
/// \return Number of real roots found.
int solve_deg3(double a, double b, double c, double d,
               double &x0, double &x1, double &x2)
{
  if (a == 0)
  {
    // Solve second order system
    if (b == 0)
    {
      // Solve first order system
      if (c == 0)
        return 0;

      x0 = -d / c;
      return 1;
    }

    x2 = 0;
    return solve_deg2(b, c, d, x0, x1);
  }

  // Calculate the normalized form x^3 + a2 * x^2 + a1 * x + a0 = 0
  double inv_a = 1. / a;
  double b_a = inv_a * b, b_a2 = b_a * b_a;
  double c_a = inv_a * c;
  double d_a = inv_a * d;

  // Solve the cubic equation
  double Q = (3 * c_a - b_a2) / 9;
  double R = (9 * b_a * c_a - 27 * d_a - 2 * b_a * b_a2) / 54;
  double Q3 = Q * Q * Q;
  double D = Q3 + R * R;
  double b_a_3 = (1. / 3.) * b_a;

  if (Q == 0)
  {
    if (R == 0)
    {
      x0 = x1 = x2 = -b_a_3;
      return 3;
    }
    else
    {
      x0 = pow(2 * R, 1 / 3.0) - b_a_3;
      return 1;
    }
  }

  if (D <= 0)
  {
    // Three real roots
    double theta = acos(R / sqrt(-Q3));
    double sqrt_Q = sqrt(-Q);
    x0 = 2 * sqrt_Q * cos(theta / 3.0) - b_a_3;
    x1 = 2 * sqrt_Q * cos((theta + 2 * 3.1415) / 3.0) - b_a_3;
    x2 = 2 * sqrt_Q * cos((theta + 4 * 3.1415) / 3.0) - b_a_3;

    return 3;
  }

  // D > 0, only one real root
  double AD = pow(fabs(R) + sqrt(D), 1.0 / 3.0) * (R > 0 ? 1 : (R < 0 ? -1 : 0));
  double BD = (AD == 0) ? 0 : -Q / AD;

  // Calculate the only real root
  x0 = AD + BD - b_a_3;

  return 1;
}

/// Reference : Eric W. Weisstein. "Quartic Equation." From MathWorld--A Wolfram Web Resource.
/// http://mathworld.wolfram.com/QuarticEquation.html
/// \return Number of real roots found.
float solve_deg4(double a, double b, double c, double d, double e)
{
  double x0 = 0, x1 = 0, x2 = 0, x3 = 0;
  if (a == 0)
  {
    x3 = 0;
    return solve_deg3(b, c, d, e, x0, x1, x2);
  }

  // Normalize coefficients
  double inv_a = 1. / a;
  b *= inv_a;
  c *= inv_a;
  d *= inv_a;
  e *= inv_a;
  double b2 = b * b, bc = b * c, b3 = b2 * b;

  // Solve resultant cubic
  double r0, r1, r2;
  int n = solve_deg3(1, -c, d * b - 4 * e, 4 * c * e - d * d - b2 * e, r0, r1, r2);
  if (n == 0)
    return 0;

  // Calculate R^2
  double R2 = 0.25 * b2 - c + r0, R;
  if (R2 < 0)
    return 0;

  R = sqrt(R2);
  double inv_R = 1. / R;

  int nb_real_roots = 0;

  // Calculate D^2 and E^2
  double D2, E2;
  if (R < 10E-12)
  {
    double temp = r0 * r0 - 4 * e;
    if (temp < 0)
      D2 = E2 = -1;
    else
    {
      double sqrt_temp = sqrt(temp);
      D2 = 0.75 * b2 - 2 * c + 2 * sqrt_temp;
      E2 = D2 - 4 * sqrt_temp;
    }
  }
  else
  {
    double u = 0.75 * b2 - 2 * c - R2,
           v = 0.25 * inv_R * (4 * bc - 8 * d - b3);
    D2 = u + v;
    E2 = u - v;
  }

  double b_4 = 0.25 * b, R_2 = 0.5 * R;
  if (D2 >= 0)
  {
    double D = sqrt(D2);
    nb_real_roots = 2;
    double D_2 = 0.5 * D;
    x0 = R_2 + D_2 - b_4;
    x1 = x0 - D;
  }

  // Calculate E^2
  if (E2 >= 0)
  {
    double E = sqrt(E2);
    double E_2 = 0.5 * E;
    if (nb_real_roots == 0)
    {
      x0 = -R_2 + E_2 - b_4;
      x1 = x0 - E;
      nb_real_roots = 2;
    }
    else
    {
      x2 = -R_2 + E_2 - b_4;
      x3 = x2 - E;
      nb_real_roots = 4;
    }
  }

  float lambda = x0 > x1 ? x0 : x1;
  lambda = lambda > x2 ? lambda : x2;
  lambda = lambda > x3 ? lambda : x3;
  return lambda;
}

// returns the largest real root to ax^3 + bx^2 + cx + d = 0
complex<float> get_cubic_root(float a, float b, float c, float d)
{
  //std::cout<< "a= " << a<< ";  b= " << b<< ";  c= " << c<< ";  d= " << d<<";"<<std::endl;
  // Reduce to depressed cubic
  float p = c / a - b * b / (3 * a * a);
  float q = 2 * b * b * b / (27 * a * a * a) + d / a - b * c / (3 * a * a);

  // std::cout<<"p = "<<p<<";"<<std::endl;
  // std::cout<<"q = "<<q<<";"<<std::endl;

  complex<float> xi(-.5, sqrt(3) / 2);

  complex<float> inside = sqrt(q * q / 4 + p * p * p / 27);

  complex<float> root;

  for (float k = 0; k < 3; ++k)
  {
    // get root for 3 possible values of k
    root = -b / (3 * a) + pow(xi, k) * pow(-q / 2.f + inside, 1.f / 3.f) + pow(xi, 2.f * k) * pow(-q / 2.f - inside, 1.f / 3.f);
    //std::cout<<"RootTemp: "<< root<<std::endl;
    if (root.imag() != 0)
    {
      return root;
    }
  }

  return root;
}

// returns the largest real root to ax^4 + bx^3 + cx^2 + dx + e = 0
float greatest_real_root(float a, float b, float c, float d, float e)
{
  // Written with inspiration from: https://en.wikipedia.org/wiki/Quartic_function#General_formula_for_roots
  //std::cout<< "a= " << a<< ";  b= " << b<< ";  c= " << c<< ";  d= " << d<< ";  e= " << e<<";"<<std::endl;

  // Reduce to depressed Quadratic
  float p = (8 * a * c - 3 * b * b) / (8 * a * a);
  float q = (b * b * b - 4 * a * b * c + 8 * a * a * d) / (8 * a * a * a);
  float r = (-3 * b * b * b * b + 256 * a * a * a * e - 64 * a * a * b * d + 16 * a * b * b * c) / (256 * a * a * a * a);

  // std::cout<<"p = "<<p<<";"<<std::endl;
  // std::cout<<"q = "<<q<<";"<<std::endl;
  // std::cout<<"r = "<<r<<";"<<std::endl;

  // Ferrari's Solution for Quartics: 8m^3 + 8pm^2 + (2p^2-8r)m - q^2 = 0
  complex<float> m = get_cubic_root(8, 8 * p, 2 * p * p - 8 * r, -q * q);
  complex<float> root1 = -b / (4 * a) + (sqrt(2.f * m) + sqrt(-(2 * p + 2.f * m + sqrt(2.f) * q / sqrt(m)))) / 2.f;
  complex<float> root2 = -b / (4 * a) + (sqrt(2.f * m) - sqrt(-(2 * p + 2.f * m + sqrt(2.f) * q / sqrt(m)))) / 2.f;
  complex<float> root3 = -b / (4 * a) + (-sqrt(2.f * m) + sqrt(-(2 * p + 2.f * m - sqrt(2.f) * q / sqrt(m)))) / 2.f;
  complex<float> root4 = -b / (4 * a) + (-sqrt(2.f * m) - sqrt(-(2 * p + 2.f * m - sqrt(2.f) * q / sqrt(m)))) / 2.f;

  vector<complex<float>> roots{root1, root2, root3, root4};

  float max_real_root = 0.f;

  for (complex<float> root : roots)
  {
    if (root.imag() == 0)
    {
      max_real_root = max(max_real_root, root.real());
    }
  }

  return max_real_root;
}

void updateTransform(vector<Correspondence>& corresponds, Transform& curr_trans)
{
    if (corresponds.size() < 3) {
        return;  // Not enough correspondences to compute a reliable transform
    }

    Eigen::Matrix3f M = Eigen::Matrix3f::Zero();
    Eigen::Vector3f g = Eigen::Vector3f::Zero();

    for (const auto& c : corresponds) {
        Eigen::Vector2f p(c.p->getX(), c.p->getY());
        Eigen::Vector2f q = c.getPiVec();
        Eigen::Vector2f n = c.getNormalNorm();

        Eigen::Matrix2f C;
        C << 0, -1,
             1,  0;
        Eigen::Vector2f Cp = C * p;

        Eigen::Matrix3f M_i;
        M_i << n(0), n(1), n.dot(Cp),
               n(1), -n(0), -n.dot(p),
               0, 0, n.dot(n);

        M += M_i;
        g += M_i.leftCols<2>() * (q - p);
    }

    // Print M and g
    std::cout << "M:\n" << M << std::endl;
    std::cout << "g:\n" << g << std::endl;

    // Solve for the transformation parameters
    Eigen::Vector3f delta = M.colPivHouseholderQr().solve(g);

    float a = delta(0);
    float b = delta(1);
    float c = delta(2);
    float x = curr_trans.x_disp;
    float y = curr_trans.y_disp;
    float z = curr_trans.theta_rot;

    std::cout << "a: " << a << ", b: " << b << ", c: " << c << std::endl;
    std::cout << "x: " << x << ", y: " << y << ", z: " << z << std::endl;

    // Update the transformation
    curr_trans.x_disp += delta(0);
    curr_trans.y_disp += delta(1);
    curr_trans.theta_rot += delta(2);

    std::cout << "Updated x: " << curr_trans.x_disp << ", y: " << curr_trans.y_disp << ", z: " << curr_trans.theta_rot << std::endl;
}
