using System;
using UnityEngine;

namespace MuMech {
    public class RKF45 {
        public delegate void Function(int n, double t, double[] x, ref double[] dx);

        /* in-place addition time optional factor */
        private static void rk_add(ref double[] dest, double[] k, double f = 1.0) {
            for(int i = 0; i < dest.Length; i++)
                dest[i] += f * k[i];
        }

        /* this does one rkf45 round, it may not be the step that we take */
        private static void rkf45_atom(int n, Function f, double h, double t, double[] x, double[] w, ref double e) {
            double[] k1 = new double[n];
            double[] k2 = new double[n];
            double[] k3 = new double[n];
            double[] k4 = new double[n];
            double[] k5 = new double[n];
            double[] k6 = new double[n];
            double[] a  = new double[n];

            for(int i = 0; i < n; i++)  // k1
                f(n, t, x, ref k1);

            for(int i = 0; i < n; i++) {  // k2
                Array.Copy(x, 0, a, 0, n);
                rk_add(ref a, k1, h * 1.0 / 4.0);
                f(n, t + h / 4.0, a, ref k2);
            }

            for(int i = 0; i < n; i++) {   // k3
                Array.Copy(x, 0, a, 0, n);
                rk_add(ref a, k1, h * 3.0 / 32.0);
                rk_add(ref a, k2, h * 9.0 / 32.0);
                f(n, t + 3.0 * h / 8.0, a, ref k3);
            }

            for(int i = 0; i < n; i++) {  // k4
                Array.Copy(x, 0, a, 0, n);
                rk_add(ref a, k1, h * 1932.0 / 2197.0);
                rk_add(ref a, k2, h * -7200.0 / 2197.0);
                rk_add(ref a, k3, h * 7296.0 / 2197.0);
                f(n, t + 12.0 * h / 13.0, a, ref k4);
            }

            for(int i = 0; i < n; i++) {  // k5
                Array.Copy(x, 0, a, 0, n);
                rk_add(ref a, k1, h * 439.0 / 216.0);
                rk_add(ref a, k2, h * -8.0);
                rk_add(ref a, k3, h * 3680.0 / 513.0);
                rk_add(ref a, k4, h * -845.0 / 4104.0 );
                f(n, t + h, a, ref k5);
            }

            for(int i = 0; i < n; i++) {  // k6
                Array.Copy(x, 0, a, 0, n);
                rk_add(ref a, k1, h * -8.0 / 27.0);
                rk_add(ref a, k2, h * 2.0);
                rk_add(ref a, k3, h * -3544.0 / 2565.0);
                rk_add(ref a, k4, h * 1859.0 / 4104.0);
                rk_add(ref a, k5, h * -11.0 / 40.0);
                f(n, t + h / 2.0, a, ref k6);
            }

            // 4th order estimate
            for(int i = 0; i < n; i++)
                w[i] = 0;

            rk_add(ref w, k1, 25.0 / 216.0);
            rk_add(ref w, k3, 1408.0 / 2565.0);
            rk_add(ref w, k4, 2197.0 / 4104.0);
            rk_add(ref w, k5, -1.0/5.0 );

            for(int i = 0; i < n; i++)
                w[i] *= h;

            // error
            for(int i = 0; i < n; i++)
                a[i] = 0;

            rk_add(ref a, k1, 1.0 / 360.0);
            rk_add(ref a, k3, -128.0 / 4275.0);
            rk_add(ref a, k4, -2197.0 / 75240.0);
            rk_add(ref a, k5, 1.0 / 50.0);
            rk_add(ref a, k6, 2.0 / 55.0);

            for(int i = 0; i < n; i++)
                a[i] *= h;

            // add errors in quadrature (FIXME: how to do this properly?)
            e = 0;
            for(int i = 0; i < n; i++)
                e += a[i] * a[i];

            e = Math.Sqrt(e);
        }

        /* this takes one rkf45 step */
        private static void rkf45_step(int n, Function f, ref double h, ref double t, double t_end, ref double[] x, double T) {
            double[] w = new double[n];
            double e = 0.0;

            while(true) {
                if ( t + h > t_end )
                    h = t_end - t;
                rkf45_atom(n, f, h, t, x, w, ref e);
                if ( e > T ) {
                    // error exceeds tolerance
                    h = .9 * h * Math.Pow( T / e, 0.2 );
                } else {
                    // error <= tolerance
                    rk_add(ref x, w);  // x += w
                    t += h;
                    h = .9 * h * Math.Pow( T / e, 0.25 );
                    break;
                }
            }
        }

        /* this integrates to t_end */
        private static void rkf45(int n, Function f, double h, double t, double t_end, double[] xi, ref double[] xf, double T, int maxI) {
            Array.Copy(xi, 0, xf, 0, n); // xf = xi
            int i = 0;

            while (t < t_end) {
                rkf45_step(n, f, ref h, ref t, t_end, ref xf, T);
                var r = new Vector3d(xf[1], xf[2], xf[3]);
                var v = new Vector3d(xf[4], xf[5], xf[6]);
                Debug.Log(t + ": " + xf[0] + " " + r + "(" + r.magnitude + ") " + v + "(" + v.magnitude + ")" );
                i++;
                if (i >= maxI)
                    return;  // FIXME: should probably throw
            }
        }

        public class CentralForceThrust {

            /* 1. setup thrust + isp/mdot vehicle stage parameters, and mu for the central force */
            public double _thrust;
            public double thrust { get { return _thrust; } set { _thrust = value; _update_mdot(); } }
            public double _isp;
            public double isp { get { return _isp; } set { _isp = value; _update_mdot(); } }
            public const double g0 = 9.80665;
            public double mdot;
            public double mu;
            private void _update_mdot() { if (isp != 0) { mdot = _thrust / (g0 * _isp ); } }

            /* 2. setup m, r, v, lambda, lambdaDot, t initial conditions */
            public double m;
            public Vector3d r;
            public Vector3d v;
            public Vector3d lambda;
            public Vector3d lambdaDot;
            public double t;

            /* 3. turn the crank */
            public void integrate(double t_end, double T, int maxI) {
                double[] xi = new double[] {
                    m,
                    r.x, r.y, r.z,
                    v.x, v.y, v.z,
                    lambda.x, lambda.y, lambda.z,
                    lambdaDot.x, lambdaDot.y, lambdaDot.z,
                };
                double[] xf = new double[13];
                rkf45(13, EQMotion, 1.0, t, t_end, xi, ref xf, T, maxI);
                m = xf[0];
                r = new Vector3d(xf[1], xf[2], xf[3]);
                v = new Vector3d(xf[4], xf[5], xf[6]);
                lambda = new Vector3d(xf[7], xf[8], xf[9]);
                lambdaDot = new Vector3d(xf[10], xf[11], xf[12]);
                t = t_end;
            }

            public void EQMotion(int n, double t, double[] x, ref double[] dx) {
                // r-dot = v
                dx[1] = x[4];
                dx[2] = x[5];
                dx[3] = x[6];

                // u = unit(-s)
                double[] u = new double[3];
                double p = Math.Sqrt( x[7] * x[7] + x[8] * x[8] + x[9] * x[9] );
                u[0] = - x[7] / p;
                u[1] = - x[8] / p;
                u[2] = - x[9] / p;

                double r  = Math.Sqrt( x[1] * x[1] + x[2] * x[2] + x[3] * x[3] );
                double r3 = r * r * r;
                double r5 = r3 * r * r;
                double Fm = thrust / x[0];

                // v-dot = g(r) = - mu / r^3 * rvec = - mu / r^2 * rhat
                dx[4] = - mu * x[1] / r3 + Fm * u[0];
                dx[5] = - mu * x[2] / r3 + Fm * u[1];
                dx[6] = - mu * x[3] / r3 + Fm * u[2];

                // s-dot = - q
                dx[7] = - x[10];
                dx[8] = - x[11];
                dx[9] = - x[12];

                // q-dot = -r ( 3 mu r^-5 r dot s ) + s ( mu r^-3 )
                double qr = - 3.0 * mu * ( x[1] * x[7] + x[2] * x[8] + x[3] * x[9] ) / r5;
                double qs = mu / r3;
                dx[10] = qr * x[1] + qs * x[7];
                dx[11] = qr * x[2] + qs * x[8];
                dx[12] = qr * x[3] + qs * x[9];

                dx[0] = - mdot;
            }

            public void test() {
                isp    = 316;               // LR-91
                thrust = 232.7 * 1000;      // LR-91 232.7 kN
                m      = 23.74 * 1000;      // 23.74t
                mu     = 3.9860044189e+14;  // Earth
                double r0 = 6.371e+6 + 0.185e+6; // 185 km
                double r1 = 6.371e+6 + 1.000e+6; // 1000 km
                double a  = ( r0 + r1 ) / 2.0;
                r      = new Vector3d( r0, 0, 0 );
                v      = new Vector3d( 0, Math.Sqrt(mu/r0), 0);    // circular
                lambda = -v;
                lambdaDot = Vector3d.zero;
                t      = 0;

                // impulsive hohmann transfer
                double deltav = Math.Sqrt(mu/r0)*(Math.Sqrt(2.0 * r1/(r0+r1)) - 1.0);
                double tburn = ( m * g0 * isp / thrust ) * ( 1 - Math.Exp( - deltav / (g0 * isp ) ) );

                integrate(tburn, 1e-5, 100);

                double tcoast = 2.0 * Math.PI * Math.Sqrt( a * a * a / mu );
                thrust = 0.0;

                integrate(tcoast, 1e-5, 100);
            }
        }
    }
}
