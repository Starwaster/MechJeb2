﻿using System;
using UnityEngine;
using System.Collections.Generic;

/*
 * PEG algorithm references:
 *
 * - https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19760024151.pdf (Langston1976)
 * - https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19740004402.pdf (Brand1973)
 * - https://ntrs.nasa.gov/search.jsp?R=19790048206 (Mchenry1979)
 * - https://arc.aiaa.org/doi/abs/10.2514/6.1977-1051 (Jaggers1977)
 * - https://ntrs.nasa.gov/search.jsp?R=19760020204 (Jaggers1976)
 * - https://ntrs.nasa.gov/search.jsp?R=19740024190 (Jaggers1974)
 * - i've found the Jaggers thrust integrals aren't stable
 *
 * For future inspiration:
 *
 * - https://arc.aiaa.org/doi/abs/10.2514/6.2012-4843 (Ping Lu's updated PEG)
 *
 */

/*
 *  Higher Priority / Nearer Term TODO list:
 *
 *  - better thrust integrals or just rkf45
 *  - manual entry of coast phase (probably based on kerbal-stage rather than final-stage since final-stage may change if we e.g. eat into TLI)
 *
 *  Medium Priority / Medium Term TODO list:
 *
 *  - linear terminal velocity constraints and engine throttling?  (landings and rendezvous?)
 *  - injection into orbits at other than the periapsis
 *  - matching planes with contract orbits
 *  - launch to rendevous with space-stations (engine throttling?)
 *  - Lambert-driven end conditions
 *  - direct ascent to Lunar intercept
 *  - J^2 fixes for Principia
 *
 *  Wishlist for PEG Nirvana:
 *
 *  - throttling down core engine asymmetrically until booster sep (Delta IV Heavy)
 *  - constant accelleration phase through throttle down (space shuttle style g-limiting)
 *  - timed stage-and-a-half booster separation (Atlas I/II)
 *  - PEG for landing
 *  - launch to free-return around Moon (with and without N-body Principia)
 *  - direct ascent to interplanetary trajectories, 'cuz why the hell not?
 */

namespace MuMech
{
    public enum PegStatus { ENABLED, INITIALIZING, INITIALIZED, CONVERGED, TERMINAL, FINISHED, FAILED };

    public class MechJebModulePEGController : ComputerModule
    {
        public MechJebModulePEGController(MechJebCore core) : base(core) { }

        [Persistent(pass = (int)(Pass.Type | Pass.Global))]
        public EditableDouble pegInterval = new EditableDouble(0.01);

        public Vector3d lambda;
        public Vector3d lambdaDot;
        public double t_lambda;
        public Vector3d primer;
        public Vector3d iF { get { return primer.normalized; } }
        public double phi { get { return lambdaDot.magnitude * K; } }
        public double primerMag { get { return primer.magnitude; } }
        public double pitch;
        public double heading;

        public PegStatus status;

        private MechJebModuleStageStats stats { get { return core.GetComputerModule<MechJebModuleStageStats>(); } }
        private FuelFlowSimulation.Stats[] vacStats { get { return stats.vacStats; } }
        private FuelFlowSimulation.Stats[] atmoStats { get { return stats.atmoStats; } }

        public override void OnModuleEnabled()
        {
            status = PegStatus.ENABLED;
            core.AddToPostDrawQueue(DrawCSE);
        }

        public override void OnModuleDisabled()
        {
        }

        public void AssertStart()
        {
            if (status == PegStatus.ENABLED )
                Reset();
        }

        public override void OnFixedUpdate()
        {
            if ( !enabled || status == PegStatus.ENABLED )
                return;

            if ( status == PegStatus.FINISHED )
            {
                Done();
                return;
            }

            // FIXME: we need to add liveStats to vacStats and atmoStats from MechJebModuleStageStats so we don't have to force liveSLT here
            stats.liveSLT = true;
            stats.RequestUpdate(this, true);

            converge();

            update_pitch_and_heading();
        }

        // state for next iteration
        public double tgo;          // time to burnout
        public double tgo_prev;     // time for last full converge
        public Vector3d vgo;        // velocity remaining to be gained
        private Vector3d rd;        // burnout position vector
        private Vector3d rbias;
        private Vector3d rthrust;
        // following for graphing + stats
        private Vector3d vd;
        private Vector3d rp;
        private Vector3d vp;
        double deltaTcoast;

        private double last_PEG;    // this is the last PEG update time
        public double K;

        public List<StageInfo> stages = new List<StageInfo>();

        /*
         * TARGET APIs
         */

        // private
        private IncMode imode;
        private TargetMode tmode;

        // target burnout radius
        private double rdval;
        // target burnout velocity
        private double vdval;
        // target burnout angle
        private double gamma;
        // tangent plane of desired orbit (opposite of the orbit normal)
        public Vector3d iy;
        // unit vector of rd
        private Vector3d ix;
        // unit vector downrange
        private Vector3d iz;
        // inclination target for FREE_LAN
        private double incval;
        // linear terminal velocity targets
        private Orbit target_orbit;

        private enum IncMode { FIXED_LAN, FREE_LAN };
        private enum TargetMode { PERIAPSIS, ORBIT };

        // must be called after one of TargetPe* for now
        public void AscentInit()
        {
            if ( status != PegStatus.CONVERGED )
            {
                // rd initialized to rdval-length vector 20 degrees downrange from r
                rd = QuaternionD.AngleAxis(20, -iy) * vesselState.up * rdval;
                // Dv initialized to rdval-length vector perpendicular to rd, minus current v
                vgo = Vector3d.Cross(-iy, rd).normalized * vdval - vesselState.orbitalVelocity;
            }
        }

        // does its own initialization and is idempotent
        public void TargetNode(ManeuverNode node)
        {
            if ( status != PegStatus.CONVERGED )
            {
                imode = IncMode.FIXED_LAN;
                tmode = TargetMode.ORBIT;
                target_orbit = node.nextPatch;
                vgo = node.GetBurnVector(orbit);
                iy = - target_orbit.SwappedOrbitNormal();
            }
        }

        /* meta state for consumers that means "is iF usable?" (or pitch/heading) */
        public bool isStable()
        {
            return status == PegStatus.CONVERGED || status == PegStatus.TERMINAL;
        }

        /* normal pre-states but not usefully converged */
        public bool isInitializing()
        {
            return status == PegStatus.ENABLED || status == PegStatus.INITIALIZING || status == PegStatus.INITIALIZED;
        }

        public void TargetPeInsertMatchPlane(double PeA, double ApA, Vector3d tangent)
        {
            imode = IncMode.FIXED_LAN;
            iy = -tangent.normalized;
            SetRdvalVdval(PeA, ApA);
        }

        public void TargetPeInsertMatchOrbitPlane(double PeA, double ApA, Orbit o)
        {
            TargetPeInsertMatchPlane(PeA, ApA, o.SwappedOrbitNormal());
        }

        public void TargetPeInsertMatchInc(double PeA, double ApA, double inc)
        {
            imode = IncMode.FREE_LAN;
            if (incval != inc)
            {
                incval = inc;
                SetPlaneFromInclination(inc);  /* updating every time would defeat PEG's corrector */
            }
            SetRdvalVdval(PeA, ApA);
        }

        /* converts PeA + ApA into rdval/vdval for periapsis insertion.
           - handles hyperbolic orbits
           - remaps ApA < PeA onto circular orbits */
        private void SetRdvalVdval(double PeA, double ApA)
        {
            tmode = TargetMode.PERIAPSIS;
            double PeR = mainBody.Radius + PeA;
            double ApR = mainBody.Radius + ApA;

            rdval = PeR;

            double sma = (PeR + ApR) / 2;

            /* remap nonsense ApAs onto circular orbits */
            if ( ApA >= 0 && ApA < PeA )
                sma = PeR;

            vdval = Math.Sqrt( mainBody.gravParameter * ( 2 / PeR - 1 / sma ) );
            gamma = 0;  /* periapsis */
        }

        /* this will produce an inclination plane based strictly off of the current vessel position, it is somewhat far from optimal,
           but is a useful first guess to seed the predictor (not sure if the below-latitude stuff is strictly necessary, but this is
           code i first used to attempt to directly set iy, but which performs poorly) */
        private void SetPlaneFromInclination(double inc)
        {
            double desiredHeading = UtilMath.Deg2Rad * OrbitalManeuverCalculator.HeadingForInclination(inc, vesselState.latitude);
            Vector3d desiredHeadingVector = Math.Sin(desiredHeading) * vesselState.east + Math.Cos(desiredHeading) * vesselState.north;
            Vector3d tangent = Vector3d.Cross(vesselState.orbitalPosition, desiredHeadingVector).normalized;
            if ( Math.Abs(inc) < Math.Abs(vesselState.latitude) )
            {
                if (Vector3.Angle(tangent, Planetarium.up) < 90)
                    tangent = Vector3.RotateTowards(Planetarium.up, tangent, (float)(inc * UtilMath.Deg2Rad), 10.0f);
                else
                    tangent = Vector3.RotateTowards(-Planetarium.up, tangent, (float)(inc * UtilMath.Deg2Rad), 10.0f);
            }
            iy = -tangent;
        }

        private double last_call;   // this is the last call to converge

        private void converge()
        {
            if ( status == PegStatus.FINISHED )
            {
                Done();
                return;
            }

            if ( status == PegStatus.FAILED )
                Reset();

            if ( last_call != 0 )
            {
                tgo -= vesselState.time - last_call;
                vgo -= vessel.acceleration_immediate * TimeWarp.fixedDeltaTime;
            }

            if ( tgo < TimeWarp.fixedDeltaTime && last_call != 0 )
            {
                Done();
                return;
            }

            if ( core.thrust.targetThrottle > 0 )
                last_call = vesselState.time;

            if ( status == PegStatus.TERMINAL )
                return;

            // only do active guidance every pegInterval
            if ( (vesselState.time - last_PEG) < pegInterval && core.thrust.targetThrottle > 0 )
                return;

            // skipping active guidance for last 10 seconds is neccessary due to thrust integral instability
            if ( status == PegStatus.CONVERGED && tgo < 10 )
            {
                status = PegStatus.TERMINAL;
                return;
            }

            bool converged = false;

            try {
                for(int i = 0; i < 20 && !converged && status != PegStatus.FAILED; i++)
                    converged = update();
            }
            catch
            {
                status = PegStatus.FAILED;
            }

            if (!converged)
                status = PegStatus.FAILED;

            if (vgo.magnitude == 0.0)
                status = PegStatus.FAILED;

            if (status != PegStatus.FAILED)
                status = PegStatus.CONVERGED;

            Debug.Log("============= UPDATE DONE =============");
            last_PEG = vesselState.time;
        }

        /* extract pitch and heading off of iF to avoid continuously recomputing on every call */
        private void update_pitch_and_heading()
        {
            double z = vesselState.time - t_lambda;
            // this more complicated implementation from Jaggers seems to offer no practical advantage as far as I can tell
            // double sinTheta = Vector3d.Dot(lambda, vesselState.up);
            // double w = Math.Sqrt( mainBody.gravParameter / ( vesselState.radius * vesselState.radius * vesselState.radius ) );
            // primer = lambda * Math.Cos(w * z) + lambdaDot / w * Math.Sin(w * z)  + 3.0 * w * z / 2.0 * sinTheta * Math.Sin(w * z) * vesselState.up;
            primer = lambda + lambdaDot * z;
            pitch = 90.0 - Vector3d.Angle(iF, vesselState.up);
            Vector3d headingDir = iF - Vector3d.Project(iF, vesselState.up);
            heading = UtilMath.Rad2Deg * Math.Atan2(Vector3d.Dot(headingDir, vesselState.east), Vector3d.Dot(headingDir, vesselState.north));
        }

        private bool update()
        {
            Vector3d r = vesselState.orbitalPosition;
            double rm = vesselState.orbitalPosition.magnitude;
            Vector3d v = vesselState.orbitalVelocity;

            double gm = mainBody.gravParameter;

            if ( status == PegStatus.INITIALIZING )
            {
                rp = r + v * tgo;
                vp = v;
                t_lambda = vesselState.time;
                rbias = Vector3d.zero;
                rthrust = Vector3d.zero;
                vgo = Vector3d.zero;
                tgo = 1;
                corrector();
            }

            if (vgo == Vector3d.zero)
                vgo = v.normalized;

            // debugging clamp
            if (vgo.magnitude > 10000)
                vgo = vgo.normalized * 10000;

            // need accurate stage information before thrust integrals, Li is just dV so we read it from MJ
            UpdateStages();

            // find out how many stages we really need and clean up the Li (dV) and dt of the upper stage
            double vgo_temp_mag = vgo.magnitude;
            int last_stage = 0;
            for(int i = 0; i < stages.Count; i++)
            {
                last_stage = i;
                if ( stages[i].Li > vgo_temp_mag || i == stages.Count - 1 )
                {
                    stages[i].Li = vgo_temp_mag;
                    stages[i].dt = stages[i].tau * ( 1 - Math.Exp(-stages[i].Li/stages[i].ve) );
                    break;
                }
                else
                {
                  vgo_temp_mag -= stages[i].Li;
                }
            }

            // zero out all the upper stages
            for(int i = last_stage + 1; i < stages.Count; i++)
            {
                stages[i].Li = stages[i].Ji = stages[i].Si = 0;
                stages[i].Qi = stages[i].Pi = stages[i].Hi = 0;
                stages[i].dt = 0;
            }

            // compute cumulative tgo's
            tgo = 0;
            for(int i = 0; i <= last_stage; i++)
            {
                stages[i].tgo1 = tgo;
                tgo += stages[i].dt;
                stages[i].tgo = tgo;
            }

            // total thrust integrals
            double J, L, S, Q, H, P;
            L = J = S = Q = H = P = 0.0;

            for(int i = 0; i <= last_stage; i++)
            {
                stages[i].updateIntegrals();
            }

            for(int i = 0; i <= last_stage; i++)
            {
                double HA = stages[i].Ji * stages[i].tgo - stages[i].Qi;
                double SA = stages[i].Si + L * stages[i].dt;
                double QA = stages[i].Qi + J * stages[i].dt;
                double PA = stages[i].Pi + H * stages[i].dt;
                L += stages[i].Li;
                S += SA;
                H += HA;
                J += stages[i].Ji;
                Q += QA;
                P += PA;
            }

            K = J / L;
            double QT = Q - S * K;

            // steering
            lambda = vgo.normalized;

            // rgo from rd without rgrav from jaggers
            double theta1 = tgo * Math.Sqrt( gm / (rm * rm * rm )) / 2.0;
            double rpm = rp.magnitude;
            double theta2 = tgo * Math.Sqrt( gm / (rpm * rpm * rpm )) / 2.0;
            double T1 = Math.Tan(theta1) / theta1;
            double T2 = Math.Tan(theta2) / theta2;
            Vector3d rgo = rd / T2 - r / T1 - tgo * ( v + vd - vgo ) / 2.0 + rbias;

            // from Jaggers 1977, not clear if you still should orthogonolize this (below) or not
            /*
            if ( imode == IncMode.FREE_LAN && tgo > 40 && isStable() )
            {
                Vector3d ip = Vector3d.Cross(lambda, iy).normalized;
                double Q1 = Vector3d.Dot(rgo - Vector3d.Dot(lambda, rgo) * lambda, ip);
                rgo = S * lambda + Q1 * ip;
            }
            */

            // orthogonalize
            rgo = rgo + ( S - Vector3d.Dot(lambda, rgo) ) * lambda;
            if ( QT == 0 )
                lambdaDot = Vector3d.zero;
            else
                // this comes from rgo = S * lambda + QT * lambdaDot (simplified rthrust)
                lambdaDot = ( rgo - S * lambda ) / QT;

            // lambdaDot = Vector3d.zero;

            double ldm = lambdaDot.magnitude;
            if (ldm < 1e-8)
                ldm = 1e-8;

            // linear tangent guidance is only valid over +/- 90
            double phiMax = 45.0 * UtilMath.Deg2Rad;

            if (tmode == TargetMode.ORBIT)
                // FIXME? only clamp xz and not y?
                phiMax = K * 0.35 * Math.Sqrt( gm / ( rm * rm * rm ) );

            if ( lambdaDot.magnitude > phiMax / K )
            {
                ldm = phiMax / K;
                lambdaDot = lambdaDot.normalized * ldm;
                rgo = S * lambda + QT * lambdaDot;
            }

            Debug.Log("ldm = " + ldm + " ldm * K = " + (ldm * K ) );
            t_lambda = vesselState.time + Math.Tan( ldm * K ) / ldm;
            Debug.Log("t_lambda = " + t_lambda);

            Vector3d u = lambda + lambdaDot * (vesselState.time - t_lambda);

            var integrator = new RKF45.CentralForceThrust();

            Debug.Log("u = " + u + " lambdaDot = " + lambdaDot);
            Debug.Log("--- START ---");
            integrator.t = 0;
            integrator.r = r;
            integrator.v = v;
            integrator.lambda = - u;
            integrator.lambdaDot = lambdaDot;
            integrator.mu = gm;

            for(int i = 0; i <= last_stage; i++)
            {
                Debug.Log("---- STAGE " + i + "----");
                integrator.isp = stages[i].isp;
                integrator.thrust = stages[i].thrust;
                integrator.m = stages[i].mass;
                integrator.mdot = stages[i].mdot;
                integrator.integrate(integrator.t + stages[i].dt, 1e-5, 100);
            }

            Debug.Log("---- DONE ----");

            rp = integrator.r;
            vp = integrator.v;

            rthrust = rp - r;
            rbias = rgo - rthrust;
            //rbias = Vector3d.zero;

            // corrector
            Vector3d vmiss = corrector();

            tgo_prev = tgo;

            if ( status == PegStatus.INITIALIZING )
                status = PegStatus.INITIALIZED;

            return Math.Abs(vmiss.magnitude) < 0.01 * Math.Abs(vgo.magnitude);
        }

        private Vector3d corrector()
        {
            rp = rp - Vector3d.Dot(rp, iy) * iy;
            ix = rp.normalized;
            Vector3d iz = Vector3d.Cross(ix, iy);

            if (tmode == TargetMode.ORBIT)
            {
                /* FIXME: very unlikely to work on ascents / below inv rotation threshold */
                Vector3d target_orbit_periapsis = target_orbit.getRelativePositionFromTrueAnomaly(0).xzy;
                double ta = Vector3.Angle(target_orbit_periapsis, rp) * UtilMath.Deg2Rad;
                if ( Vector3d.Dot(Vector3d.Cross(target_orbit_periapsis, rp), -iy) < 0 )
                    ta = -ta;
                Debug.Log("ta = " + ta);
                vd = target_orbit.getOrbitalVelocityAtTrueAnomaly(ta).xzy;
                rd = target_orbit.getRelativePositionFromTrueAnomaly(ta).xzy;
                Vector3d v = vesselState.orbitalVelocity;
                Debug.Log("v = " + v.magnitude + " vp = " + vp.magnitude + " vd = " + vd.magnitude + " vd-vp = " + (vd - vp).magnitude + " vd-v = " + (vd - v).magnitude);
            }
            else
            {
                rd = rdval * ix;
                vd = vdval * ( Math.Sin(gamma) * ix + Math.Cos(gamma) * iz );
            }

            if ( status == PegStatus.CONVERGED && tgo < 40 )
                rd = rp;

            Vector3d vmiss = vp - vd;
            vgo = vgo - 1.0 * vmiss;

            /*
            if ( imode == IncMode.FREE_LAN && tgo > 40 && isStable() )
            {
                // correct iy to fixed inc with free LAN
                double d = Vector3d.Dot( -Planetarium.up, ix );
                double SE = - 0.5 * ( Vector3d.Dot( -Planetarium.up, iy) + Math.Cos(incval * UtilMath.Deg2Rad) ) * Vector3d.Dot( -Planetarium.up, iz ) / (1 - d * d);
                iy = ( iy * Math.Sqrt( 1 - SE * SE ) + SE * iz ).normalized;
            }
            */
            return vmiss;
        }


        List<Vector3d> CSEPoints = new List<Vector3d>();

        private void DrawCSE()
        {
            var r = orbit.getRelativePositionAtUT(vesselState.time).xzy;
            var p = mainBody.position;
            Vector3d vpos = vessel.CoM + (vesselState.orbitalVelocity - Krakensbane.GetFrameVelocity() - vessel.orbit.GetRotFrameVel(vessel.orbit.referenceBody).xzy) * Time.fixedDeltaTime;
            GLUtils.DrawPath(mainBody, new List<Vector3d> { p, p + r }, Color.red, true, false, false);
            GLUtils.DrawPath(mainBody, new List<Vector3d> { vpos, vpos + vgo }, Color.green, true, false, false);
            GLUtils.DrawPath(mainBody, new List<Vector3d> { rd + p, rd + p + ( vd * 100 ) }, Color.green, true, false, false);
            GLUtils.DrawPath(mainBody, new List<Vector3d> { rp + p, rp + p + ( vp * 100 ) }, Color.red, true, false, false);
            GLUtils.DrawPath(mainBody, new List<Vector3d> { vpos, vpos + iF * 100 }, Color.red, true, false, false);
            GLUtils.DrawPath(mainBody, new List<Vector3d> { vpos, vpos + lambda * 100 }, Color.blue, true, false, false);
            GLUtils.DrawPath(mainBody, new List<Vector3d> { vpos, vpos + lambdaDot * 100 }, Color.cyan, true, false, false);
            // GLUtils.DrawPath(mainBody, new List<Vector3d> { vpos, vpos + rthrust }, Color.magenta, true, false, false);
            // GLUtils.DrawPath(mainBody, CSEPoints, Color.red, true, false, false);
        }

        Orbit CSEorbit = new Orbit();

        /* FIXME: this still doesn't quite work due to the inverse rotation problem -- still wiggles at 145km on ascents, particularly polar ones */
        /* am I just missing Krakensbane.GetFrameVelocity() ? */
        private void CSEKSP(Vector3d r0, Vector3d v0, double t, out Vector3d rf, out Vector3d vf)
        {
            Vector3d rot = orbit.GetRotFrameVelAtPos(mainBody, r0.xzy);
            CSEorbit.UpdateFromStateVectors(r0.xzy, v0.xzy + rot, mainBody, vesselState.time);
            CSEorbit.GetOrbitalStateVectorsAtUT(vesselState.time, out rf, out vf);
            CSEorbit.GetOrbitalStateVectorsAtUT(vesselState.time + t, out rf, out vf);

            rot = orbit.GetRotFrameVelAtPos(mainBody, rf);

            rf = rf.xzy;
            vf = (vf - rot).xzy;
        }

        /* stupidly simple, but expensive, and you get graphs */
        private void CSESimple(Vector3d r0, Vector3d v0, double t, out Vector3d rf, out Vector3d vf)
        {
            CSEPoints.Clear();
            int N = Convert.ToInt32( 2.0 * t );
            if (N < 150)
                N = 150;
            vf = v0;
            rf = r0;
            double dt = t/N;
            for(int i = 0; i < N; i++)
            {
                double rfm = rf.magnitude;
                vf = vf - dt * mainBody.gravParameter * rf / (rfm * rfm * rfm );
                rf = rf + dt * vf;
                if (i % 10 == 0) {
                    CSEPoints.Add(rf + mainBody.position);
                }
            }
        }

        private void Done()
        {
            users.Clear();
            core.thrust.targetThrottle = 0.0F;
            status = PegStatus.FINISHED;
        }

        public void Reset()
        {
            System.Diagnostics.StackTrace t = new System.Diagnostics.StackTrace();
            // failed is deliberately not set to false here
            // lambda and lambdaDot are deliberately not cleared here
            if ( imode == IncMode.FREE_LAN )
                SetPlaneFromInclination(incval);
            status = PegStatus.INITIALIZING;
            stages = new List<StageInfo>();
            tgo_prev = 0.0;
            tgo = 1.0;
            last_PEG = 0.0;
            last_call = 0.0;
            rd = Vector3d.zero;
        }

        private int MatchInOldStageList(int i)
        {
            // some paranoia
            if ( i > (vacStats.Length - 1) )
                return -1;

            // it may later match, but zero dV is useless
            if ( vacStats[i].deltaV <= 0 )
                return -1;

            // this is used to copy stages from the oldlist to the newlist and reduce garbage
            for ( int j = 0; j < stages.Count; j++ )
            {
                if ( stages[j].kspStage == i && stages[j].PartsListMatch(vacStats[i].parts) )
                {
                    return j;
                }
            }
            return -1;
        }

        public void SynchStats()
        {
            for (int i = 0; i < stages.Count; i++ )
            {
                // only live atmostats for the bottom stage, which is inaccurate, but this
                // is abusing an algorithm that can't fly properly in the atmosphere anyway
                FuelFlowSimulation.Stats[] mjstats = ( i == 0 ) ? atmoStats : vacStats;

                int k = stages[i].kspStage;
                stages[i].parts = mjstats[k].parts;
                stages[i].isp = mjstats[k].isp;
                stages[i].ve = mjstats[k].isp * 9.80665;
                stages[i].thrust = mjstats[k].startThrust;
                stages[i].dt = mjstats[k].deltaTime;
                stages[i].Li = mjstats[k].deltaV;
                if ( i == 0 )
                    // mechjeb's staging can get confused sometimes
                    stages[i].mass = vesselState.mass;
                else
                    stages[i].mass = mjstats[k].startMass;

                stages[i].a0 = stages[i].thrust / stages[i].mass;
                stages[i].tau = stages[i].ve / stages[i].a0;
                stages[i].mdot = stages[i].thrust / stages[i].ve;
            }
        }

        private void UpdateStages()
        {
            List<StageInfo>newlist = new List<StageInfo>();

            for ( int i = vacStats.Length-1; i >= 0; i-- )
            {
                int j;
                if ( ( j = MatchInOldStageList(i) ) > 0 )
                {
                    newlist.Add(stages[j]);
                    stages.RemoveAt(j);
                    continue;
                }
                if ( vacStats[i].deltaV > 0 )
                {
                    StageInfo stage = new StageInfo();
                    stage.kspStage =  i;
                    newlist.Add(stage);
                }
            }

            stages = newlist;

            SynchStats();
        }

        public class StageInfo
        {
            // vehicle data
            public double mdot;
            public double ve;
            public double isp;
            public double thrust;
            public double mass;
            public double a0;
            public double tau;
            public double dt;

            // integrals
            public double Li;      // delta-V (first integral)
            public double Si;
            public double Hi;
            public double Ji;
            public double Qi;
            public double Pi;

            // cumulative dt
            public double tgo1;    // tgo of i-1th stage (tgo of previous stage / tgo at start of this stage)
            public double tgo;     // tgo at the end of the stage (sum of dt of this stage and all previous)

            // tracking
            public List<Part> parts;
            public int kspStage;

            // does the first pass of updating the integrals
            public void updateIntegrals()
            {
                Si = - Li * ( tau - dt ) + ve * dt;
                Ji = Li * tgo - Si;
                Qi = Si * ( tau + tgo1 ) - ve * dt * dt / 2.0;
                Pi = Qi * ( tau + tgo1 ) - ve * dt * dt / 2.0 * ( dt / 3.0 + tgo1 );
            }

            public bool PartsListMatch(List<Part> other)
            {
                for(int i = 0; i < parts.Count; i++)
                {
                    if ( !other.Contains(parts[i]) )
                        return false;
                }
                for(int i = 0; i < other.Count; i++)
                {
                    if ( !parts.Contains(other[i]) )
                        return false;
                }
                return true;
            }

            public override string ToString()
            {
                return "kspstage = " + kspStage +
                       " a0 = " + a0 +
                       " mdot = " + mdot +
                       " ve = " + ve +
                       " thrust = " + thrust +
                       " mass = " + mass +
                       " tau = " + tau +
                       " tgo1 = " + tgo1 +
                       " dt = " + dt +
                       " tgo = " + tgo +
                       " Li = " + Li +
                       " Si = " + Si +
                       " Ji = " + Ji;
            }
        }

        private void log_stages()
        {
            Debug.Log("num stages = " + stages.Count);
            for ( int i = 0; i < stages.Count; i++ )
            {
                Debug.Log(stages[i]);
            }
        }
    }
}
