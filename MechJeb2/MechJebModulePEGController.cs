using System;
using UnityEngine;
using System.Collections.Generic;

namespace MuMech
{
    public class MechJebModulePEGController : ComputerModule
    {
        public MechJebModulePEGController(MechJebCore core) : base(core) { }

        // NOTE:  While you can end your burn when tgo is zero, this may be incorrect if any of the data in the
        // FuelFlowSimulation are incorrect.  It will often be better to burn until some criteria like orbital
        // angular momentum is reached, which will actually cover up small issues with calculation of thrust,
        // accel, isp, mdot, etc with the engines.

        // target burnout radius
        public double rdval;
        // target burnout velocity
        public double vdval;
        // target burnout angle
        public double gamma;
        // target tangent vector (defines the plane)
        public Vector3d tangent;
        // target
        public Orbit target { get; set; }

        [Persistent(pass = (int)(Pass.Type | Pass.Global))]
        public EditableDouble stageLowDVLimit = new EditableDouble(20);
        [Persistent(pass = (int)(Pass.Type | Pass.Global))]
        public EditableDouble terminalGuidanceTime = new EditableDouble(10);

        public Vector3d lambda { get; private set; }
        public Vector3d lambdaDot { get; private set; }
        public Vector3d iF { get; private set; }
        public double phi2 { get { return lambdaDot.magnitude * JoL; } }
        public double primerMag { get { return ( lambda + lambdaDot * ( vesselState.time - last_PEG - JoL ) ).magnitude; } }
        public double pitch { get; private set; }
        public double heading { get; private set; }

        public bool initialized;
        public bool converged;
        public bool terminalGuidance;
        public bool failed;

        private MechJebModuleStageStats stats { get { return core.GetComputerModule<MechJebModuleStageStats>(); } }
        private FuelFlowSimulation.Stats[] vacStats { get { return stats.vacStats; } }
        private FuelFlowSimulation.Stats[] atmoStats { get { return stats.atmoStats; } }

        private System.Diagnostics.Stopwatch CSEtimer = new System.Diagnostics.Stopwatch();

        public override void OnModuleEnabled()
        {
            Reset();
            //core.AddToPostDrawQueue(DrawCSE);
        }

        public override void OnModuleDisabled()
        {
        }

        public override void OnFixedUpdate()
        {
            // FIXME: we need to add liveStats to vacStats and atmoStats from MechJebModuleStageStats so we don't have to force liveSLT here
            stats.liveSLT = true;
            stats.RequestUpdate(this, true);
            if (enabled)
               converge();
        }

        // state for next iteration
        public double tgo;          // time to burnout
        public Vector3d vgo;        // velocity remaining to be gained
        private Vector3d rbias;
        private Vector3d rd;        // burnout position vector
        private Vector3d vprev;     // value of v on prior iteration
        private Vector3d rgrav;
        private Vector3d rgo;  // FIXME: temp for graphing

        private double last_PEG;    // this is the last PEG update time
        private double JoL;

        public List<StageInfo> stages = new List<StageInfo>();

        private double lastTime = 0.0;  // this is the last time through update()

        /* converts PeA + ApA into rdval/vdval for periapsis insertion */
        public void PeriapsisInsertion(double PeA, double ApA)
        {
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

        public void SetPlaneFromInclination(double inc)
        {
            double desiredHeading = UtilMath.Deg2Rad * OrbitalManeuverCalculator.HeadingForInclination(inc, vesselState.latitude);
            Vector3d desiredHeadingVector = Math.Sin(desiredHeading) * vesselState.east + Math.Cos(desiredHeading) * vesselState.north;
            tangent = Vector3d.Cross(vesselState.orbitalPosition, desiredHeadingVector).normalized;
            if ( Math.Abs(inc) < Math.Abs(vesselState.latitude) )
            {
                if (Vector3.Angle(tangent, Planetarium.up) < 90)
                    tangent = Vector3.RotateTowards(Planetarium.up, tangent, (float)(inc * UtilMath.Deg2Rad), 10.0f);
                else
                    tangent = Vector3.RotateTowards(-Planetarium.up, tangent, (float)(inc * UtilMath.Deg2Rad), 10.0f);
            }
        }

        private void converge()
        {
            double dt = 0;

            if ( lastTime > 0 )
                dt = vesselState.time - lastTime;

            lastTime = vesselState.time;

            if (terminalGuidance)
            {
                tgo -= dt;
                // FIXME: should manually decrement staging info here of last stage
                // UpdateStages();
                // FIXME: should decrement vgo here as well as update() or move to top of this loop
            }
            else
            {
                if (converged)
                {
                    update();
                }
                else
                {
                    for(int i = 0; i < 200 && !converged; i++)
                        update();
                    if (!converged)
                        failed = true;
                }
                if (vgo.magnitude == 0.0)
                {
                    failed = true;
                    converged = false;
                }
                if (converged && !failed && tgo < terminalGuidanceTime)
                    terminalGuidance = true;
            }

            if (!converged || failed)
            {
                converged = false;
                failed = true;
            }

            update_pitch_and_heading();
        }

        /* extract pitch and heading off of iF to avoid continuously recomputing on every call */
        private void update_pitch_and_heading()
        {
            iF = ( lambda + lambdaDot * ( vesselState.time - last_PEG - JoL ) ).normalized;
            pitch = 90 - Vector3d.Angle(iF, vesselState.up);
            Vector3d headingDir = iF - Vector3d.Project(iF, vesselState.up);
            heading = UtilMath.Rad2Deg * Math.Atan2(Vector3d.Dot(headingDir, vesselState.east), Vector3d.Dot(headingDir, vesselState.north));
        }

        private void update()
        {
            failed = false;

            double gm = mainBody.gravParameter;

            Vector3d iy;
            if (target != null)
                iy = -target.SwappedOrbitNormal().normalized;  // SwappedOrbitNormal() also has a minus sign
            else
                // iy direction for PEG is minus the orbit normal in order to make it a left-handed (in KSP) coordinate system
                iy = -tangent;

            // value of tgo from previous iteration
            double tgo_prev = 0;

            if (!initialized)
            {
                CSEtimer.Reset();
                rbias = Vector3d.zero;
                // rd initialized to rdval-length vector 20 degrees downrange from r
                rd = QuaternionD.AngleAxis(20, -iy) * vesselState.up * rdval;
                // vgo initialized to rdval-length vector perpendicular to rd, minus current v
                vgo = Vector3d.Cross(-iy, rd).normalized * vdval - vesselState.orbitalVelocity;
                tgo = 1;
            }
            else
            {
                tgo_prev = tgo;
                Vector3d dvsensed = vesselState.orbitalVelocity - vprev;
                vgo = vgo - dvsensed;
                vprev = vesselState.orbitalVelocity;
            }

            Debug.Log("vgo1 = " + vgo );

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
            L = J = S = Q = H = P = 0;

            for(int i = 0; i <= last_stage; i++)
            {
                stages[i].updateIntegrals();
                stages[i].Hi = stages[i].Ji * stages[i].tgo - stages[i].Qi;
                stages[i].Si += L * stages[i].dt;
                stages[i].Qi += J * stages[i].dt;
                stages[i].Pi += H * stages[i].dt;

                L += stages[i].Li;
                S += stages[i].Si;
                H += stages[i].Hi;
                J += stages[i].Ji;
                Q += stages[i].Qi;
                P += stages[i].Pi;
            }

            JoL = J/L;
            double Qp = Q - S * JoL;

            Debug.Log("vgo2 = " + vgo );
            // steering
            lambda = vgo.normalized;
            Debug.Log("lambda = " + lambda);

            if (!initialized)
            {
                var r = vesselState.radius;
                rgrav = -vesselState.orbitalPosition * Math.Sqrt( gm / ( r * r * r ) ) / 2.0;
            }
            else
            {
                rgrav = tgo * tgo / ( tgo_prev * tgo_prev ) * rgrav;
            }

            /* from https://arc.aiaa.org/doi/abs/10.2514/6.1977-1051, page 570 */
            rgo = rd - ( vesselState.orbitalPosition + vesselState.orbitalVelocity * tgo + rgrav );
            Vector3d iz = Vector3d.Cross(rd, iy).normalized;
            Vector3d rgoxy = rgo - Vector3d.Dot(iz, rgo) * iz;
            double rgoz = ( S - Vector3d.Dot(lambda, rgoxy) ) / Vector3d.Dot(lambda, iz);
            rgo = rgoxy + rgoz*iz + rbias;

            if (Qp != 0)
                lambdaDot = ( rgo - S*lambda ) / Qp;
            else
                lambdaDot = Vector3d.zero;

            /* this is from the PEG-4 paper for clipping phi values */
            double phi2Max = 90.0 * UtilMath.Deg2Rad;

            if ( phi2 > phi2Max )
            {
                double lambdaDotMag = phi2Max / JoL;
                lambdaDot = lambdaDotMag * lambdaDot.normalized;
                rgo = S * lambda + Qp * lambdaDot;
            }

            /* we've now updated lambda, lambdaDot and JoL, so iF(t) just needs last_PEG time */
            last_PEG = vesselState.time;

            update_pitch_and_heading();

            double phi = Math.Acos(Vector3d.Dot(iF, lambda));
            double phidot = - phi * L / J;

            Vector3d vthrust = ( L - L * phi * phi / 2.0 - J * phi * phidot - H * phidot * phidot / 2.0 ) * lambda;

            Vector3d rthrust = ( S - S * phi * phi / 2.0 - Q * phi * phidot - P * phidot * phidot / 2.0 ) * lambda
                - ( S * phi + Q * phidot ) * lambdaDot.normalized;

            /* PEG-4
            Vector3d vthrust = (L - lambdaDot.sqrMagnitude * ( H - J * JoL ) / 2.0 ) * lambda;
            Vector3d rthrust = (S - lambdaDot.sqrMagnitude * ( P - JoL * (Q + Qp) ) / 2.0 ) * lambda + Qp * lambdaDot;
            */

            // going faster than the speed of light or doing burns the length of the Oort cloud are not supported...
            if (!vthrust.magnitude.IsFinite() || !rthrust.magnitude.IsFinite() || (vthrust.magnitude > 1E10) || (rthrust.magnitude > 1E16))
            {
                Fail();
                return;
            }

            rbias = rgo - rthrust;

            // BLOCK7 - CSE gravity averaging

            Vector3d rc1 = vesselState.orbitalPosition - rthrust / 10.0 - vthrust * tgo / 30.0;
            Vector3d vc1 = vesselState.orbitalVelocity + 1.2 * rthrust / tgo - vthrust/10.0;

            if (!vc1.magnitude.IsFinite() || !rc1.magnitude.IsFinite() || !tgo.IsFinite())
            {
                Fail();
                return;
            }

            Vector3d rc2, vc2;

            CSEtimer.Start();
            CSEKSP(rc1, vc1, tgo, out rc2, out vc2);
            //ConicStateUtils.CSE(mainBody.gravParameter, rc1, vc1, tgo, out rc2, out vc2);
            //CSESimple(rc1, vc1, tgo, out rc2, out vc2);
            CSEtimer.Stop();

            Debug.Log("CSETimer = " + CSEtimer.Elapsed);

            Vector3d vgrav = vc2 - vc1;
            rgrav = rc2 - rc1 - vc1 * tgo;

            Vector3d rp = vesselState.orbitalPosition + vesselState.orbitalVelocity * tgo + rgrav + rthrust;
            Vector3d vp = vesselState.orbitalVelocity + vthrust + vgrav;

            // corrector

            rp = rp - Vector3d.Dot(rp, iy) * iy;
            rd = rdval * rp.normalized;
            Vector3d ix = rd.normalized;
            iz = Vector3d.Cross(ix, iy);
            Vector3d vd = vdval * ( Math.Sin(gamma) * ix + Math.Cos(gamma) * iz );
            Vector3d vmiss = vp - vd;
            vgo = vgo - 1.0 * vmiss;

            Debug.Log("lambdaDot = " + lambdaDot.magnitude + " Qp = " + Qp + " rgo = " + rgo.magnitude);

            // housekeeping
            initialized = true;

            if ( Math.Abs(vmiss.magnitude) < 0.001 * Math.Abs(vgo.magnitude) )
                converged = true;
        }

        List<Vector3d> CSEPoints = new List<Vector3d>();

        private void DrawCSE()
        {
            var p = mainBody.position;
            GLUtils.DrawPath(mainBody, new List<Vector3d> { Vector3d.zero, vgo }, Color.green, true, false, false);
            GLUtils.DrawPath(mainBody, new List<Vector3d> { Vector3d.zero, iF * 100 }, Color.red, true, false, false);
            GLUtils.DrawPath(mainBody, new List<Vector3d> { Vector3d.zero, lambda * 101 }, Color.blue, true, false, false);
            GLUtils.DrawPath(mainBody, new List<Vector3d> { Vector3d.zero, lambdaDot * 100 }, Color.cyan, true, false, false);
            GLUtils.DrawPath(mainBody, new List<Vector3d> { Vector3d.zero, rgo }, Color.magenta, true, false, false);
            GLUtils.DrawPath(mainBody, CSEPoints, Color.red, true, false, false);
        }

        Orbit CSEorbit = new Orbit();

        private void CSEKSP(Vector3d r0, Vector3d v0, double t, out Vector3d rf, out Vector3d vf)
        {
            /* apparently orbit.GetRotFrameVelAtPos is "smart" and does not need this altitude check? bill says 'TEST!' (yep it is smart) */
            /*
            if ( vessel.altitude < mainBody.inverseRotThresholdAltitude )
                rot = mainBody.getRFrmVel(r0 + mainBody.position);
            else
                rot = new Vector3d(0, 0, 0);
                */

            /* FIXME:
               - use orbit.GetRotFrameVelAtPos instead of mainBody.getRFrmVel
               - vectors for GetRotFrameVelAtPos are swizzled
               - UpdateFromStateVectors needs the velocity fixed for the frame rotation
               - GetOrbitalStateVectorsAtUT does *NOT* and should be correct below + above the threshold
               */

            Debug.Log("inputs      r0 = " + r0 + " v0 = " + v0);
            Vector3d rot = orbit.GetRotFrameVelAtPos(mainBody, r0.xzy);
            Debug.Log("rot = " + rot);
            Debug.Log("rot.xzy = " + rot.xzy);
            Debug.Log("( v0.xzy + rot ).xzy = " + ( v0.xzy + rot ).xzy );
            CSEorbit.UpdateFromStateVectors(r0.xzy, v0.xzy + rot, mainBody, vesselState.time);
            CSEorbit.GetOrbitalStateVectorsAtUT(vesselState.time, out rf, out vf);
            Debug.Log("from GOSVAU r0 = " + rf.xzy + " v0 = " + vf.xzy);
            CSEorbit.GetOrbitalStateVectorsAtUT(vesselState.time + t, out rf, out vf);

            rot = orbit.GetRotFrameVelAtPos(mainBody, rf);

            rf = rf.xzy;
            vf = (vf - rot).xzy;
            Debug.Log("rf = " + rf + " vf = " + vf);
        }

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

        private void Fail()
        {
            failed = true;
            Reset();
        }

        public void Reset()
        {
            // failed is deliberately not set to false here
            // lambda and lambdaDot are deliberately not cleared here
            terminalGuidance = false;
            initialized = false;
            converged = false;
            stages = new List<StageInfo>();
            tgo = 0.0;
            rbias = Vector3d.zero;
            rd = Vector3d.zero;
            vgo = Vector3d.zero;
            vprev = Vector3d.zero;
            rgrav = Vector3d.zero;
        }

        private int MatchInOldStageList(int i)
        {
            // some paranoia
            if ( i > (vacStats.Length - 1) )
                return -1;

            // it may later match, but zero dV is useless
            if ( vacStats[i].deltaV <= 0 )
                return -1;

            // mostly this is useful for tracking the final stage when its dV falls below the stageLowDVLimit
            // and for copying old stages to the new stagelist and thereby reducing garbage
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
                stages[i].ve = mjstats[k].isp * 9.80665;
                stages[i].a0 = mjstats[k].startThrust / mjstats[k].startMass;
                stages[i].thrust = mjstats[k].startThrust;
                stages[i].dt = mjstats[k].deltaTime;
                stages[i].Li = mjstats[k].deltaV;
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
                if ( ( j = MatchInOldStageList(i) ) > 0 ) {
                    newlist.Add(stages[j]);
                    stages.RemoveAt(j);
                    continue;
                }
                if ( vacStats[i].deltaV > stageLowDVLimit )
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
            public double thrust;
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
                Ji = - Si + Li * tgo;
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
                String ret = "kspstage = " + kspStage + "\n" +
                       "a0 = " + a0 + "\n" +
                       "mdot = " + mdot + "\n" +
                       "ve = " + ve + "\n" +
                       "thrust = " + thrust + "\n" +
                       "tau = " + tau + "\n" +
                       "dt = " + dt + "\n" +
                       "Li = " + Li + "\n" +
                       "Parts = ";

                for(int i=0; i < parts.Count; i++)
                {
                    ret += parts[i];
                }
                ret += "\n";
                return ret;
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
