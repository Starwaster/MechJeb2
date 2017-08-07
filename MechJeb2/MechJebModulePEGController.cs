using System;
using UnityEngine;
using System.Collections.Generic;

namespace MuMech
{
    public class MechJebModulePEGController : ComputerModule
    {
        public MechJebModulePEGController(MechJebCore core) : base(core) { }

        // target burnout radius
        public double rdval;
        // target burnout velocity
        public double vdval;
        // target burnout angle
        public double gamma;
        // target
        public Orbit target { get; set; }

        public double stageLowDVLimit = 20;
        public double terminalGuidanceTime = 10;

        public Vector3d lambda { get; private set; }
        public Vector3d lambdaDot { get; private set; }
        public Vector3d iF { get; private set; }
        public double pitch { get; private set; }
        public double heading { get; private set; }

        private bool initialized;
        private bool converged;
        private bool terminalGuidance;

        private MechJebModuleStageStats stats { get { return core.GetComputerModule<MechJebModuleStageStats>(); } }
        private FuelFlowSimulation.Stats[] vacStats { get { return stats.vacStats; } }
        private FuelFlowSimulation.Stats[] atmoStats { get { return stats.atmoStats; } }

        public override void OnModuleEnabled()
        {
            terminalGuidance = false;
            initialized = false;
            converged = false;
            // FIXME: we need to add liveStats to vacStats and atmoStats from MechJebModuleStageStats so we don't have to force liveSLT here
            stats.liveSLT = true;
            stages = new List<StageInfo>();
        }

        public override void OnModuleDisabled()
        {
        }

        public override void OnFixedUpdate()
        {
            stats.RequestUpdate(this, true);
            if (enabled)
               converge();
        }

        // state for next iteration
        public double tgo;          // time to burnout
        private Vector3d rbias;
        private Vector3d rd;        // burnout position vector
        private Vector3d vgo;       // velocity remaining to be gained
        private Vector3d vprev;     // value of v on prior iteration
        private Vector3d rgrav;
        private Vector3d cser;

        // exposed for terminal guidance
        private double L;
        private double J;

        public List<StageInfo> stages;

        private double lastTime = 0.0;

        private void converge()
        {
            double dt = 0;

            if ( lastTime > 0 )
                dt = vesselState.time - lastTime;
            lastTime = vesselState.time;

            if (terminalGuidance)
            {
                tgo -= dt;
                lambda += lambdaDot * dt;
                iF = ( lambda - lambdaDot * J / L ).normalized; // FIXME: J and L need to be updating continuously?
                Debug.Log("tgo = " + tgo);
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
                }
                if (tgo < terminalGuidanceTime)
                    terminalGuidance = true;
            }
        }

        private void update()
        {
            double gm = mainBody.gravParameter;
            //target = FlightGlobals.GetBodyByName("Moon").orbit;
            // fixed 185x185
            rdval = 6371000 + 185000;
            vdval = Math.Sqrt( gm / rdval);
            gamma = 0;

            // vector position of vessel in ECI
            Vector3d r = vesselState.CoM - vessel.mainBody.position;
            Vector3d v = vesselState.orbitalVelocity;

            Vector3d iy;
            if (target != null)
                iy = -target.SwappedOrbitNormal().normalized;  // SwappedOrbitNormal() also has a minus sign
            else
                // downrrange plane if there's no target set
                iy = -vessel.orbit.SwappedOrbitNormal().normalized; // SwappedOrbitNormal() also has a minus sign

            // value of tgo from previous iteration
            double tgo_prev = 0;

            if (!initialized)
            {
                rbias = new Vector3d(0, 0, 0);
                // rd initialized to rdval-length vector 20 degrees downrange from r
                rd = QuaternionD.AngleAxis(20, -iy) * r.normalized * rdval;
                // vgo initialized to rdval-length vector perpendicular to rd, minus current v
                vgo = Vector3d.Cross(-iy, rd).normalized * vdval - v;
            }
            else
            {
                tgo_prev = tgo;
                Vector3d dvsensed = v - vprev;
                vgo = vgo - dvsensed;
                vprev = v;
            }

            // need accurate stage information before thrust integrals, Li is just dV so we read it from MJ
            UpdateStages();

            // find out how many stages we really need and clean up the Li (dV) and dt of the upper stage
            double vgo_temp_mag = vgo.magnitude;
            int last_stage = 0;
            for(int i = 0; i < stages.Count; i++)
            {
                if ( stages[i].Li > vgo_temp_mag )
                {
                    last_stage = i;
                    stages[i].Li = vgo_temp_mag;
                    stages[i].dt = stages[i].tau * ( 1 - Math.Exp(-stages[i].Li/stages[i].ve) );
                    break;
                }
                else
                {
                    vgo_temp_mag -= stages[i].Li;
                }
            }

            for(int i = last_stage + 1; i < stages.Count; i++)
            {
                stages[i].Li = 0;
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

            // zero out all the upper stages
            for(int i = 0; i <= last_stage; i++)
                stages[i].updateIntegrals();

            // total thrust integrals
            double S, Q, H, P;
            L = J = S = Q = H = P = 0;

            for(int i = 0; i <= last_stage; i++)
            {
                stages[i].Ji += stages[i].Li * stages[i].tgo1;
                stages[i].Si += L * stages[i].dt;
                stages[i].Qi += J * stages[i].dt;
                stages[i].Pi += H * stages[i].dt;

                L += stages[i].Li;
                J += stages[i].Ji;
                S += stages[i].Si;
                Q += stages[i].Qi;
                P += stages[i].Pi;
                H = J * stages[i].tgo - Q;
            }

            // steering
            lambda = vgo.normalized;

            if (!initialized)
                rgrav = -r / ( r.magnitude * r.magnitude * r.magnitude ) * gm / 2;
            else
                rgrav = tgo * tgo / ( tgo_prev * tgo_prev ) * rgrav;

            Vector3d rgo = rd - ( r + v * tgo + rgrav );

            Vector3d iz = Vector3d.Cross(rd, iy).normalized;
            Vector3d rgoxy = rgo - Vector3d.Dot(iz, rgo) * iz;
            double rgoz = ( S - Vector3d.Dot(lambda, rgoxy) ) / Vector3d.Dot(lambda, iz);
            rgo = rgoxy + rgoz*iz + rbias;

            lambdaDot = ( rgo - S*lambda ) / ( Q - S*J/L );
            iF = ( lambda - lambdaDot * J / L ).normalized;

            double phi = Math.Acos(Vector3d.Dot(iF, lambda));
            double phidot = - phi * L / J;

            Vector3d vthrust = ( L - L * phi * phi / 2.0 - J * phi * phidot - H * phidot * phidot / 2.0 ) * lambda
                - ( L * phi + J * phidot ) * lambdaDot.normalized;

            Vector3d rthrust = ( S - S * phi * phi / 2.0 - Q * phi * phidot - P * phidot * phidot / 2.0 ) * lambda
                - ( S * phi + Q * phidot ) * lambdaDot.normalized;

            Vector3d vbias = vgo - vthrust;
            rbias = rgo - rthrust;

            // BLOCK6 - extract pitch and heading
            pitch = 90 - Vector3d.Angle(iF, vesselState.up);
            Vector3d headingDir = iF - Vector3d.Project(iF, vesselState.up);
            heading = UtilMath.Rad2Deg * Math.Atan2(Vector3d.Dot(headingDir, vesselState.east), Vector3d.Dot(headingDir, vesselState.north));

            // BLOCK7 - CSE gravity averaging
            Vector3d rc1 = r - rthrust / 10.0 - vthrust * tgo / 30.0;
            Vector3d vc1 = v + 1.2 * rthrust / tgo - vthrust/10.0;

            cser = new Vector3d(0,0,0);  // FIXME: initialize properly
            Vector3d rc2, vc2;

            CSESimple(rc1, vc1, tgo, out rc2, out vc2, ref cser);

            Vector3d vgrav = vc2 - vc1;
            rgrav = rc2 - rc1 - vc1 * tgo;

            // BLOCK8 - vgo update
            Vector3d rp = r + v * tgo + rgrav + rthrust;
            rp = rp - Vector3d.Dot(rp, iy) * iy;
            rd = rdval * rp.normalized;
            Vector3d ix = rd.normalized;
            iz = Vector3d.Cross(ix, iy);
            Vector3d vd = vdval * ( Math.Sin(gamma) * ix + Math.Cos(gamma) * iz );
            Vector3d vgop = vd - v - vgrav + vbias;
            Vector3d dvgo = 0.0 * (vgop - vgo);  // smoothing - 0.0 for standard ascent, should be tweakable
            vgo = vgop + dvgo;

            // housekeeping
            initialized = true;

            Debug.Log("tgo = " + tgo + " pitch = " + pitch + " heading = " + heading);
            log_stages();
            if ( Math.Abs(( tgo - tgo_prev ) / tgo_prev) < 0.01 )
                converged = true;
        }

        // horribly expensive, very simple CSE calculation
        private void CSESimple(Vector3d r0, Vector3d v0, double t, out Vector3d rf, out Vector3d vf, ref Vector3d last)
        {
            last = last;
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
            }
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
            public double Ji;
            public double Si;
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
                Ji = tau * Li - ve * dt;
                Si = - Ji + dt * Li;
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
                return "kspstage = " + kspStage + "\n" +
                       "a0 = " + a0 + "\n" +
                       "mdot = " + mdot + "\n" +
                       "ve = " + ve + "\n" +
                       "thrust = " + thrust + "\n" +
                       "tau = " + tau + "\n" +
                       "dt = " + dt + "\n" +
                       "Li = " + Li + "\n";
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
