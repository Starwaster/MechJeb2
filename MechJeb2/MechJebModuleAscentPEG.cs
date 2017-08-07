using System;
using UnityEngine;
using System.Collections.Generic;

/*
 * Space-Shuttle PEG launches for RSS/RO
 */

namespace MuMech
{
    public class MechJebModuleAscentPEG : MechJebModuleAscentBase
    {
        public MechJebModuleAscentPEG(MechJebCore core) : base(core) { }

        /* default pitch program here works decently at SLT of about 1.4 */
        [Persistent(pass = (int)(Pass.Type | Pass.Global))]
        public EditableDoubleMult pitchStartTime = new EditableDoubleMult(10);
        [Persistent(pass = (int)(Pass.Type | Pass.Global))]
        public EditableDoubleMult pitchRate = new EditableDoubleMult(0.75);
        [Persistent(pass = (int)(Pass.Type | Pass.Global))]
        public EditableDoubleMult pitchEndTime = new EditableDoubleMult(55);
        [Persistent(pass = (int)(Pass.Global))]
        public EditableDoubleMult desiredApoapsis = new EditableDoubleMult(0, 1000);
        [Persistent(pass = (int)(Pass.Type | Pass.Global))]
        public EditableDoubleMult terminalGuidanceSecs = new EditableDoubleMult(10);
        [Persistent(pass = (int)(Pass.Type | Pass.Global))]

        /* this deliberately does not persist, it is for emergencies only */
        public EditableDoubleMult pitchBias = new EditableDoubleMult(0);

        private MechJebModulePEGController peg { get { return core.GetComputerModule<MechJebModulePEGController>(); } }

        public override void OnModuleEnabled()
        {
            mode = AscentMode.VERTICAL_ASCENT;
        }

        public override void OnModuleDisabled()
        {
            peg.enabled = false;
        }

        private enum AscentMode { VERTICAL_ASCENT, INITIATE_TURN, GRAVITY_TURN, EXIT };
        private AscentMode mode;

        public override bool DriveAscent(FlightCtrlState s)
        {
            peg.enabled = true;
            setTarget();
            switch (mode)
            {
                case AscentMode.VERTICAL_ASCENT:
                    DriveVerticalAscent(s);
                    break;

                case AscentMode.INITIATE_TURN:
                    DriveInitiateTurn(s);
                    break;

                case AscentMode.GRAVITY_TURN:
                    DriveGravityTurn(s);
                    break;
            }

            return (mode != AscentMode.EXIT);
        }

        private void setTarget()
        {
            /* FIXME: inclination with no target */
            if ( core.target.NormalTargetExists )
            {
                peg.target = core.target.TargetOrbit;
            }
            else
            {
                peg.target = null;
            }
            peg.PeriapsisInsertion(autopilot.desiredOrbitAltitude, desiredApoapsis);
        }

        private void attitudeToPEG(double pitch)
        {
            double heading = peg.heading;
            /* FIXME: use srfvel heading if peg is bad */
            attitudeTo(pitch, heading);
        }

        private void DriveVerticalAscent(FlightCtrlState s)
        {

            //during the vertical ascent we just thrust straight up at max throttle
            attitudeToPEG(90);
            if (autopilot.autoThrottle) core.thrust.targetThrottle = 1.0F;

            core.attitude.AxisControl(!vessel.Landed, !vessel.Landed, !vessel.Landed && vesselState.altitudeBottom > 50);

            if (!vessel.LiftedOff() || vessel.Landed) {
                status = "Awaiting liftoff";
            }
            else
            {
                if (autopilot.MET > pitchStartTime)
                {
                    mode = AscentMode.INITIATE_TURN;
                    return;
                }
                double dt = pitchStartTime - autopilot.MET;
                status = String.Format("Vertical ascent {0:F2} s", dt);
            }
        }

        private void DriveInitiateTurn(FlightCtrlState s)
        {
            if (autopilot.autoThrottle) core.thrust.targetThrottle = 1.0F;

            double dt = autopilot.MET - pitchStartTime;
            double theta = dt * pitchRate;
            double pitch = 90 - theta + pitchBias;

            if (autopilot.MET > pitchEndTime || pitch < peg.pitch)
            {
                mode = AscentMode.GRAVITY_TURN;
                return;
            }

            status = String.Format("Pitch program {0:F2} s", pitchEndTime - pitchStartTime - dt);
        }

        private void DriveGravityTurn(FlightCtrlState s)
        {
            if (autopilot.autoThrottle) core.thrust.targetThrottle = 1.0F;
            if (autopilot.MET < pitchEndTime)
            {
                /* this can happen when users update the endtime box */
                mode = AscentMode.INITIATE_TURN;
                return;
            }

            /* FIXME: when PEG does not converge */
            status = "Stable PEG Guidance";
            attitudeToPEG(peg.pitch);

            if (peg.tgo < 0)
            {
                /* FIXME?: we could project a tgo_next and if its going to be negative kill the engines, then burn RCS to raise angular momentum or even add RCS as an additional PEG guidance stage? */
                status = "PEG zero tgo";
                core.thrust.targetThrottle = 0.0F;
                peg.enabled = false;
                mode = AscentMode.EXIT;
                return;
            }

            /*
            if (saneGuidance && guidanceEnabled) {
                if (terminalGuidance)
                    status = "Locked Terminal Guidance";
                else
                    status = "Stable PEG Guidance";

                attitudeTo(guidancePitch);
            }
            else
            {
                // srfvelPitch == zero AoA
                status = "Unguided Gravity Turn";
                attitudeTo(Math.Min(90, srfvelPitch() + pitchBias));
            }
            */
        }
    }
}
