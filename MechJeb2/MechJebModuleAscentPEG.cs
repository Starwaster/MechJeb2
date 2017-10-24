using System;
using KSP.UI.Screens;
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
        public EditableDouble pitchStartTime = new EditableDouble(10);
        [Persistent(pass = (int)(Pass.Type | Pass.Global))]
        public EditableDouble pitchRate = new EditableDouble(0.75);
        [Persistent(pass = (int)(Pass.Type | Pass.Global))]
        public EditableDouble pitchEndTime = new EditableDouble(55);
        [Persistent(pass = (int)(Pass.Global))]
        public EditableDoubleMult desiredApoapsis = new EditableDoubleMult(0, 1000);
        [Persistent(pass = (int)(Pass.Type | Pass.Global))]
        public bool pitchEndToggle = false;
        [Persistent(pass = (int)(Pass.Type | Pass.Global))]
        public bool pegAfterStageToggle = false;
        [Persistent(pass = (int)(Pass.Type | Pass.Global))]
        public EditableDouble pegAfterStage = new EditableDouble(0);
        [Persistent(pass = (int)(Pass.Type | Pass.Global))]
        public EditableDouble pegInterval = new EditableDouble(1);

        /* this deliberately does not persist, it is for emergencies only */
        public EditableDouble pitchBias = new EditableDouble(0);

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
            if ( core.target.NormalTargetExists )
            {
                peg.target = core.target.TargetOrbit;
            }
            else
            {
                peg.SetPlaneFromInclination(autopilot.desiredInclination);
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

            if ((pitchEndToggle && autopilot.MET > pitchEndTime) || ( pitch < peg.pitch && !peg.failed && peg.converged))
            {
                mode = AscentMode.GRAVITY_TURN;
                return;
            }
            attitudeToPEG(pitch);

            if ( pitchEndToggle )
                status = String.Format("Pitch program {0:F2} s", pitchEndTime - pitchStartTime - dt);
            else
                status = String.Format("Pitch program {0:F2} °", pitch - peg.pitch);
        }

        private double targetAngularMomentum()
        {
            return peg.rdval * peg.vdval;  /* FIXME: periapsis insertion */
        }

        private void DriveGravityTurn(FlightCtrlState s)
        {
            if (autopilot.autoThrottle) core.thrust.targetThrottle = 1.0F;

            if (pitchEndToggle && autopilot.MET < pitchEndTime)
            {
                // this can happen when users update the endtime box
                mode = AscentMode.INITIATE_TURN;
                return;
            }

            if ( pegAfterStageToggle && StageManager.CurrentStage >= pegAfterStage )
            {
                status = "Unguided Gravity Turn";
                attitudeToPEG(Math.Min(90, srfvelPitch() + pitchBias));
            }
            else
            {
                if (peg.terminalGuidance && !peg.failed && peg.converged)
                    status = "Locked Terminal Guidance";
                else
                    status = "Stable PEG Guidance";

                attitudeToPEG(peg.pitch);

                /* FIXME: probably move this to the PEG controller itself */
                if (vessel.orbit.h.magnitude > targetAngularMomentum() )
                {
                    status = "Angular momentum target reached";
                    core.thrust.targetThrottle = 0.0F;
                    peg.enabled = false;
                    mode = AscentMode.EXIT;
                    return;
                }
            }
        }
    }
}
