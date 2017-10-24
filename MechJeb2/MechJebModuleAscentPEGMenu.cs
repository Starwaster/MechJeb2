﻿using System;
using UnityEngine;

namespace MuMech
{
    public class MechJebModuleAscentPEGMenu : MechJebModuleAscentMenuBase
    {
        public MechJebModuleAscentPEGMenu(MechJebCore core)
            : base(core)
        {
            hidden = true;
        }

        public MechJebModuleAscentPEG path { get { return autopilot.ascentPath as MechJebModuleAscentPEG; } }
        private MechJebModulePEGController peg { get { return core.GetComputerModule<MechJebModulePEGController>(); } }
        public MechJebModuleAscentAutopilot autopilot;

        public override void OnStart(PartModule.StartState state)
        {
            autopilot = core.GetComputerModule<MechJebModuleAscentAutopilot>();
        }

        public override GUILayoutOption[] WindowOptions()
        {
            return new GUILayoutOption[] { GUILayout.Width(300), GUILayout.Height(100) };
        }

        protected override void WindowGUI(int windowID)
        {
            if (path == null)
            {
                GUILayout.Label("Path is null!!!1!!1!1!1111!11eleven");
                base.WindowGUI(windowID);
                return;
            }

            GUILayout.BeginVertical();

            GuiUtils.SimpleTextBox("Target Periapsis:", autopilot.desiredOrbitAltitude, "km");
            GuiUtils.SimpleTextBox("Target Apoapsis:", path.desiredApoapsis, "km");
            if ( path.desiredApoapsis >= 0 && path.desiredApoapsis < autopilot.desiredOrbitAltitude )
            {
                GUIStyle s = new GUIStyle(GUI.skin.label);
                s.normal.textColor = Color.yellow;
                GUILayout.Label("Apoapsis < Periapsis: circularizing orbit at periapsis", s);
            }

            GuiUtils.SimpleTextBox("Booster Pitch start:", path.pitchStartTime, "s");
            GuiUtils.SimpleTextBox("Booster Pitch rate:", path.pitchRate, "°/s");
            GUILayout.BeginHorizontal();
            path.pitchEndToggle = GUILayout.Toggle(path.pitchEndToggle, "Booster Pitch end:");
            if (path.pitchEndToggle)
                GuiUtils.SimpleTextBox("", path.pitchEndTime, "s");
            GUILayout.EndHorizontal();
            if (path.pitchEndToggle)
                GUILayout.Label(String.Format("ending pitch: {0:F1}°", 90.0 - (path.pitchEndTime - path.pitchStartTime)*path.pitchRate));
            GUILayout.BeginHorizontal();
            path.pegAfterStageToggle = GUILayout.Toggle(path.pegAfterStageToggle, "Start PEG after KSP Stage #");
            if (path.pegAfterStageToggle)
                GuiUtils.SimpleTextBox("", path.pegAfterStage);
            GUILayout.EndHorizontal();
            GuiUtils.SimpleTextBox("Terminal Guidance Period:", peg.terminalGuidanceTime, "s");
            GuiUtils.SimpleTextBox("PEG Update Interval:", path.pegInterval, "s");
            GUILayout.Label("Stage Stats");
            if (GUILayout.Button("Reset PEG"))
                peg.Reset();
            GuiUtils.SimpleTextBox("Stage Minimum dV Limit:", peg.stageLowDVLimit, "m/s");

            for(int i = peg.stages.Count - 1; i >= 0; i--) {
                GUILayout.Label(String.Format("{0:D}: {1:D} {2:F1} {3:F1}", i, peg.stages[i].kspStage, peg.stages[i].dt, peg.stages[i].Li));
            }
            GUILayout.Label(String.Format("vgo: {0:F1}", peg.vgo.magnitude));
            GUILayout.Label(String.Format("tgo: {0:F1}", peg.tgo));
            GUILayout.Label(String.Format("heading: {0:F1}", peg.heading));
            GUILayout.Label(String.Format("pitch: {0:F1}", peg.pitch));
            GUILayout.Label(String.Format("primer mag: {0:F1}", peg.primerMag));
            GUILayout.Label(String.Format("phi: {0:F1}", peg.phi2 * UtilMath.Rad2Deg));

            GuiUtils.SimpleTextBox("Emergency pitch adj.:", path.pitchBias, "°");


            if (autopilot.enabled)
            {
                GUILayout.Label("Autopilot status: " + autopilot.status);
            }


            GUILayout.EndVertical();
            base.WindowGUI(windowID);
        }

        public override string GetName()
        {
            return "Space Shuttle PEG Pitch Program";
        }
    }
}
