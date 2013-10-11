using System;
using UnityEngine;
using System.Collections.Generic;

namespace Autopilot
{
	public class Autopilot : PartModule
	{
		private Rect windowPosition;
		private AutopilotController autopilotController;
		public ManeuverNode ManeuverNode { get; set; }

		public override void OnStart (StartState state)
		{
			base.OnStart (state);

			this.autopilotController = new AutopilotController (this);

			if (this.windowPosition.x == 0 && this.windowPosition.y == 0)
				this.windowPosition = new Rect (Screen.width / 2, Screen.height / 2, 10, 10);

			this.vessel.OnFlyByWire += new FlightInputCallback (this.Drive);
		}

		[KSPEvent(guiActive = true, guiName = "Autopilot")]
		public void Configuration ()
		{
			RenderingManager.AddToPostDrawQueue (3, new Callback (AutopilotGUI));
		}

		public void Drive (FlightCtrlState s)
		{
				this.autopilotController.Drive (s);
		}

		private void AutopilotGUI ()
		{
			GUI.skin = HighLogic.Skin;
			this.windowPosition = GUILayout.Window (1, this.windowPosition, DrawGUI, "Autopilot", GUILayout.ExpandWidth (true), GUILayout.MinWidth (200));
		}

		private void DrawGUI (int windowId)
		{
			GUILayout.BeginVertical ();

			this.autopilotController.DrawGUI (windowId);

			if (GUILayout.Button ("Close", GUILayout.ExpandWidth (true))) RenderingManager.RemoveFromPostDrawQueue (3, new Callback (AutopilotGUI));

			GUILayout.EndVertical ();

			GUI.DragWindow (new Rect (0, 0, 10000, 20));
		}

		public AutopilotController AutopilotController
		{
			get {
				return this.autopilotController;
			}
		}
	}
}