using System;
using System.Collections.Generic;
using UnityEngine;

namespace Autopilot
{
	public class AutopilotController : Controller
	{
		private List<Controller> controllers;

		public AutopilotController (Autopilot autopilot) : base(autopilot)
		{
			this.controllers = new List<Controller> ();
			this.controllers.Add (new AttitudeController (base.Autopilot));
			this.controllers.Add (new AutoLaunchController (base.Autopilot));
			this.controllers.Add (new AutoStagingController (base.Autopilot));

			this.AttitudeController.Enable ();
		}

		protected override void _Drive (FlightCtrlState s)
		{
			foreach (Controller c in this.controllers)
				c.Drive (s);
		}

		public override void DrawGUI (int windowId)
		{
			GUILayout.BeginHorizontal ();
			GUILayout.Label ("Status", GUILayout.ExpandWidth (true));
			base.Enabled = GUILayout.Toggle (base.Enabled, base.Enabled ? "Enabled" : "Disabled", GUILayout.ExpandWidth (true));
			GUILayout.EndHorizontal ();

			foreach (Controller c in this.controllers)
				c.DrawGUI (windowId);
		}

		public AttitudeController AttitudeController {
			get {
				return (AttitudeController)this.controllers.Find (delegate(Controller c) {
					return c is AttitudeController;
				});
			}
		}

		public AttitudeController AutoLaunchController {
			get {
				return (AttitudeController)this.controllers.Find (delegate(Controller c) {
					return c is AutoLaunchController;
				});
			}
		}

		public AutoStagingController AutoStagingController {
			get {
				return (AutoStagingController)this.controllers.Find (delegate(Controller c) {
					return c is AutoStagingController;
				});
			}
		}
	}
}

