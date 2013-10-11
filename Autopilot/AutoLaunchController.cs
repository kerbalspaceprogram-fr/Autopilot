using System;
using System.Collections.Generic;
using UnityEngine;

namespace Autopilot
{
	public class AutoLaunchController : Controller
	{
		public AutoLaunchController (Autopilot autopilot)  : base(autopilot)
		{
		}

		protected override void _Drive (FlightCtrlState s)
		{
			Vessel vessel = base.Autopilot.vessel;
			AttitudeController attitudeController = base.Autopilot.AutopilotController.AttitudeController;

			if (vessel.altitude < 10000) {
				attitudeController.Attitude = Attitude.Up;
				base.Autopilot.AutopilotController.AutoStagingController.Enable ();
				s.mainThrottle = 1.0f;
			} else if (vessel.altitude < 80000) {
				attitudeController.Pitch = 90;
				attitudeController.Yaw = 45;
				attitudeController.Attitude = Attitude.UserDefined;
				s.mainThrottle = Mathf.Clamp01 (100000.0f - (float)vessel.orbit.ApA);
			} else {
				double ut = Planetarium.GetUniversalTime () + vessel.orbit.timeToAp;


				double deltaV = Math.Sqrt (vessel.mainBody.gravParameter / (100000 + vessel.mainBody.Radius)) - vessel.orbit.getOrbitalVelocityAtUT (ut).magnitude;

				Vector3d nodeDV = new Vector3d (0, 0, deltaV);
				base.Autopilot.ManeuverNode = vessel.patchedConicSolver.AddManeuverNode (ut);
				base.Autopilot.ManeuverNode.OnGizmoUpdated (nodeDV, ut);

				base.Autopilot.AutopilotController.Disable ();
				base.Autopilot.AutopilotController.AutoStagingController.Disable ();
				attitudeController.Attitude = Attitude.None;
			}
		}

		public override void DrawGUI (int windowId)
		{
			GUILayout.BeginHorizontal ();
			GUILayout.Label ("Launch to 100km (gravity turn@10km)", GUILayout.ExpandWidth (true));
			base.Enabled = GUILayout.Toggle (base.Enabled, base.Enabled ? "Enabled" : "Disabled", GUILayout.ExpandWidth (true));
			GUILayout.EndHorizontal ();
		}
	}
}

