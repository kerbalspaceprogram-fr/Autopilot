using System;
using UnityEngine;

namespace Autopilot
{
	public class AttitudeController : Controller
	{
		private Attitude attitude;
		private float pitch, yaw;
		private string spitch, syaw;
		private PidController pidController;

		public AttitudeController (Autopilot autopilot) : base(autopilot)
		{
			this.pidController = new PidController (12, 10, 20);
			this.pitch = 0;
			this.yaw = 0;
			this.spitch = Convert.ToString (this.pitch);
			this.syaw = Convert.ToString (this.yaw);
		}

		protected override void _Drive (FlightCtrlState s)
		{
			Vessel vessel = base.Autopilot.vessel;
			Vector3 heading; 

			switch (this.attitude) {
			case Attitude.None:
				return;
			case Attitude.Prograde:
				heading = vessel.obt_velocity.normalized;
				break;
			case Attitude.Retrograde:
				heading = -vessel.obt_velocity.normalized;
				break;
			case Attitude.Nplus:
				heading = vessel.orbit.GetOrbitNormal ();
				break;
			case Attitude.Nminus:
				heading = -vessel.orbit.GetOrbitNormal ();
				break;
			case Attitude.Up:
				heading = (vessel.findWorldCenterOfMass () - vessel.mainBody.position).normalized;
				break;
			case Attitude.Down:
				heading = -(vessel.findWorldCenterOfMass () - vessel.mainBody.position).normalized;
				break;
			case Attitude.ManeuverNode:
				heading = base.Autopilot.ManeuverNode.GetBurnVector (vessel.orbit).normalized;
				break;
			case Attitude.UserDefined:
				Vector3 position = vessel.findWorldCenterOfMass ();
				Vector3d east = vessel.mainBody.getRFrmVel (position).normalized;
				Vector3d up = (position - vessel.mainBody.position).normalized;
				Vector3d north = Vector3d.Cross (east, up);

				Quaternion qPitch = Quaternion.AngleAxis (this.pitch, up);
				Quaternion qYaw = Quaternion.AngleAxis (-this.yaw, east);
				heading = qPitch * qYaw * north;
				break;
			default:
				heading = Vector3.zero;
				break;
			}

			Vector3 error = vessel.transform.InverseTransformDirection (heading).normalized - Vector3.up;

			Vector3 command = this.pidController.Compute (error);

			s.pitch = - command.z;
			s.yaw = command.x;
		}

		public override void DrawGUI (int windowId)
		{
			if (GUILayout.Toggle (this.Attitude == Attitude.None, "None", GUILayout.ExpandWidth (true))) {
				this.Attitude = Attitude.None;
			}
			if (GUILayout.Toggle (this.Attitude == Attitude.Prograde, "Prograde", GUILayout.ExpandWidth (true))) {
				this.Attitude = Attitude.Prograde;
			}
			if (GUILayout.Toggle (this.Attitude == Attitude.Retrograde, "Retrograde", GUILayout.ExpandWidth (true))) {
				this.Attitude = Attitude.Retrograde;
			}
			if (GUILayout.Toggle (this.Attitude == Attitude.Nplus, "N+", GUILayout.ExpandWidth (true))) {
				this.Attitude = Attitude.Nplus;
			}
			if (GUILayout.Toggle (this.Attitude == Attitude.Nminus, "N-", GUILayout.ExpandWidth (true))) {
				this.Attitude = Attitude.Nminus;
			}
			if (GUILayout.Toggle (this.Attitude == Attitude.Up, "Up", GUILayout.ExpandWidth (true))) {
				this.Attitude = Attitude.Up;
			}
			if (GUILayout.Toggle (this.Attitude == Attitude.Down, "Down", GUILayout.ExpandWidth (true))) {
				this.Attitude = Attitude.Down;
			}
			if (GUILayout.Toggle (this.Attitude == Attitude.ManeuverNode, "Maneuver node", GUILayout.ExpandWidth (true))) {
				this.Attitude = Attitude.ManeuverNode;
			}
			if (GUILayout.Toggle (this.Attitude == Attitude.UserDefined, "User defined (*)", GUILayout.ExpandWidth (true))) {
				this.Attitude = Attitude.UserDefined;
			}

			float temp;
			GUILayout.BeginHorizontal ();
			GUILayout.Label ("* Pitch", GUILayout.ExpandWidth (true));
			this.spitch = GUILayout.TextField (this.spitch, GUILayout.ExpandWidth (true));
			if (float.TryParse (this.spitch, out temp))
				this.Pitch = temp;
			GUILayout.EndHorizontal ();

			GUILayout.BeginHorizontal ();
			GUILayout.Label ("* Yaw", GUILayout.ExpandWidth (true));
			this.syaw = GUILayout.TextField (this.syaw, GUILayout.ExpandWidth (true));
			if (float.TryParse (this.syaw, out temp))
				this.Yaw = temp;
			GUILayout.EndHorizontal ();

			this.pidController.Draw (windowId);

		}

		public Attitude Attitude {
			get {
				return this.attitude;
			}
			set {
				if (this.attitude == value)
					return;

				if (value == Attitude.ManeuverNode && base.Autopilot.ManeuverNode == null)
					this.attitude = Attitude.None;
				else
					this.attitude = value;

				this.pidController.ResetPID ();
			}
		}

		public float Pitch {
			get {
				return this.pitch;
			}
			set {
				if (this.pitch == value)
					return;

				this.pitch = value;
				this.spitch = Convert.ToString (this.pitch);
				this.pidController.ResetPID ();
			}
		}

		public float Yaw {
			get {
				return this.yaw;
			}
			set {
				if (this.yaw == value)
					return;

				this.yaw = value;
				this.syaw = Convert.ToString (this.yaw);
				this.pidController.ResetPID ();
			}
		}
	}
}