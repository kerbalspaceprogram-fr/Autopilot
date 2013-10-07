using System;
using UnityEngine;

namespace KspAutopilot
{
	public class Autopilot : PartModule
	{
		enum Attitude {
			None,
			Prograde,
			Retrograde,
			Nplus,
			Nminus,
			Up,
			Down,
			UserDefined
		};


		private float kp, ki, kd; // correcteur proportionnel, intégrateur, dérivateur
		private string skp, ski, skd; // pour l'interface graphique
		private Vector3 pError, iError, dError; // erreur précédente, intégrale de l'erreur, dérivée de l'erreur
		private Attitude attitude;
		private Rect windowPosition;
		private Boolean autopilot, autolaunch;
		private float pitch, yaw;
		private string spitch, syaw;

		public override void OnStart (StartState state)
		{
			base.OnStart (state);

			this.SetKp(12);
			this.SetKi(10);
			this.SetKd(20);
			this.SetPitch (0);
			this.SetYaw (0);
			this.SetAttitude (Attitude.None);
			this.autopilot = false;
			this.autolaunch = false;

			if (this.windowPosition.x == 0 && this.windowPosition.y == 0)
				this.windowPosition = new Rect (Screen.width / 2, Screen.height / 2, 10, 10);

			this.vessel.OnFlyByWire += new FlightInputCallback (this.Drive);
		}

		[KSPEvent(guiActive = true, guiName = "Attitude Autopilot")]
		public void Configuration ()
		{
			RenderingManager.AddToPostDrawQueue (3, new Callback (DrawGUI));
		}

		public void Drive (FlightCtrlState s)
		{
			if (!this.autopilot)
				return;


			if (this.autolaunch) {
				//Now we call this code regularly somewhere in the part logic:
				//Staging.CurrentStage != Staging.StageCount is a way to detect whether liftoff has occurred yet.
				//This condition is false until the first stage fires. The last stage is stage zero, so we should
				//only try to fire the next stage if this is not the last stage.
				/* !!!!!!! Ca vient du wiki et ça ne marche pas.... !!!!!!!
				if (Staging.CurrentStage != Staging.StageCount && Staging.CurrentStage > 0)
				{
					if (!inverseStageDecouplesActiveEngine(Staging.CurrentStage - 1, vessel.rootPart))
					{
						Staging.ActivateNextStage();
					}
				}
				*/
				if (this.vessel.altitude < 10000) {
					this.SetAttitude (Attitude.Up);
					s.mainThrottle = 1.0f;
				}
				else if (this.vessel.orbit.ApA < 100000) {
					if (this.pitch != 90) this.SetPitch (90);
					if (this.yaw != 45) this.SetYaw (45);
					if (this.attitude != Attitude.UserDefined) this.SetAttitude (Attitude.UserDefined);
					s.mainThrottle = 1.0f;
				} else {
					this.SetAttitude(Attitude.None);
					this.autolaunch = false;
					this.autopilot = false;
				}
			}


			Vector3 heading; 

			switch (this.attitude) {
			case Attitude.None:
				return;
			case Attitude.Prograde:
				heading = this.vessel.obt_velocity.normalized;
				break;
			case Attitude.Retrograde:
				heading = -this.vessel.obt_velocity.normalized;
				break;
			case Attitude.Nplus:
				heading = Vector3.Cross (this.vessel.obt_velocity, this.vessel.findWorldCenterOfMass () - this.vessel.mainBody.position).normalized;
				break;
			case Attitude.Nminus:
				heading = -Vector3.Cross (this.vessel.obt_velocity, this.vessel.findWorldCenterOfMass () - this.vessel.mainBody.position).normalized;
				break;
			case Attitude.Up:
				heading = (this.vessel.findWorldCenterOfMass () - this.vessel.mainBody.position).normalized;
				break;
			case Attitude.Down:
				heading = -(this.vessel.findWorldCenterOfMass () - this.vessel.mainBody.position).normalized;
				break;
			case Attitude.UserDefined:
				Vector3 position = this.vessel.findWorldCenterOfMass ();
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

			Vector3 error = this.vessel.transform.InverseTransformDirection (heading).normalized - Vector3.up;

			Vector3 command = this.ComputePID (error);

			s.pitch = - command.z;
			s.yaw = command.x;
		}

		private void SetAttitude (Attitude attitude)
		{
			this.attitude = attitude;
			this.ResetPID ();
		}

		private void SetPitch (float pitch)
		{
			this.pitch = pitch;
			this.spitch = Convert.ToString (pitch);
			this.ResetPID ();
		}

		private void SetYaw (float yaw)
		{
			this.yaw = yaw;
			this.syaw = Convert.ToString (yaw);
			this.ResetPID ();
		}

		private void SetKp (float kp)
		{
			this.kp = kp;
			this.skp = Convert.ToString (kp);
			this.ResetPID ();
		}

		private void SetKi (float ki)
		{
			this.ki = ki;
			this.ski = Convert.ToString (ki);
			this.ResetPID ();
		}

		private void SetKd (float kd)
		{
			this.kd = kd;
			this.skd = Convert.ToString (kd);
			this.ResetPID ();
		}

		private void DrawGUI ()
		{
			GUI.skin = HighLogic.Skin;
			this.windowPosition = GUILayout.Window (1, this.windowPosition, AttitudeAutopilotGUI, "Attitude autopilot", GUILayout.ExpandWidth (true), GUILayout.MinWidth (200));
		}

		private void AttitudeAutopilotGUI (int windowID)
		{

			GUILayout.BeginVertical ();

			if (GUILayout.Toggle (this.attitude == Attitude.None, "None",GUILayout.ExpandWidth (true))) {
				this.SetAttitude (Attitude.None);
			}
			if (GUILayout.Toggle (this.attitude == Attitude.Prograde, "Prograde", GUILayout.ExpandWidth (true))) {
				this.SetAttitude (Attitude.Prograde);
			}
			if (GUILayout.Toggle (this.attitude == Attitude.Retrograde, "Retrograde", GUILayout.ExpandWidth (true))) {
				this.SetAttitude (Attitude.Retrograde);
			}
			if (GUILayout.Toggle (this.attitude == Attitude.Nplus, "N+", GUILayout.ExpandWidth (true))) {
				this.SetAttitude (Attitude.Nplus);
			}
			if (GUILayout.Toggle (this.attitude == Attitude.Nminus, "N-", GUILayout.ExpandWidth (true))) {
				this.SetAttitude (Attitude.Nminus);
			}
			if (GUILayout.Toggle (this.attitude == Attitude.Up, "Up", GUILayout.ExpandWidth (true))) {
				this.SetAttitude (Attitude.Up);
			}
			if (GUILayout.Toggle (this.attitude == Attitude.Down, "Down", GUILayout.ExpandWidth (true))) {
				this.SetAttitude (Attitude.Down);
			}
			if (GUILayout.Toggle (this.attitude == Attitude.UserDefined, "User defined (*)", GUILayout.ExpandWidth (true))) {
				this.SetAttitude (Attitude.UserDefined);
			}

			float temp = 0;
			GUILayout.BeginHorizontal ();
			GUILayout.Label ("* Pitch", GUILayout.ExpandWidth (true));
			this.spitch = GUILayout.TextField (this.spitch, GUILayout.ExpandWidth (true));
			if (float.TryParse (this.spitch, out temp) && temp != this.pitch) this.SetPitch (temp);
			GUILayout.EndHorizontal ();

			GUILayout.BeginHorizontal ();
			GUILayout.Label ("* Yaw", GUILayout.ExpandWidth (true));
			this.syaw = GUILayout.TextField (this.syaw, GUILayout.ExpandWidth (true));
			if (float.TryParse (this.syaw, out temp) && temp != this.yaw) this.SetYaw (temp);
			GUILayout.EndHorizontal ();

			GUILayout.BeginHorizontal ();
			GUILayout.Label ("kp", GUILayout.ExpandWidth (true));
			this.skp = GUILayout.TextField (this.skp, GUILayout.ExpandWidth (true));
			if (float.TryParse (this.skp, out temp) && temp != this.kp) this.SetKp (temp);
			GUILayout.EndHorizontal ();

			GUILayout.BeginHorizontal ();
			GUILayout.Label ("ki", GUILayout.ExpandWidth (true));
			this.ski = GUILayout.TextField (this.ski, GUILayout.ExpandWidth (true));
			if (float.TryParse (this.ski, out temp) && temp != this.ki) this.SetKi (temp);
			GUILayout.EndHorizontal ();

			GUILayout.BeginHorizontal ();
			GUILayout.Label ("kd", GUILayout.ExpandWidth (true));
			this.skd = GUILayout.TextField (this.skd, GUILayout.ExpandWidth (true));
			if (float.TryParse (this.skd, out temp) && temp != this.kd) this.SetKd (temp);
			GUILayout.EndHorizontal ();

			GUILayout.BeginHorizontal ();
			GUILayout.Label ("Status", GUILayout.ExpandWidth (true));
			this.autopilot = GUILayout.Toggle (this.autopilot, this.autopilot ? "Enabled" : "Disabled", GUILayout.ExpandWidth (true));
			GUILayout.EndHorizontal ();

			GUILayout.BeginHorizontal ();
			GUILayout.Label ("Launch (auto staging & gravity turn)", GUILayout.ExpandWidth (true));
			this.autolaunch = GUILayout.Toggle (this.autolaunch, this.autolaunch ? "Enabled" : "Disabled", GUILayout.ExpandWidth (true));
			GUILayout.EndHorizontal ();

			if (GUILayout.Button ("Close", GUILayout.ExpandWidth (true))) RenderingManager.RemoveFromPostDrawQueue (3, new Callback (DrawGUI));

			GUILayout.EndVertical ();

			GUI.DragWindow (new Rect (0, 0, 10000, 20));
		}

		//Determines whether any of the parts under this part in the ship are engines in the ACTIVE or IDLE
		//state. Engines are IDLE before they are fired, ACTIVE while firing, and DEACTIVATED after running
		//out of fuel. So if a decoupler has an ACTIVE or IDLE engine descendant we shouldn't blow the decoupler, or
		//we will shed burning or unused engines.
		private bool hasActiveEngineDescendant(Part p)
		{
			//note that if someone makes a new engine that does not subclass LiquidEngine or SolidRocket
			//then this method will not account for it
			if ((p.State == PartStates.ACTIVE || p.State == PartStates.IDLE) && (p is SolidRocket || p is LiquidEngine)) return true;
			foreach (Part child in p.children)
			{
				if (hasActiveEngineDescendant(child)) return true;
			}
			return false;
		}

		//Parts store an "inverseStage" which is just the stage number that appears in the staging display
		//in the GUI, and which counts is counts downward to zero as you activate successive stages. This
		//method looks to see if any of the parts in the given inverseStage are decouplers that would
		//decouple an active or idle engine if we activated the given inverseStage.
		private bool inverseStageDecouplesActiveEngine(int inverseStage, Part root)
		{
			if (root.inverseStage == inverseStage && (root is Decoupler || root is RadialDecoupler) && hasActiveEngineDescendant(root)) return true;
			foreach (Part child in root.children)
			{
				if (inverseStageDecouplesActiveEngine(inverseStage, child)) return true;
			}
			return false;
		}

		private Vector3d ComputePID (Vector3 error)
		{
			this.iError += error * TimeWarp.fixedDeltaTime; // calcul de l'intégrale de l'erreur
			this.dError = (error - this.pError) / TimeWarp.fixedDeltaTime; // calcul de la dérivée de l'erreur

			Vector3 result = this.kp * error + this.ki * this.iError + this.kd * this.dError; // resultat du pid

			Vector3 clampedResult = new Vector3d (// on bride le resultat entre un max et un min
				Mathf.Clamp (result.x, -1.0f, 1.0f),
				Mathf.Clamp (result.y, -1.0f, 1.0f),
				Mathf.Clamp (result.z, -1.0f, 1.0f));

			if (Math.Abs ((clampedResult - result).magnitude) > 0.01) { // si le resultat du pid a été trop bridé, alors on ne veut pas que l'intégrale de l'erreur prenne en compte ce cycle de calcul.
				this.iError -= error * TimeWarp.fixedDeltaTime; // On retire simplement la valeur qui a été ajoutée plus haut dans le calcul de l'intégrale de l'erreur
			}

			this.pError = error;

			return clampedResult;
		}

		private void ResetPID ()
		{
			this.iError = Vector3d.zero;
			this.dError = Vector3d.zero;
		}
	}
}