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
			Nminus
		};


		private float kp, ki, kd; // correcteur proportionnel, intégrateur, dérivateur
		private string skp, ski, skd; // pour l'interface graphique
		private Vector3 pError, iError, dError; // erreur précédente, intégrale de l'erreur, dérivée de l'erreur
		private Attitude attitude;
		private Rect windowPosition;
		private Boolean autopilot;

		public override void OnStart (StartState state)
		{
			base.OnStart (state);

			this.SetKp(12);
			this.SetKi(10);
			this.SetKd(20);
			this.SetAttitude (Attitude.None);
			this.autopilot = false;

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

			Vector3 heading; 

			switch (this.attitude) {
			case Attitude.None:
				return;
			case Attitude.Prograde:
				heading = this.vessel.obt_velocity.normalized;
				break;
			case Attitude.Retrograde:
				heading = - this.vessel.obt_velocity.normalized;
				break;
			case Attitude.Nplus:
				heading = Vector3.Cross (this.vessel.obt_velocity, this.vessel.findWorldCenterOfMass () - this.vessel.mainBody.position).normalized;
				break;
			case Attitude.Nminus:
				heading = -Vector3.Cross (this.vessel.obt_velocity, this.vessel.findWorldCenterOfMass () - this.vessel.mainBody.position).normalized;
				break;
			default:
				heading = Vector3.zero;
				break;
			}

			Vector3 error = this.vessel.transform.InverseTransformDirection (heading).normalized - this.vessel.transform.InverseTransformDirection (this.transform.up).normalized;

			Vector3 command = this.ComputePID (error);

			s.pitch = - command.z;
			s.yaw = command.x;
		}

		private void SetAttitude (Attitude attitude)
		{
			this.attitude = attitude;
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

			float temp = 0;
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

			if (GUILayout.Button ("Close", GUILayout.ExpandWidth (true))) RenderingManager.RemoveFromPostDrawQueue (3, new Callback (DrawGUI));

			GUILayout.EndVertical ();

			GUI.DragWindow (new Rect (0, 0, 10000, 20));
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