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
		private Vector3 pError, iError, dError; // erreur précédente, intégrale de l'erreur, dérivée de l'erreur
		private Attitude attitude;
		private Rect windowPosition;
		private Boolean autopilot;

		public override void OnStart (StartState state)
		{
			base.OnStart (state);

			this.kp = 12;
			this.ki = 10;
			this.kd = 20;
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

		private void DrawGUI ()
		{
			GUI.skin = HighLogic.Skin;
			this.windowPosition = GUILayout.Window (1, this.windowPosition, AttitudeAutopilotGUI, "Attitude autopilot", GUILayout.MinWidth (100));
		}

		private void AttitudeAutopilotGUI (int windowID)
		{
			GUIStyle mySty = new GUIStyle (GUI.skin.button);
			mySty.normal.textColor = mySty.focused.textColor = Color.white;
			mySty.hover.textColor = mySty.active.textColor = Color.yellow;
			mySty.onNormal.textColor = mySty.onFocused.textColor = mySty.onHover.textColor = mySty.onActive.textColor = Color.green;
			mySty.padding = new RectOffset (8, 8, 8, 8);

			GUILayout.BeginVertical ();
			/*
			GUILayout.Label (Convert.ToString (this.kp * this.error.x), mySty, GUILayout.ExpandWidth (true));
			GUILayout.Label (Convert.ToString (this.kp * this.error.y), mySty, GUILayout.ExpandWidth (true));
			GUILayout.Label (Convert.ToString (this.kp * this.error.z), mySty, GUILayout.ExpandWidth (true));
			GUILayout.Label (Convert.ToString (this.ki * this.integralError.x), mySty, GUILayout.ExpandWidth (true));
			GUILayout.Label (Convert.ToString (this.ki * this.integralError.y), mySty, GUILayout.ExpandWidth (true));
			GUILayout.Label (Convert.ToString (this.ki * this.integralError.z), mySty, GUILayout.ExpandWidth (true));
			GUILayout.Label (Convert.ToString (this.kd * this.derivativeError.x), mySty, GUILayout.ExpandWidth (true));
			GUILayout.Label (Convert.ToString (this.kd * this.derivativeError.y), mySty, GUILayout.ExpandWidth (true));
			GUILayout.Label (Convert.ToString (this.kd * this.derivativeError.z), mySty, GUILayout.ExpandWidth (true));

*/
			if (GUILayout.Toggle (this.attitude == Attitude.None, "None", mySty, GUILayout.ExpandWidth (true))) {
				this.SetAttitude (Attitude.None);
			}
			if (GUILayout.Toggle (this.attitude == Attitude.Prograde, "Prograde", mySty, GUILayout.ExpandWidth (true))) {
				this.SetAttitude (Attitude.Prograde);
			}
			if (GUILayout.Toggle (this.attitude == Attitude.Retrograde, "Retrograde", mySty, GUILayout.ExpandWidth (true))) {
				this.SetAttitude (Attitude.Retrograde);
			}
			if (GUILayout.Toggle (this.attitude == Attitude.Nplus, "N+", mySty, GUILayout.ExpandWidth (true))) {
				this.SetAttitude (Attitude.Nplus);
			}
			if (GUILayout.Toggle (this.attitude == Attitude.Nminus, "N-", mySty, GUILayout.ExpandWidth (true))) {
				this.SetAttitude (Attitude.Nminus);
			}

			GUILayout.BeginHorizontal ();
			this.kp = float.Parse (GUILayout.TextField (Convert.ToString (this.kp), mySty, GUILayout.ExpandWidth (true)));
			this.ki = float.Parse (GUILayout.TextField (Convert.ToString (this.ki), mySty, GUILayout.ExpandWidth (true)));
			this.kd = float.Parse (GUILayout.TextField (Convert.ToString (this.kd), mySty, GUILayout.ExpandWidth (true)));
			GUILayout.EndHorizontal ();

			GUILayout.BeginHorizontal ();
			GUILayout.Label ("Status", mySty, GUILayout.ExpandWidth (true));
			this.autopilot = GUILayout.Toggle (this.autopilot, this.autopilot ? "Enabled" : "Disabled", mySty, GUILayout.ExpandWidth (true));
			GUILayout.EndHorizontal ();
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