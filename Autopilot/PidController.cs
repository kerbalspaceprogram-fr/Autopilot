using System;
using UnityEngine;

namespace Autopilot
{
	public class PidController //: Controller
	{

		private float kp, ki, kd; // correcteur proportionnel, intégrateur, dérivateur
		private string skp, ski, skd;
		private Vector3 pError, iError, dError; // erreur précédente, intégrale de l'erreur, dérivée de l'erreur

		public PidController (float kp, float ki, float kd)// : base(autopilot)
		{
			this.kp = kp;
			this.ki = ki;
			this.kd = kd;
			this.skp = Convert.ToString (kp);
			this.ski = Convert.ToString (ki);
			this.skd = Convert.ToString (kd);
			this.pError = Vector3.zero;
			this.iError = Vector3.zero;
			this.dError = Vector3.zero;
		}

		public Vector3 Compute (Vector3 error)
		{
			this.iError += error * TimeWarp.fixedDeltaTime; // calcul de l'intégrale de l'erreur
			this.dError = (error - this.pError) / TimeWarp.fixedDeltaTime; // calcul de la dérivée de l'erreur

			Vector3 result = this.kp * error + this.ki * this.iError + this.kd * this.dError; // resultat du pid

			Vector3 clampedResult = new Vector3 (// on bride le resultat entre un max et un min
			                                      Mathf.Clamp (result.x, -1.0f, 1.0f),
			                                      Mathf.Clamp (result.y, -1.0f, 1.0f),
			                                      Mathf.Clamp (result.z, -1.0f, 1.0f));

			if (Math.Abs ((clampedResult - result).magnitude) > 0.01) { // si le resultat du pid a été trop bridé, alors on ne veut pas que l'intégrale de l'erreur prenne en compte ce cycle de calcul.
				this.iError -= error * TimeWarp.fixedDeltaTime; // On retire simplement la valeur qui a été ajoutée plus haut dans le calcul de l'intégrale de l'erreur
			}

			this.pError = error;

			return clampedResult;
		}

		public float Kp
		{
			get {
				return this.kp;
			}
			set {
				if (this.kp == value)
					return;

				this.kp = value;
				this.ResetPID ();
			}
		}

		public float Ki
		{
			get {
				return this.ki;
			}
			set {
				if (this.ki == value)
					return;

				this.ki = value;
				this.ResetPID ();
			}
		}

		public float Kd
		{
			get {
				return this.kd;
			}
			set {
				if (this.kd == value)
					return;

				this.kd = value;
				this.ResetPID ();
			}
		}

		public void ResetPID ()
		{
			this.pError = Vector3.zero;
			this.iError = Vector3.zero;
			this.dError = Vector3.zero;
		}

		public void Draw(int windowId)
		{
			float temp;
			GUILayout.BeginHorizontal ();
			GUILayout.Label ("kp", GUILayout.ExpandWidth (true));
			this.skp = GUILayout.TextField (this.skp, GUILayout.ExpandWidth (true));
			if (float.TryParse (this.skp, out temp)) this.Kp = temp;
			GUILayout.EndHorizontal ();

			GUILayout.BeginHorizontal ();
			GUILayout.Label ("ki", GUILayout.ExpandWidth (true));
			this.ski = GUILayout.TextField (this.ski, GUILayout.ExpandWidth (true));
			if (float.TryParse (this.ski, out temp)) this.Ki = temp;
			GUILayout.EndHorizontal ();

			GUILayout.BeginHorizontal ();
			GUILayout.Label ("kd", GUILayout.ExpandWidth (true));
			this.skd = GUILayout.TextField (this.skd, GUILayout.ExpandWidth (true));
			if (float.TryParse (this.skd, out temp)) this.Kd = temp;
			GUILayout.EndHorizontal ();
		}
	}
}

