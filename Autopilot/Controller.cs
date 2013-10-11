using System;

namespace Autopilot
{
	public abstract class Controller
	{

		public Autopilot Autopilot { get; set; }

		public bool Enabled { get; set; }

		public Controller (Autopilot autopilot)
		{
			if (autopilot == null)
				throw new NullReferenceException ();
			this.Autopilot = autopilot;
			this.Enabled = false;
		}

		public void Enable ()
		{
			this.Enabled = true;
		}

		public void Disable ()
		{
			this.Enabled = false;
		}

		protected abstract void _Drive (FlightCtrlState s);

		public void Drive (FlightCtrlState s)
		{
			if (this.Enabled)
				this._Drive (s);
		}

		public abstract void DrawGUI (int windowId);
	}
}