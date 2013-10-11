using System;
using System.Collections.Generic;
using UnityEngine;

namespace Autopilot
{
	public class AutoStagingController : Controller
	{
		public AutoStagingController (Autopilot autopilot) : base(autopilot)
		{
		}


		protected override void _Drive (FlightCtrlState s)
		{
		Vessel vessel = base.Autopilot.vessel;

			if (LiftedOff(vessel) && Staging.CurrentStage > 0)
			{
				//don't decouple active or idle engines or tanks
				List<int> burnedResources = FindBurnedResources(vessel);
				if (!InverseStageDecouplesActiveOrIdleEngineOrTank(Staging.CurrentStage - 1, vessel, burnedResources))
				{
					//Don't fire a stage that will activate a parachute, unless that parachute gets decoupled:
					if (!HasStayingChutes(Staging.CurrentStage - 1, vessel))
					{
						//only fire decouplers to drop deactivated engines or tanks
						if (!InverseStageFiresDecoupler(Staging.CurrentStage - 1, vessel) || InverseStageDecouplesDeactivatedEngineOrTank (Staging.CurrentStage - 1, vessel)) {
							//When we find that we're allowed to stage, start a countdown (with a
							//length given by autostagePreDelay) and only stage once that countdown finishes,
							Staging.ActivateNextStage ();

						}

					}
				}
			}
		}

		//determine whether activating inverseStage will fire any sort of decoupler. This
		//is used to tell whether we should delay activating the next stage after activating inverseStage
		private bool InverseStageFiresDecoupler (int inverseStage, Vessel v)
		{
			foreach (Part p in v.parts) {
				if (p.inverseStage == inverseStage && IsDecoupler (p))
					return true;
			}
			return false;
		}
		//determine whether inverseStage sheds a dead engine
		private bool InverseStageDecouplesDeactivatedEngineOrTank (int inverseStage, Vessel v)
		{
			foreach (Part p in v.parts) {
				if (p.inverseStage == inverseStage && IsDecoupler (p) && HasDeactivatedEngineOrTankDescendant (p))
					return true;
			}
			return false;
		}
		//detect if a part is above a deactivated engine or fuel tank
		private bool HasDeactivatedEngineOrTankDescendant (Part p)
		{
			if ((p.State == PartStates.DEACTIVATED) && (p is FuelTank || IsEngine (p)) && !IsSepratron (p)) {
				return true; // TODO: yet more ModuleEngine lazy checks
			}

			//check if this is a new-style fuel tank that's run out of resources:
			bool hadResources = false;
			bool hasResources = false;
			foreach (PartResource r in p.Resources) {
				if (r.name == "ElectricCharge")
					continue;
				if (r.maxAmount > 0)
					hadResources = true;
				if (r.amount > 0)
					hasResources = true;
			}
			if (hadResources && !hasResources)
				return true;

			if (IsEngine (p) && !EngineHasFuel (p))
				return true;

			foreach (Part child in p.children) {
				if (HasDeactivatedEngineOrTankDescendant (child))
					return true;
			}
			return false;
		}
		//determine if there are chutes being fired that wouldn't also get decoupled
		private bool HasStayingChutes (int inverseStage, Vessel v)
		{
			var chutes = v.parts.FindAll (p => p.inverseStage == inverseStage && IsParachute (p));

			foreach (Part p in chutes) {
				if (!IsDecoupledInStage (p, inverseStage)) {
					return true;
				}
			}

			return false;
		}
		//determine whether it's safe to activate inverseStage
		private bool InverseStageDecouplesActiveOrIdleEngineOrTank (int inverseStage, Vessel v, List<int> tankResources)
		{
			foreach (Part p in v.parts) {
				if (p.inverseStage == inverseStage && IsDecoupler (p) && HasActiveOrIdleEngineOrTankDescendant (p, tankResources)) {
					return true;
				}
			}
			return false;
		}

		private List<int> FindBurnedResources (Vessel vessel)
		{
			//var activeEngines = vessel.parts.Where(p => p.inverseStage >= Staging.CurrentStage && IsEngine(p) && !IsSepratron(p));
			List<Part> activeEngines = new List<Part> ();
			foreach (Part p in vessel.parts)
				if (p.inverseStage >= Staging.CurrentStage && IsEngine (p) && !IsSepratron (p))
					activeEngines.Add (p);

			//var engineModules = activeEngines.Select(p => p.Modules.OfType<ModuleEngines>().First());
			List<ModuleEngines> engineModules = new List<ModuleEngines> ();
			foreach (Part p in activeEngines)
				foreach (PartModule pm in p.Modules)
					if (pm is ModuleEngines) {
						engineModules.Add ((ModuleEngines)pm);
						break;
					}

			//var  burnedPropellants = engineModules.SelectMany(eng => eng.propellants);
			List<ModuleEngines.Propellant> burnedPropellants = new List<ModuleEngines.Propellant> ();
			foreach (ModuleEngines me in engineModules)
				foreach (ModuleEngines.Propellant mep in me.propellants)
					burnedPropellants.Add (mep);

			//List<int> propellantIDs = burnedPropellants.Select(prop => prop.id).ToList();
			List<int> propellantIDs = new List<int> ();
			foreach (ModuleEngines.Propellant mpe in burnedPropellants)
				propellantIDs.Add (mpe.id);


			return propellantIDs;
		}
		//detect if a part is above an active or idle engine in the part tree
		private bool HasActiveOrIdleEngineOrTankDescendant (Part p, List<int> tankResources)
		{
			if ((p.State == PartStates.ACTIVE || p.State == PartStates.IDLE)
				&& IsEngine (p) && !IsSepratron (p) && EngineHasFuel (p)) {
				return true; // TODO: properly check if ModuleEngines is active
			}
			if ((p is FuelTank) && (((FuelTank)p).fuel > 0))
				return true;
			if (!IsSepratron (p)) {
				foreach (PartResource r in p.Resources) {
					if (r.amount > 0 && r.info.name != "ElectricCharge" && tankResources.Contains (r.info.id)) {
						return true;
					}
				}
			}
			foreach (Part child in p.children) {
				if (HasActiveOrIdleEngineOrTankDescendant (child, tankResources))
					return true;
			}
			return false;
		}
		//Any engine that is decoupled in the same stage in
		//which it activates we call a sepratron.
		private bool IsSepratron (Part p)
		{
			return p.ActivatesEvenIfDisconnected
				&& IsEngine (p)
				&& IsDecoupledInStage (p, p.inverseStage);
		}

		private bool IsDecoupledInStage (Part p, int stage)
		{
			if ((IsDecoupler (p) || IsLaunchClamp (p)) && p.inverseStage == stage)
				return true;
			if (p.parent == null)
				return false;
			return IsDecoupledInStage (p.parent, stage);
		}

		private bool IsLaunchClamp (Part p)
		{
			return HasModule<LaunchClamp> (p);
		}

		private bool LiftedOff (Vessel vessel)
		{
			return vessel.situation != Vessel.Situations.PRELAUNCH;
		}

		private bool IsDecoupler (Part p)
		{
			return (p is Decoupler ||
				p is DecouplerGUI ||
				p is RadialDecoupler ||
				HasModule<ModuleDecouple> (p) ||
				HasModule<ModuleAnchoredDecoupler> (p));
		}

		private bool IsEngine (Part p)
		{
			return (p is SolidRocket ||
				p is LiquidEngine ||
				p is LiquidFuelEngine ||
				p is AtmosphericEngine ||
				HasModule<ModuleEngines> (p));
		}

		private bool HasModule<T> (Part p) where T : PartModule
		{
			for (int i=0; i<p.Modules.Count; i++)
				if (p.Modules.GetModule (i) is T)
					return true;

			return false;
		}

		private bool IsParachute (Part p)
		{
			return p is Parachutes ||
				p is HParachutes ||
				HasModule<ModuleParachute> (p);
		}

		private bool EngineHasFuel (Part p)
		{
			if (p is LiquidEngine || p is LiquidFuelEngine || p is AtmosphericEngine) {
				//I don't really know the details of how you're supposed to use RequestFuel, but this seems to work to
				//test whether something can get fuel.
				return p.RequestFuel (p, 0, Part.getFuelReqId ());
			} else if (HasModule<ModuleEngines> (p)) {
				for (int i=0; i<p.Modules.Count; i++)
					if (p.Modules.GetModule (i) is ModuleEngines)
						return !((ModuleEngines)p.Modules.GetModule (i)).getFlameoutState;
				return false;
			} else
				return false;
		}

		public override void DrawGUI(int windowId)
		{
			GUILayout.BeginHorizontal ();
			GUILayout.Label ("Autostaging", GUILayout.ExpandWidth (true));
			base.Enabled = GUILayout.Toggle (base.Enabled, base.Enabled ? "Enabled" : "Disabled", GUILayout.ExpandWidth (true));
			GUILayout.EndHorizontal ();
		}
	}
}

