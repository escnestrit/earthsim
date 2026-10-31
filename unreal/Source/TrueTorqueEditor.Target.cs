using UnrealBuildTool;

public class TrueTorqueEditorTarget : TargetRules
{
	public TrueTorqueEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		ExtraModuleNames.Add("TrueTorque");
	}
}
