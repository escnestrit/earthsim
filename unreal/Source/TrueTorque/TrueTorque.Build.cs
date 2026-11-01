using UnrealBuildTool;
using System.IO;

public class TrueTorque : ModuleRules
{
	public TrueTorque(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		PublicDependencyModuleNames.AddRange(new string[] {
			"Core", "CoreUObject", "Engine", "InputCore", "EnhancedInput",
			"ProceduralMeshComponent", "AudioMixer", "NetCore", "PhysicsCore", "Json", "RenderCore"
		});
		// car geometry and visual damage: plain C++ shared with the command-line tests
		PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "Geom"));
		ShadowVariableWarningLevel = WarningLevel.Off;
		bUseUnity = false;
	}
}
