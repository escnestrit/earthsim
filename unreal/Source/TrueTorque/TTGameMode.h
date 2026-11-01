// Game mode (server): spawns a car for every player. Game state (everywhere): builds the test world —
// sky, sun, clouds, fog, ground, a circuit with kerbs, armco and tyre walls, a grandstand, a skid pad,
// ramps, a crash wall, barrels, a garage, trees and hills on the horizon.
#pragma once
#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "GameFramework/GameStateBase.h"
#include "GameFramework/PlayerState.h"
#include "TTGameMode.generated.h"

UCLASS()
class TRUETORQUE_API ATTGameMode : public AGameModeBase
{
	GENERATED_BODY()
public:
	ATTGameMode();
	virtual APawn* SpawnDefaultPawnFor_Implementation(AController* NewPlayer, AActor* StartSpot) override;
	virtual void StartPlay() override;
private:
	int32 Spawned = 0;
	class ATTCar* SpawnCar(const FVector& Loc, float Yaw, int32 CarIndex, const FLinearColor& Paint);
};

// Money and lap times of one player (replicated to everyone).
UCLASS()
class TRUETORQUE_API ATTPlayerState : public APlayerState
{
	GENERATED_BODY()
public:
	virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;
	UPROPERTY(Replicated) int32 Money = 3000;      // starting capital
	UPROPERTY(Replicated) float BestLap = 0.f;     // s, 0 = none yet
	UPROPERTY(Replicated) float LastLap = 0.f;
	bool Pay(int32 Amount);                        // server: false if there is not enough
};

UCLASS()
class TRUETORQUE_API ATTGameState : public AGameStateBase
{
	GENERATED_BODY()
public:
	ATTGameState();
	virtual void BeginPlay() override;
	virtual void Tick(float Dt) override;
	virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;

	// ---- time and weather (server decides, everyone renders)
	UPROPERTY(Replicated) float TimeOfDay = 14.f;  // hours
	UPROPERTY(Replicated) float Rain = 0.f;        // 0..1 how hard it rains
	UPROPERTY(Replicated) float Wetness = 0.f;     // 0..1 water on the road (lags the rain, dries slowly)
	UPROPERTY(Replicated) float TimeScale = 60.f;  // game seconds per real second
	void SkipTime(float Hours);                    // server
	void ToggleRain();                             // server
	float GripFactor(float SurfaceGrip) const;     // what the water does to a tyre on this surface
	FString WeatherText() const;

private:
	void BuildWorld();
	void ApplyEnvironment(float Dt);
	void UpdateRain(float Dt);
	AActor* Box(const FVector& Centre, const FVector& SizeM, const FRotator& Rot, class UMaterialInterface* M, bool bPhysics = false, const TCHAR* Shape = TEXT("Cube"),
	            class UPhysicalMaterial* Surface = nullptr);
	class UHierarchicalInstancedStaticMeshComponent* Instances(AActor* Owner, const TCHAR* Shape, class UMaterialInterface* M, bool bCollide);
	UPROPERTY() TArray<TObjectPtr<class UMaterialInstanceDynamic>> Mats;
	UPROPERTY() TArray<TObjectPtr<class UMaterialInstanceDynamic>> WetMats;  // ground materials that get wet
	UPROPERTY() TArray<TObjectPtr<class UPhysicalMaterial>> Surfaces;
	UPROPERTY() TObjectPtr<class ADirectionalLight> Sun;
	UPROPERTY() TObjectPtr<class ADirectionalLight> Moon;
	UPROPERTY() TObjectPtr<class ASkyLight> Sky;
	UPROPERTY() TObjectPtr<class AExponentialHeightFog> Fog;
	UPROPERTY() TObjectPtr<class APostProcessVolume> Post;
	UPROPERTY() TArray<TObjectPtr<class UPointLightComponent>> StreetLights;
	UPROPERTY() TObjectPtr<class UInstancedStaticMeshComponent> Drops;
	TArray<FVector> DropPos;
	float WeatherTimer = 120.f, RainTarget = 0.f, SkyRecapture = 0.f;
	// /Game/TT/Materials/<Name> tinted with Color (falls back to the engine's basic material)
	class UMaterialInstanceDynamic* Mat(const TCHAR* Name, const FLinearColor& C, float Rough = -1.f);
};
