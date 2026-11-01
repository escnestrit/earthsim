// Game mode (server): spawns a car for every player. Game state (everywhere): builds the test world —
// sky, sun, clouds, fog, ground, a circuit with kerbs, armco and tyre walls, a grandstand, a skid pad,
// ramps, a crash wall, barrels, a garage, trees and hills on the horizon.
#pragma once
#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "GameFramework/GameStateBase.h"
#include "TTGameMode.generated.h"

UCLASS()
class TRUETORQUE_API ATTGameMode : public AGameModeBase
{
	GENERATED_BODY()
public:
	ATTGameMode();
	virtual APawn* SpawnDefaultPawnFor_Implementation(AController* NewPlayer, AActor* StartSpot) override;
private:
	int32 Spawned = 0;
};

UCLASS()
class TRUETORQUE_API ATTGameState : public AGameStateBase
{
	GENERATED_BODY()
public:
	virtual void BeginPlay() override;
private:
	void BuildWorld();
	AActor* Box(const FVector& Centre, const FVector& SizeM, const FRotator& Rot, class UMaterialInterface* M, bool bPhysics = false, const TCHAR* Shape = TEXT("Cube"),
	            class UPhysicalMaterial* Surface = nullptr);
	class UHierarchicalInstancedStaticMeshComponent* Instances(AActor* Owner, const TCHAR* Shape, class UMaterialInterface* M, bool bCollide);
	UPROPERTY() TArray<TObjectPtr<class UMaterialInstanceDynamic>> Mats;
	UPROPERTY() TArray<TObjectPtr<class UPhysicalMaterial>> Surfaces;
	// /Game/TT/Materials/<Name> tinted with Color (falls back to the engine's basic material)
	class UMaterialInstanceDynamic* Mat(const TCHAR* Name, const FLinearColor& C, float Rough = -1.f);
};
