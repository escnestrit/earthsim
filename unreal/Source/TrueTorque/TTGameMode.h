// Game mode (server): spawns a car for every player. Game state (everywhere): builds the test world —
// sky, sun, clouds, fog, ground, a circuit, a skid pad, ramps, kerbs, a crash wall, barrels, a garage.
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
	AActor* Box(const FVector& Centre, const FVector& SizeM, const FRotator& Rot, const FLinearColor& Col, bool bPhysics = false, const TCHAR* Shape = TEXT("Cube"));
	UPROPERTY() TArray<TObjectPtr<class UMaterialInstanceDynamic>> Mats;
	class UMaterialInstanceDynamic* Mat(const FLinearColor& C);
};
