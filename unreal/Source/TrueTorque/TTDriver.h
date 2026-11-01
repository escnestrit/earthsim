// The player on foot: walks around, gets into any free car (F), and is the mechanic — everything
// is done by looking at the part and using a tool, no menus.
//   LMB  torque wrench on the wheel you look at (nut by nut, clicks at 120 N m; keep going = strip it)
//   RMB  undo the nuts            R  straighten / refit the panel or wheel you look at
//   T    duct tape                V  cold weld (radiator when you look at the bonnet, else the panel)
//   G    top up coolant and oil   H  rebuild the engine      Y  fit the next tuning variant of a part
#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "InputActionValue.h"
#include "TTDriver.generated.h"

class ATTCar;
class UCameraComponent;
class UStaticMeshComponent;
class UTextRenderComponent;
class UInputAction;
class UInputMappingContext;

UCLASS()
class TRUETORQUE_API ATTDriver : public ACharacter
{
	GENERATED_BODY()
public:
	ATTDriver();
	virtual void SetupPlayerInputComponent(UInputComponent* PIC) override;

	UFUNCTION(Server, Reliable) void ServerEnter(ATTCar* Car);
	UFUNCTION(Server, Reliable) void ServerCarCommand(ATTCar* Car, uint8 Cmd, int32 Index);
	FString AimText() const;
	UFUNCTION(Server, Reliable) void ServerWorld(uint8 Cmd);
	UFUNCTION(Server, Reliable) void ServerGiveMoney(APawn* To, int32 Amount);

private:
	UPROPERTY(VisibleAnywhere) TObjectPtr<UCameraComponent> Eyes;
	UPROPERTY(VisibleAnywhere) TObjectPtr<UStaticMeshComponent> BodyMesh;
	UPROPERTY(VisibleAnywhere) TObjectPtr<UStaticMeshComponent> HeadMesh;
	UPROPERTY(VisibleAnywhere) TObjectPtr<UTextRenderComponent> Dot;  // where the hands go (a mark in the world, not a HUD)
	UPROPERTY() TObjectPtr<UInputMappingContext> Mapping;
	UPROPERTY() TMap<FName, TObjectPtr<UInputAction>> Actions;

	ATTCar* LookedAtCar(FVector& Eye, FVector& Dir) const;
	void Tool(uint8 Cmd);

	void OnMove(const FInputActionValue& V);
	void OnLook(const FInputActionValue& V);
	void OnJump();
	void OnEnter();
	void OnTighten();
	void OnLoosen();
	void OnRepair();
	void OnTape();
	void OnWeld();
	void OnFluids();
	void OnEngine();
	void OnSwap();
	void OnDebugHud();
	void OnWorldTime();
	void OnWorldRain();
	void OnGiveMoney();
};
