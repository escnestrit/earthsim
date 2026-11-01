// The drivable car: wraps the soft-body simulation (Sim/), draws the deformed body, feeds it the
// world (traces), plays procedural sound, shows information only in the world (dashboard, wrench dial),
// and replicates its whole deformed state to clients.
#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "InputActionValue.h"
#include <memory>
#include <vector>
#include "vehicle.hpp"
#include "TTVehicle.generated.h"

class UProceduralMeshComponent;
class UStaticMeshComponent;
class UCameraComponent;
class USpringArmComponent;
class UTextRenderComponent;
class UTTEngineSound;
class UInputAction;
class UInputMappingContext;
class UMaterialInstanceDynamic;

UENUM()
enum class ETTView : uint8 { Chase, Cockpit, Mechanic };

UCLASS()
class TRUETORQUE_API ATTVehicle : public APawn
{
	GENERATED_BODY()
public:
	ATTVehicle();
	virtual ~ATTVehicle() override;
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void Tick(float Dt) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PIC) override;

	UPROPERTY(EditAnywhere, Category = "TrueTorque") FLinearColor PaintColor = FLinearColor(0.42f, 0.03f, 0.02f);

	// network: owning client -> server input, server -> everyone deformed state
	UFUNCTION(Server, Unreliable) void ServerInput(float Throttle, float Brake, float Steer, float Handbrake, int8 Shift);
	UFUNCTION(Server, Reliable) void ServerWorkshop(int32 Action, int32 Index, float Value);
	UFUNCTION(NetMulticast, Unreliable) void MulticastState(const TArray<uint8>& Bytes);
	UFUNCTION(NetMulticast, Reliable) void MulticastImpact(float Strength);

	tt::Vehicle* GetSim() const { return Sim.get(); }

private:
	std::unique_ptr<tt::Vehicle> Sim;
	// rendering
	UPROPERTY() TObjectPtr<UProceduralMeshComponent> Body;
	UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> WheelMeshes;
	UPROPERTY() TObjectPtr<USpringArmComponent> Arm;
	UPROPERTY() TObjectPtr<UCameraComponent> Camera;
	UPROPERTY() TObjectPtr<UTTEngineSound> Sound;
	UPROPERTY() TObjectPtr<UTextRenderComponent> DashSpeed;
	UPROPERTY() TObjectPtr<UTextRenderComponent> DashInfo;
	UPROPERTY() TObjectPtr<UTextRenderComponent> WorldNote;
	UPROPERTY() TArray<TObjectPtr<UMaterialInstanceDynamic>> Mats;
	TArray<TArray<int32>> SectionQuads;   // per material: skin quad indices
	TArray<float> WheelSpin;
	// input
	UPROPERTY() TObjectPtr<UInputMappingContext> Mapping;
	UPROPERTY() TMap<FName, TObjectPtr<UInputAction>> Actions;
	float InThrottle = 0, InBrake = 0, InSteer = 0, InHandbrake = 0;
	int8 InShift = 0;
	ETTView View = ETTView::Chase;
	float LookYaw = 0, LookPitch = -12;
	int32 WrenchWheel = -1, WrenchNut = 0;
	float NoteTimer = 0;
	// network
	float SendTimer = 0;
	uint32 Tick_ = 0;
	TArray<uint8> PendingState;
	bool bHavePending = false;
	std::vector<double> RenderFrom, RenderTo;  // client interpolation of node positions
	float InterpT = 1;
	double LastPeak = 0;

	void BuildMesh();
	void UpdateMesh();
	void UpdateWheels(float Dt);
	void UpdateContacts(float Dt);
	void UpdateDash();
	void Note(const FString& Text, const FVector& Where, float Secs = 3.f);
	void SetupInput();
	void ApplyView();
	tt::GroundHit GroundAt(double X, double Y) const;
	int32 AimedWheel() const;
	int32 AimedPart() const;

	// cached ground under each wheel (refreshed every frame by traces)
	struct FGroundCache { double X = 0, Y = 0, H = -1e9, Nx = 0, Ny = 0, Nz = 1, Friction = 1; int32 Surface = 0; };
	TArray<FGroundCache> Ground;

	void OnThrottle(const FInputActionValue& V);
	void OnBrake(const FInputActionValue& V);
	void OnSteer(const FInputActionValue& V);
	void OnHandbrake(const FInputActionValue& V);
	void OnLook(const FInputActionValue& V);
	void OnShiftUp();
	void OnShiftDown();
	void OnCamera();
	void OnMechanic();
	void OnReset();
	void OnUse();      // mechanic: tighten (LMB)
	void OnLoosen();   // mechanic: loosen (RMB)
	void OnReplace();  // mechanic: fit a new part (E)
	void OnRefill();   // mechanic: fluids (F)
	void OnGearbox();  // automatic / manual
	void DoWorkshop(int32 Action, int32 Index, float Value);
};
