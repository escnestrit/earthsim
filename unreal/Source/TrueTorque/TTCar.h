// The drivable car. Physics: one Chaos rigid body (stable collisions with the world and other cars),
// raycast suspension, a slip-based tyre model, engine + clutch + gearbox + limited-slip diff, brakes
// with ABS. Looks: a procedural body whose panels dent, crease, lose paint and fall off on impact
// (the deformation is visual, the collision shape stays rigid). Workshop: wheel nuts with torque,
// panel and engine repair, refitting fallen parts. Multiplayer: the server owns physics and damage;
// clients get replicated movement, inputs and every dent.
#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "InputActionValue.h"
#include "ttgeom.hpp"
#include "TTCar.generated.h"

class UBoxComponent;
class UProceduralMeshComponent;
class UCameraComponent;
class USpringArmComponent;
class UTextRenderComponent;
class USpotLightComponent;
class UTTEngineSound;
class UInputAction;
class UInputMappingContext;
class UMaterialInstanceDynamic;

UENUM()
enum class ETTView : uint8 { Chase, Cockpit, Mechanic };

UCLASS()
class TRUETORQUE_API ATTCar : public APawn
{
	GENERATED_BODY()
public:
	ATTCar();
	virtual void BeginPlay() override;
	virtual void Tick(float Dt) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PIC) override;
	virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;

	UPROPERTY(EditAnywhere, Replicated, Category = "TrueTorque") FLinearColor PaintColor = FLinearColor(0.42f, 0.03f, 0.02f);

	// ---- network
	UFUNCTION(Server, Unreliable) void ServerInput(float Throttle, float Brake, float Steer, float Handbrake);
	UFUNCTION(Server, Reliable) void ServerCommand(uint8 Cmd, int32 Index);
	UFUNCTION(NetMulticast, Reliable) void MulticastDent(FVector_NetQuantize10 LocalPoint, FVector_NetQuantizeNormal LocalDir, float Depth, float Radius, float Seed, uint8 Kind);
	UFUNCTION(NetMulticast, Reliable) void MulticastRepair(int32 Part);

	float GetSpeed() const { return Speed; }

private:
	// ---- components
	UPROPERTY(VisibleAnywhere) TObjectPtr<UBoxComponent> Chassis;
	UPROPERTY(VisibleAnywhere) TObjectPtr<UBoxComponent> Cabin;
	UPROPERTY(VisibleAnywhere) TObjectPtr<USceneComponent> Visual;  // ground-level car space
	UPROPERTY(VisibleAnywhere) TArray<TObjectPtr<UProceduralMeshComponent>> PartComps;
	UPROPERTY(VisibleAnywhere) TArray<TObjectPtr<UProceduralMeshComponent>> WheelComps;
	UPROPERTY(VisibleAnywhere) TObjectPtr<USpringArmComponent> Arm;
	UPROPERTY(VisibleAnywhere) TObjectPtr<UCameraComponent> Camera;
	UPROPERTY(VisibleAnywhere) TObjectPtr<UTTEngineSound> Sound;
	UPROPERTY(VisibleAnywhere) TObjectPtr<UTextRenderComponent> DashSpeed;
	UPROPERTY(VisibleAnywhere) TObjectPtr<UTextRenderComponent> DashInfo;
	UPROPERTY(VisibleAnywhere) TObjectPtr<UTextRenderComponent> WorldNote;
	UPROPERTY(VisibleAnywhere) TObjectPtr<USpotLightComponent> HeadL;
	UPROPERTY(VisibleAnywhere) TObjectPtr<USpotLightComponent> HeadR;
	UPROPERTY() TArray<TObjectPtr<UMaterialInstanceDynamic>> Mats;      // car slots (paint, glass, trim, head, tail)
	UPROPERTY() TArray<TObjectPtr<UMaterialInstanceDynamic>> WheelMats; // tyre, rim

	// ---- body parts (visual deformation)
	struct FPart
	{
		ttg::Mesh Mesh;
		float Health = 1.f;
		bool bDirty = false;
		bool bCanFall = false;
	};
	TArray<FPart> Parts;  // body, hood, boot lid, front bumper, rear bumper
	ttg::Mesh WheelMesh;
	void BuildPartMesh(int32 Index, bool bCreate);
	void ApplyDent(const FVector& LocalPoint, const FVector& LocalDir, float Depth, float Radius, float Seed, uint8 Kind);

	// ---- wheels and drivetrain
	struct FWheel
	{
		FVector Mount = FVector::ZeroVector;  // chassis space, cm
		bool bFront = false, bLeft = false;
		float Omega = 0, Spin = 0;             // rad/s, rad
		float SpringLen = 22, Compression = 0; // cm below the mount, m
		float Load = 0, SlipRatio = 0, Slide = 0, Grip = 1;
		bool bContact = false;
		FVector ContactPoint = FVector::ZeroVector, ContactNormal = FVector::UpVector;
		float LooseDistance = 0;
	};
	TArray<FWheel> Wheels;
	float Speed = 0;        // m/s along the car
	float SteerAngle = 0;   // deg
	float EngineRpm = 850;
	float ShiftTimer = 0, SinceShift = 0, ReverseHold = 0;
	bool bEngineRunning = true;
	FVector LastVelocity = FVector::ZeroVector;  // cm/s, for impact speed
	float LastDentTime = -10;
	FVector LastDentPoint = FVector::ZeroVector;

	// replicated state
	UPROPERTY(Replicated) float RepThrottle = 0;
	UPROPERTY(Replicated) float RepBrake = 0;
	UPROPERTY(Replicated) float RepSteer = 0;
	UPROPERTY(Replicated) float RepHandbrake = 0;
	UPROPERTY(Replicated) int8 Gear = 1;
	UPROPERTY(Replicated) bool bAutomatic = true;
	UPROPERTY(Replicated) bool bLights = false;
	UPROPERTY(Replicated) float EngineHealth = 1.f;
	UPROPERTY(Replicated) TArray<float> NutTorque;      // 5 per wheel, N m; < 0 = stripped thread
	UPROPERTY(Replicated) TArray<float> WheelDamage;    // 0 .. 1 bent, > 1 broken off
	UPROPERTY(ReplicatedUsing = OnRep_Detached) uint8 DetachedParts = 0;
	UPROPERTY(ReplicatedUsing = OnRep_Detached) uint8 DetachedWheels = 0;
	UFUNCTION() void OnRep_Detached();
	uint8 AppliedParts = 0, AppliedWheels = 0;
	void SetPartDetached(int32 Part, bool bOff, const FVector& Kick);
	void SetWheelDetached(int32 Wheel, bool bOff);

	void UpdatePhysics(float Dt);
	void UpdateVisuals(float Dt);
	void UpdateCamera(float Dt);
	float WheelNutAverage(int32 Wheel) const;

	UFUNCTION() void OnChassisHit(UPrimitiveComponent* HitComp, AActor* Other, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit);

	// ---- input and views
	UPROPERTY() TObjectPtr<UInputMappingContext> Mapping;
	UPROPERTY() TMap<FName, TObjectPtr<UInputAction>> Actions;
	float InThrottle = 0, InBrake = 0, InSteer = 0, InHandbrake = 0;
	ETTView View = ETTView::Chase;
	float LookYaw = 0, LookPitch = -10, CamYaw = 0;
	int32 WrenchWheel = -1;
	float NoteTimer = 0;
	void SetupInput();
	void ApplyView();
	void Note(const FString& Text, const FVector& Where, float Secs = 3.f);
	int32 AimedWheel() const;
	int32 AimedPart() const;
	void DoCommand(uint8 Cmd, int32 Index);
	void Command(uint8 Cmd, int32 Index);

	void OnThrottle(const FInputActionValue& V);
	void OnBrake(const FInputActionValue& V);
	void OnSteer(const FInputActionValue& V);
	void OnHandbrake(const FInputActionValue& V);
	void OnLook(const FInputActionValue& V);
	void OnShiftUp();
	void OnShiftDown();
	void OnGearbox();
	void OnCamera();
	void OnMechanic();
	void OnReset();
	void OnLightsKey();
	void OnUse();
	void OnLoosen();
	void OnRepairPart();
	void OnRepairEngine();
};
