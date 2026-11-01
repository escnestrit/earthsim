// The drivable car. Physics: one Chaos rigid body (stable collisions with the world and other cars),
// raycast suspension, a slip-based tyre model, engine + clutch + gearbox + limited-slip diff, brakes
// with ABS, cooling and oil. Looks: a body made of parts (the built-in procedural sedan or imported
// modular models) whose panels dent, crease, lose paint, crack from fatigue and fall off. Workshop:
// wheel nuts with torque, straightening and refitting panels, duct tape, cold weld, fluids, engine
// rebuild, swapping tuning parts. Multiplayer: the server owns physics and damage; clients get
// replicated movement, inputs and every dent.
#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "InputActionValue.h"
#include "ttgeom.hpp"
#include "TTCar.generated.h"

class UBoxComponent;
class UStaticMeshComponent;
class UProceduralMeshComponent;
class UCameraComponent;
class USpringArmComponent;
class UTextRenderComponent;
class USpotLightComponent;
class UParticleSystemComponent;
class UTTEngineSound;
class UInputAction;
class UInputMappingContext;
class UMaterialInterface;
class UMaterialInstanceDynamic;

UENUM()
enum class ETTView : uint8 { Chase, Cockpit, Mechanic };

// commands from the driver or a mechanic standing next to the car (validated on the server)
namespace TTCmd
{
enum : uint8 { Tighten, Loosen, RepairPart, Fluids, EngineRebuild, Tape, ColdWeld, SwapVariant, Reset, ShiftUp, ShiftDown, Gearbox, Lights, WorldTime, WorldRain };
int32 Cost(uint8 Cmd, int32 Index);  // what the job costs (parts, consumables, tow)
}

// one imported car from Content/TT/Cars/cars.json (written by Tools/import_cars.py)
struct FTTCarPartDef
{
	FString Category, Group;
	TArray<FString> VariantNames;            // sorted, "stock" first
	TArray<TArray<FString>> VariantMeshes;   // static mesh object paths per variant
	bool bCanFall = false;
};
struct FTTCarDef
{
	FString Name;
	TArray<FTTCarPartDef> Parts;
};

UCLASS()
class TRUETORQUE_API ATTCar : public APawn
{
	GENERATED_BODY()
	friend class ATTHUD;
public:
	ATTCar();
	virtual void BeginPlay() override;
	virtual void Tick(float Dt) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PIC) override;
	virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;
	virtual void PossessedBy(AController* NewController) override;
	virtual void UnPossessed() override;

	static const TArray<FTTCarDef>& Catalog();

	UPROPERTY(EditAnywhere, Replicated, Category = "TrueTorque") FLinearColor PaintColor = FLinearColor(0.42f, 0.03f, 0.02f);
	UPROPERTY(EditAnywhere, Replicated, Category = "TrueTorque") int32 CarIndex = -1;  // -1: built-in sedan

	// ---- network
	UFUNCTION(Server, Unreliable) void ServerInput(float Throttle, float Brake, float Steer, float Handbrake);
	UFUNCTION(Server, Reliable) void ServerCommand(uint8 Cmd, int32 Index);
	UFUNCTION(Server, Reliable) void ServerExit();
	UFUNCTION(NetMulticast, Reliable) void MulticastDent(FVector_NetQuantize10 LocalPoint, FVector_NetQuantizeNormal LocalDir, float Depth, float Radius, float Seed, uint8 Kind);
	UFUNCTION(NetMulticast, Reliable) void MulticastRepair(int32 Part);

	// ---- used by the mechanic on foot
	void DoCommand(uint8 Cmd, int32 Index, class APlayerState* Payer = nullptr);  // server
	void ShowCommand(uint8 Cmd, int32 Index);        // local feedback (notes, sounds)
	int32 AimedWheel(const FVector& Eye, const FVector& Dir) const;
	int32 AimedPart(const FVector& Eye, const FVector& Dir) const;
	void Note(const FString& Text, const FVector& Where, float Secs = 3.f);
	float GetSpeed() const { return Speed; }
	FVector DriverDoor() const;
	FString DebugReport() const;   // F1 overlay
	FString AimReport(const FVector& Eye, const FVector& Dir) const;  // what a mechanic looking at the car would work on
	void DrawDebugLines() const;   // F2
	static bool bDebugLines;
	bool WheelNeedsWork(int32 W) const { return W >= 0 && W < 4 && ((DetachedWheels >> W) & 1 || WheelDamage[W] > 0.05f); }

private:
	// ---- components
	// physics body: convex hulls fitted to the car's own shape (front / cabin / rear)
	UPROPERTY(VisibleAnywhere) TObjectPtr<UProceduralMeshComponent> Chassis;
	UPROPERTY(VisibleAnywhere) TObjectPtr<USceneComponent> Visual;  // ground-level car space
	UPROPERTY() TArray<TObjectPtr<UProceduralMeshComponent>> PartComps;
	UPROPERTY(VisibleAnywhere) TArray<TObjectPtr<UProceduralMeshComponent>> WheelComps;
	UPROPERTY(VisibleAnywhere) TObjectPtr<USpringArmComponent> Arm;
	UPROPERTY(VisibleAnywhere) TObjectPtr<UCameraComponent> Camera;
	UPROPERTY(VisibleAnywhere) TObjectPtr<UTTEngineSound> Sound;
	UPROPERTY(VisibleAnywhere) TObjectPtr<UTextRenderComponent> DashSpeed;
	UPROPERTY(VisibleAnywhere) TObjectPtr<UTextRenderComponent> DashInfo;
	UPROPERTY(VisibleAnywhere) TObjectPtr<UTextRenderComponent> WorldNote;
	UPROPERTY(VisibleAnywhere) TObjectPtr<USpotLightComponent> HeadL;
	UPROPERTY(VisibleAnywhere) TObjectPtr<USpotLightComponent> HeadR;
	UPROPERTY() TObjectPtr<UParticleSystemComponent> Steam;
	UPROPERTY() TObjectPtr<UParticleSystemComponent> Smoke;
	UPROPERTY() TArray<TObjectPtr<UParticleSystemComponent>> TyreSmoke;
	UPROPERTY() TArray<TObjectPtr<UMaterialInstanceDynamic>> Mats;      // procedural car slots (paint, glass, trim, head, tail)
	UPROPERTY() TArray<TObjectPtr<UMaterialInstanceDynamic>> WheelMats; // tyre, rim
	UPROPERTY() TArray<TObjectPtr<UMaterialInterface>> PartMats;        // every material any part uses (GC)
	// running gear you can see: per corner disc, caliper, damper, lower arm, upper arm, tie rod / driveshaft;
	// plus engine, gearbox, radiator, exhaust, diff, tank
	UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> Mech;
	UPROPERTY() TArray<TObjectPtr<UProceduralMeshComponent>> Springs;
	UPROPERTY() TArray<TObjectPtr<UMaterialInstanceDynamic>> MechMats;

	// ---- body parts (visual deformation)
	struct FPart
	{
		ttg::Mesh Mesh;
		TArray<UMaterialInterface*> Materials;  // per section (kept alive by PartMats)
		float Health = 1.f, Fatigue = 0.f;
		bool bDirty = false, bCanFall = false, bTaped = false, bCracked = false;
		int32 Def = -1;                    // index into the car's part defs (imported cars)
		FString Name;
	};
	TArray<FPart> Parts;
	ttg::Mesh WheelMesh;
	int32 EnginePart = -1;          // the part that covers the engine (hood/bonnet)
	FTransform ImportFrame;         // mesh space -> car space for imported parts
	float CarLength = 452.f, CarWidth = 180.f;
	void BuildProcedural();
	void BuildImported(const FTTCarDef& Def);
	bool LoadPartMeshes(int32 Part, const TArray<FString>& Meshes, bool bApplyFrame);
	void CreatePartComponent(int32 Part);
	void BuildPartMesh(int32 Index, bool bCreate);
	void ApplyDent(const FVector& LocalPoint, const FVector& LocalDir, float Depth, float Radius, float Seed, uint8 Kind);

	// ---- wheels and drivetrain
	struct FWheel
	{
		FVector Mount = FVector::ZeroVector;  // chassis space, cm
		bool bFront = false, bLeft = false;
		float Omega = 0, Spin = 0;             // rad/s, rad
		float SpringLen = 22, Compression = 0.09f;
		float Load = 0, SlipRatio = 0, Slide = 0, Grip = 1, BumpSpeed = 0;
		bool bContact = false;
		FVector ContactPoint = FVector::ZeroVector, ContactNormal = FVector::UpVector;
		float LooseDistance = 0;
	};
	TArray<FWheel> Wheels;
	float WheelRadius = 33.f;      // cm
	float Speed = 0;               // m/s along the car
	float SteerAngle = 0;          // deg
	float EngineRpm = 850;
	float ShiftTimer = 0, SinceShift = 0, ReverseHold = 0;
	bool bEngineRunning = true;
	FVector LastVelocity = FVector::ZeroVector;
	float LastDentTime = -10;
	FVector LastDentPoint = FVector::ZeroVector;
	float SmokeTimer = 0;

	// replicated state
	UPROPERTY(Replicated) float RepThrottle = 0;
	UPROPERTY(Replicated) float RepBrake = 0;
	UPROPERTY(Replicated) float RepSteer = 0;
	UPROPERTY(Replicated) float RepHandbrake = 0;
	UPROPERTY(Replicated) int8 Gear = 1;
	UPROPERTY(Replicated) bool bAutomatic = true;
	UPROPERTY(Replicated) bool bLights = false;
	UPROPERTY(Replicated) float EngineHealth = 1.f;
	UPROPERTY(Replicated) float EngineTemp = 20.f;      // deg C
	UPROPERTY(Replicated) float Coolant = 1.f;          // fraction of a full system
	UPROPERTY(Replicated) float Oil = 1.f;
	UPROPERTY(Replicated) float RadiatorLeak = 0.f;     // fraction per second
	UPROPERTY(Replicated) bool bRadiatorWelded = false;
	UPROPERTY(Replicated) TArray<float> NutTorque;      // 5 per wheel, N m; < 0 = stripped thread
	UPROPERTY(Replicated) TArray<float> WheelDamage;    // 0 .. 1 bent, > 1 broken off
	UPROPERTY(ReplicatedUsing = OnRep_Variant) TArray<uint8> PartVariant;  // chosen tuning variant per part (imported cars)
	UPROPERTY(ReplicatedUsing = OnRep_Detached) TArray<uint8> PartOff;     // 1 = fallen off
	UPROPERTY(ReplicatedUsing = OnRep_Detached) uint8 DetachedWheels = 0;
	UFUNCTION() void OnRep_Detached();
	UFUNCTION() void OnRep_Variant();
	TArray<uint8> AppliedOff, AppliedVariant;
	uint8 AppliedWheels = 0;
	bool IsOff(int32 Part) const { return PartOff.IsValidIndex(Part) && PartOff[Part]; }
	const FTTCarDef* Def() const;
	void ApplyVariant(int32 Part, int32 Variant);
	void SetPartDetached(int32 Part, bool bOff, const FVector& Kick);
	void SetWheelDetached(int32 Wheel, bool bOff);

	// skid marks: dark ribbons left by sliding tyres (local, world space)
	UPROPERTY() TObjectPtr<UProceduralMeshComponent> Skids;
	UPROPERTY() TObjectPtr<UMaterialInstanceDynamic> SkidMat;
	TArray<FVector> SkidVerts;          // 4 per quad, ring buffer
	int32 SkidNext = 0;
	FVector SkidLast[4][2];
	bool bSkidOn[4] = {false, false, false, false};
	bool bSkidDirty = false;
	float SkidRebuild = 0;
	void UpdateSkids(float Dt);
	// lap timing across the start line
	float LapStart = -1.f, PrevX = 0.f;
	void UpdateLap();
	void UpdatePhysics(float Dt);
	void UpdateEngineHeat(float Dt);
	void UpdateVisuals(float Dt);
	void UpdateCamera(float Dt);
	float WheelNutAverage(int32 Wheel) const;
	void SetupWheels();
	void BuildHull();
	void BuildMechanics();
	void UpdateMechanics();
	UStaticMeshComponent* AddMech(const TCHAR* Shape, UMaterialInterface* M);
	float ShakeAmp = 0;
	bool bCoMSet = false;

	UFUNCTION() void OnChassisHit(UPrimitiveComponent* HitComp, AActor* Other, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit);

	// ---- input and views
	UPROPERTY() TObjectPtr<UInputMappingContext> Mapping;
	UPROPERTY() TMap<FName, TObjectPtr<UInputAction>> Actions;
	float InThrottle = 0, InBrake = 0, InSteer = 0, InHandbrake = 0;
	ETTView View = ETTView::Chase;
	float LookYaw = 0, LookPitch = -10, CamYaw = 0;
	float NoteTimer = 0;
	void SetupInput();
	void ApplyView();
	void Command(uint8 Cmd, int32 Index);
	bool CameraRay(FVector& Eye, FVector& Dir) const;

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
	void OnExit();
	void OnDebugHud();
	void OnDebugLines();
	void OnWorldTime();
	void OnWorldRain();
};
