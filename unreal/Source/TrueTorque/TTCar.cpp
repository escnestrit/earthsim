#include "TTCar.h"
#include "TTEngineSound.h"
#include "Camera/CameraComponent.h"
#include "Camera/PlayerCameraManager.h"
#include "Components/BoxComponent.h"
#include "Components/SpotLightComponent.h"
#include "Components/TextRenderComponent.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "Engine/LocalPlayer.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/SpringArmComponent.h"
#include "InputAction.h"
#include "InputMappingContext.h"
#include "InputModifiers.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Net/UnrealNetwork.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "ProceduralMeshComponent.h"

namespace
{
// ---- car constants (cm unless noted)
constexpr float ZC = 72.f;           // chassis origin height above the ground at ride height
constexpr float SuspMax = 22.f;      // wheel centre travel below the mount
constexpr float MountZ = -26.f;      // mount height in chassis space
constexpr float WheelR = 33.f;
constexpr float MassKg = 1350.f;
constexpr float SpringK = 36000.f;   // N/m
constexpr float DampBump = 3000.f, DampRebound = 4600.f;  // N s/m
constexpr float AntiRollF = 14000.f, AntiRollR = 9000.f;  // N/m
constexpr float WheelInertia = 1.3f; // kg m^2
constexpr float NutSpec = 120.f;     // N m
const float GearRatio[6] = {3.45f, 2.20f, 1.55f, 1.18f, 0.94f, 0.78f};
constexpr float FinalDrive = 3.7f, ReverseRatio = 3.3f;
constexpr float IdleRpm = 850.f, RedLine = 6900.f;

float EngineTorque(float Rpm)
{
	if (Rpm < 1000.f) return 190.f;
	if (Rpm < 4400.f) return FMath::Lerp(230.f, 310.f, (Rpm - 1000.f) / 3400.f);
	return FMath::Lerp(310.f, 255.f, FMath::Clamp((Rpm - 4400.f) / 2500.f, 0.f, 1.f));
}

// combined-slip force curve: rises to 1 at s = 1 (peak grip), settles at ~0.78 when sliding
float SlipCurve(float S)
{
	if (S <= 1.f) return S * (2.f - S);
	return 1.f - 0.22f * (1.f - FMath::Exp(-(S - 1.f) * 1.5f));
}

FVector ToF(const ttg::V& v) { return FVector(v.x, v.y, v.z); }
ttg::V ToG(const FVector& v) { return ttg::V(v.X, v.Y, v.Z); }

UMaterialInstanceDynamic* MakeMat(UObject* Outer, const TCHAR* Name, const FLinearColor& Col)
{
	UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr, *FString::Printf(TEXT("/Game/TT/Materials/%s.%s"), Name, Name));
	if (!Base) Base = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	UMaterialInstanceDynamic* M = UMaterialInstanceDynamic::Create(Base, Outer);
	M->SetVectorParameterValue(TEXT("Color"), Col);
	return M;
}

enum ECmd : uint8 { CmdTighten, CmdLoosen, CmdRepairPart, CmdRepairEngine, CmdReset, CmdShiftUp, CmdShiftDown, CmdGearbox, CmdLights };
enum EPart : int32 { PartBody, PartHood, PartTrunk, PartBumperF, PartBumperR, NumParts };
}  // namespace

ATTCar::ATTCar()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;
	bReplicates = true;
	SetReplicateMovement(true);
	SetNetUpdateFrequency(60.f);
	bAlwaysRelevant = true;

	Chassis = CreateDefaultSubobject<UBoxComponent>(TEXT("Chassis"));
	RootComponent = Chassis;
	Chassis->SetBoxExtent(FVector(222.f, 84.f, 48.f));
	Chassis->SetCollisionProfileName(TEXT("Vehicle"));
	Chassis->SetSimulatePhysics(true);
	Chassis->SetNotifyRigidBodyCollision(true);
	Chassis->BodyInstance.bUseCCD = true;
	Chassis->SetHiddenInGame(true);

	Cabin = CreateDefaultSubobject<UBoxComponent>(TEXT("Cabin"));
	Cabin->SetupAttachment(Chassis);
	Cabin->SetRelativeLocation(FVector(-30.f, 0.f, 56.f));
	Cabin->SetBoxExtent(FVector(70.f, 66.f, 12.f));
	Cabin->SetCollisionProfileName(TEXT("Vehicle"));
	Cabin->SetNotifyRigidBodyCollision(true);
	Cabin->SetHiddenInGame(true);

	Visual = CreateDefaultSubobject<USceneComponent>(TEXT("Visual"));
	Visual->SetupAttachment(Chassis);
	Visual->SetRelativeLocation(FVector(0.f, 0.f, -ZC));

	const TCHAR* PartNames[NumParts] = {TEXT("Body"), TEXT("Hood"), TEXT("BootLid"), TEXT("BumperFront"), TEXT("BumperRear")};
	for (int32 i = 0; i < NumParts; i++)
	{
		UProceduralMeshComponent* P = CreateDefaultSubobject<UProceduralMeshComponent>(PartNames[i]);
		P->SetupAttachment(Visual);
		P->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		P->bUseComplexAsSimpleCollision = false;
		P->SetCastShadow(true);
		PartComps.Add(P);
	}
	const TCHAR* WheelNames[4] = {TEXT("WheelFL"), TEXT("WheelFR"), TEXT("WheelRL"), TEXT("WheelRR")};
	for (int32 i = 0; i < 4; i++)
	{
		UProceduralMeshComponent* W = CreateDefaultSubobject<UProceduralMeshComponent>(WheelNames[i]);
		W->SetupAttachment(Visual);
		W->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		W->bUseComplexAsSimpleCollision = false;
		WheelComps.Add(W);
	}

	Arm = CreateDefaultSubobject<USpringArmComponent>(TEXT("Arm"));
	Arm->SetupAttachment(Chassis);
	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	Camera->SetupAttachment(Arm);
	Camera->PostProcessSettings.bOverride_MotionBlurAmount = true;
	Camera->PostProcessSettings.MotionBlurAmount = 0.3f;

	Sound = CreateDefaultSubobject<UTTEngineSound>(TEXT("Sound"));
	Sound->SetupAttachment(Chassis);

	auto MakeText = [this](const TCHAR* Name, float Size, FColor Col) {
		UTextRenderComponent* T = CreateDefaultSubobject<UTextRenderComponent>(Name);
		T->SetupAttachment(Visual);
		T->SetHorizontalAlignment(EHTA_Center);
		T->SetWorldSize(Size);
		T->SetTextRenderColor(Col);
		T->SetCastShadow(false);
		return T;
	};
	DashSpeed = MakeText(TEXT("DashSpeed"), 4.f, FColor(255, 214, 150));
	DashSpeed->SetRelativeLocation(FVector(84.f, -36.f, 99.f));
	DashSpeed->SetRelativeRotation(FRotator(20.f, 180.f, 0.f));
	DashInfo = MakeText(TEXT("DashInfo"), 2.f, FColor(225, 225, 225));
	DashInfo->SetRelativeLocation(FVector(84.f, -36.f, 95.f));
	DashInfo->SetRelativeRotation(FRotator(20.f, 180.f, 0.f));
	WorldNote = MakeText(TEXT("WorldNote"), 9.f, FColor::White);
	WorldNote->SetUsingAbsoluteLocation(true);
	WorldNote->SetUsingAbsoluteRotation(true);
	WorldNote->SetVisibility(false);

	auto MakeLamp = [this](const TCHAR* Name, float Y) {
		USpotLightComponent* L = CreateDefaultSubobject<USpotLightComponent>(Name);
		L->SetupAttachment(Visual);
		L->SetRelativeLocation(FVector(212.f, Y, 70.f));
		L->SetRelativeRotation(FRotator(-4.f, 0.f, 0.f));
		L->SetIntensityUnits(ELightUnits::Candelas);
		L->SetIntensity(9000.f);
		L->SetAttenuationRadius(7000.f);
		L->SetInnerConeAngle(12.f);
		L->SetOuterConeAngle(32.f);
		L->SetLightColor(FLinearColor(1.f, 0.95f, 0.85f));
		L->SetVisibility(false);
		return L;
	};
	HeadL = MakeLamp(TEXT("HeadL"), -58.f);
	HeadR = MakeLamp(TEXT("HeadR"), 58.f);

	AutoPossessPlayer = EAutoReceiveInput::Disabled;
	NutTorque.Init(NutSpec, 20);
	WheelDamage.Init(0.f, 4);
}

void ATTCar::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& Out) const
{
	Super::GetLifetimeReplicatedProps(Out);
	DOREPLIFETIME(ATTCar, PaintColor);
	DOREPLIFETIME(ATTCar, RepThrottle);
	DOREPLIFETIME(ATTCar, RepBrake);
	DOREPLIFETIME(ATTCar, RepSteer);
	DOREPLIFETIME(ATTCar, RepHandbrake);
	DOREPLIFETIME(ATTCar, Gear);
	DOREPLIFETIME(ATTCar, bAutomatic);
	DOREPLIFETIME(ATTCar, bLights);
	DOREPLIFETIME(ATTCar, EngineHealth);
	DOREPLIFETIME(ATTCar, NutTorque);
	DOREPLIFETIME(ATTCar, WheelDamage);
	DOREPLIFETIME(ATTCar, DetachedParts);
	DOREPLIFETIME(ATTCar, DetachedWheels);
}

// ------------------------------------------------------------------ setup

void ATTCar::BeginPlay()
{
	Super::BeginPlay();
	// rigid body: mass, low centre of mass, a dead (non-bouncy) shell
	Chassis->SetMassOverrideInKg(NAME_None, MassKg, true);
	Cabin->SetMassOverrideInKg(NAME_None, 1.f, true);
	Chassis->SetCenterOfMass(FVector(8.f, 0.f, -16.f));
	Chassis->SetLinearDamping(0.01f);
	Chassis->SetAngularDamping(0.15f);
	UPhysicalMaterial* Shell = NewObject<UPhysicalMaterial>(this);
	Shell->Friction = 0.35f;
	Shell->Restitution = 0.05f;
	Shell->bOverrideRestitutionCombineMode = true;
	Shell->RestitutionCombineMode = EFrictionCombineMode::Min;
	Chassis->SetPhysMaterialOverride(Shell);
	Cabin->SetPhysMaterialOverride(Shell);
	Chassis->OnComponentHit.AddDynamic(this, &ATTCar::OnChassisHit);
	Cabin->OnComponentHit.AddDynamic(this, &ATTCar::OnChassisHit);

	// materials
	Mats.Add(MakeMat(this, TEXT("M_CarPaint"), PaintColor));
	Mats.Add(MakeMat(this, TEXT("M_Glass"), FLinearColor(0.01f, 0.012f, 0.015f)));
	Mats.Add(MakeMat(this, TEXT("M_Trim"), FLinearColor(0.02f, 0.02f, 0.02f)));
	Mats.Add(MakeMat(this, TEXT("M_Light"), FLinearColor(1.f, 0.96f, 0.88f)));
	Mats.Add(MakeMat(this, TEXT("M_Light"), FLinearColor(0.9f, 0.05f, 0.02f)));
	WheelMats.Add(MakeMat(this, TEXT("M_Tyre"), FLinearColor(0.025f, 0.025f, 0.025f)));
	WheelMats.Add(MakeMat(this, TEXT("M_Rim"), FLinearColor(0.7f, 0.7f, 0.72f)));

	// geometry
	ttg::CarGeom G;
	ttg::build_car(G);
	Parts.SetNum(NumParts);
	Parts[PartBody].Mesh = MoveTemp(G.body);
	Parts[PartHood].Mesh = MoveTemp(G.hood);
	Parts[PartTrunk].Mesh = MoveTemp(G.trunk);
	Parts[PartBumperF].Mesh = MoveTemp(G.bumper_f);
	Parts[PartBumperR].Mesh = MoveTemp(G.bumper_r);
	for (int32 i = PartHood; i < NumParts; i++) Parts[i].bCanFall = true;
	for (int32 i = 0; i < NumParts; i++) BuildPartMesh(i, true);
	ttg::build_wheel(WheelMesh);

	// wheels: corners, mesh, and a convex hull so a wheel that comes off can roll away
	Wheels.SetNum(4);
	for (int32 i = 0; i < 4; i++)
	{
		FWheel& W = Wheels[i];
		W.bFront = i < 2;
		W.bLeft = (i % 2) == 0;
		W.Mount = FVector(W.bFront ? ttg::WHEELBASE_HALF : -ttg::WHEELBASE_HALF, W.bLeft ? -ttg::TRACK_HALF : ttg::TRACK_HALF, MountZ);
		UProceduralMeshComponent* C = WheelComps[i];
		TArray<FVector> V[2], N[2];
		TArray<int32> I[2];
		TArray<FVector2D> UV[2];
		TArray<FLinearColor> Col[2];
		TArray<FProcMeshTangent> Tan[2];
		std::vector<ttg::V> PN;
		WheelMesh.point_normals(PN);
		for (const ttg::Tri& T : WheelMesh.tris)
		{
			const int32 S = T.slot;
			for (int32 c : {T.a, T.b, T.c})
			{
				I[S].Add(V[S].Num());
				V[S].Add(ToF(WheelMesh.base[c]));
				N[S].Add(ToF(PN[c]));
				UV[S].Add(FVector2D(WheelMesh.base[c].x / 66.0, WheelMesh.base[c].z / 66.0));
				Col[S].Add(FLinearColor(0, 0, 0, 1));
				Tan[S].Add(FProcMeshTangent(FVector(0, 1, 0), false));
			}
		}
		for (int32 S = 0; S < 2; S++)
		{
			C->CreateMeshSection_LinearColor(S, V[S], I[S], N[S], UV[S], Col[S], Tan[S], false);
			C->SetMaterial(S, WheelMats[S]);
		}
		TArray<FVector> Hull;
		for (int32 k = 0; k < 16; k++)
		{
			const double a = 2 * PI * k / 16;
			for (double y : {-11.0, 11.0}) Hull.Add(FVector(WheelR * FMath::Cos(a), y, WheelR * FMath::Sin(a)));
		}
		C->AddCollisionConvexMesh(Hull);
		if (W.bLeft) C->SetRelativeScale3D(FVector(1, -1, 1));
	}
	ApplyView();
}

void ATTCar::BuildPartMesh(int32 Index, bool bCreate)
{
	FPart& P = Parts[Index];
	UProceduralMeshComponent* C = PartComps[Index];
	const int32 NS = ttg::NUM_SLOTS;
	TArray<TArray<FVector>> V, N;
	TArray<TArray<int32>> I;
	TArray<TArray<FVector2D>> UV;
	TArray<TArray<FLinearColor>> Col;
	TArray<TArray<FProcMeshTangent>> Tan;
	V.SetNum(NS), N.SetNum(NS), I.SetNum(NS), UV.SetNum(NS), Col.SetNum(NS), Tan.SetNum(NS);
	std::vector<ttg::V> PN;
	P.Mesh.point_normals(PN);
	for (const ttg::Tri& T : P.Mesh.tris)
	{
		const int32 S = T.slot;
		const ttg::V A = P.Mesh.p(T.a), B = P.Mesh.p(T.b), Cc = P.Mesh.p(T.c);
		const FVector Face = ToF(ttg::norm(ttg::cross(Cc - A, B - A)));
		for (int32 c : {T.a, T.b, T.c})
		{
			const FVector Nrm = T.smooth ? ToF(PN[c]) : Face;
			FVector Tx = FVector::CrossProduct(Nrm, FVector::UpVector);
			if (Tx.SizeSquared() < 0.01) Tx = FVector::ForwardVector;
			I[S].Add(V[S].Num());
			V[S].Add(ToF(P.Mesh.p(c)));
			N[S].Add(Nrm);
			UV[S].Add(FVector2D(P.Mesh.base[c].x / 100.0, P.Mesh.base[c].z / 100.0));
			// R: paint gone, G: how creased the metal is (the material darkens folds)
			Col[S].Add(FLinearColor(P.Mesh.scrape[c], FMath::Min(1.f, float(ttg::len(P.Mesh.off[c])) / 15.f), 0.f, 1.f));
			Tan[S].Add(FProcMeshTangent(Tx.GetSafeNormal(), false));
		}
	}
	for (int32 S = 0; S < NS; S++)
	{
		if (bCreate)
		{
			C->CreateMeshSection_LinearColor(S, V[S], I[S], N[S], UV[S], Col[S], Tan[S], false);
			C->SetMaterial(S, Mats[S]);
		}
		else if (V[S].Num() > 0)
			C->UpdateMeshSection_LinearColor(S, V[S], N[S], UV[S], Col[S], Tan[S]);
	}
	if (bCreate && P.bCanFall)
	{
		// convex hull for when it falls off (the box around its points)
		ttg::V Lo, Hi;
		P.Mesh.bounds(Lo, Hi);
		TArray<FVector> Hull;
		for (int32 k = 0; k < 8; k++) Hull.Add(FVector((k & 1) ? Hi.x : Lo.x, (k & 2) ? Hi.y : Lo.y, (k & 4) ? Hi.z : Lo.z));
		C->AddCollisionConvexMesh(Hull);
	}
	P.bDirty = false;
}

// ------------------------------------------------------------------ tick

void ATTCar::Tick(float Dt)
{
	Super::Tick(Dt);
	if (Parts.Num() == 0) return;
	Dt = FMath::Min(Dt, 1.f / 30.f);
	if (IsLocallyControlled())
	{
		if (HasAuthority()) RepThrottle = InThrottle, RepBrake = InBrake, RepSteer = InSteer, RepHandbrake = InHandbrake;
		else
		{
			ServerInput(InThrottle, InBrake, InSteer, InHandbrake);
			// drive our own copy with fresh input; the server corrects the body
			RepThrottle = InThrottle, RepBrake = InBrake, RepSteer = InSteer, RepHandbrake = InHandbrake;
		}
	}
	UpdatePhysics(Dt);
	UpdateVisuals(Dt);
	UpdateCamera(Dt);
	LastVelocity = Chassis->GetPhysicsLinearVelocity();
}

void ATTCar::ServerInput_Implementation(float Throttle, float Brake, float Steer, float Handbrake)
{
	RepThrottle = FMath::Clamp(Throttle, 0.f, 1.f);
	RepBrake = FMath::Clamp(Brake, 0.f, 1.f);
	RepSteer = FMath::Clamp(Steer, -1.f, 1.f);
	RepHandbrake = FMath::Clamp(Handbrake, 0.f, 1.f);
}

float ATTCar::WheelNutAverage(int32 Wheel) const
{
	float S = 0;
	for (int32 k = 0; k < 5; k++) S += FMath::Max(0.f, NutTorque[Wheel * 5 + k]);
	return S / 5.f;
}

void ATTCar::UpdatePhysics(float Dt)
{
	if (!Chassis->IsSimulatingPhysics()) return;
	const FTransform T = Chassis->GetComponentTransform();
	const FVector Fwd = T.GetUnitAxis(EAxis::X), Up = T.GetUnitAxis(EAxis::Z);
	const FVector Vel = Chassis->GetPhysicsLinearVelocity() / 100.0;  // m/s
	Speed = float(FVector::DotProduct(Vel, Fwd));
	const bool bAuth = HasAuthority();

	// ---- controls (automatic: brake at a standstill engages reverse, pedals swap in reverse)
	float Thr = RepThrottle, Brk = RepBrake;
	if (bAutomatic)
	{
		if (Gear < 0)
		{
			Thr = RepBrake, Brk = RepThrottle;
			if (bAuth && Speed > -0.5f && RepThrottle > 0.1f) Gear = 1, ShiftTimer = 0.15f;
		}
		else if (bAuth)
		{
			if (Speed < 0.5f && RepBrake > 0.1f && RepThrottle < 0.05f) ReverseHold += Dt;
			else ReverseHold = 0;
			if (ReverseHold > 0.35f) Gear = -1, ReverseHold = 0, ShiftTimer = 0.15f;
			if (Gear == 0) Gear = 1;
		}
	}
	const float MaxSteer = FMath::Lerp(34.f, 9.f, FMath::Clamp(FMath::Abs(Speed) / 40.f, 0.f, 1.f));
	SteerAngle = FMath::FInterpTo(SteerAngle, RepSteer * MaxSteer, Dt, 7.f);

	// ---- engine, clutch, gearbox
	ShiftTimer -= Dt;
	SinceShift += Dt;
	if (EngineHealth <= 0.05f) bEngineRunning = false;
	const float Ratio = Gear > 0 ? GearRatio[Gear - 1] * FinalDrive : Gear < 0 ? -ReverseRatio * FinalDrive : 0.f;
	float DrivenOmega = 0;
	int32 NDriven = 0;
	for (int32 i = 2; i < 4; i++)
		if (!(DetachedWheels & (1 << i))) DrivenOmega += Wheels[i].Omega, NDriven++;
	if (NDriven) DrivenOmega /= NDriven;
	const float WheelRpm = DrivenOmega * Ratio * 60.f / (2.f * PI);
	float EngineT = 0;
	if (!bEngineRunning)
		EngineRpm = FMath::FInterpTo(EngineRpm, 0.f, Dt, 3.f);
	else if (Ratio == 0.f || ShiftTimer > 0.f)
		EngineRpm = FMath::FInterpTo(EngineRpm, IdleRpm + Thr * 6000.f, Dt, 5.f);
	else if (WheelRpm < 1300.f)
	{
		// clutch slipping: launch / creep
		EngineRpm = FMath::FInterpTo(EngineRpm, FMath::Max(1300.f, IdleRpm + Thr * 2800.f), Dt, 6.f);
		EngineT = Thr > 0.02f ? EngineTorque(EngineRpm) * Thr : (bAutomatic ? 30.f : 0.f);
	}
	else
	{
		EngineRpm = WheelRpm;
		EngineT = Thr > 0.02f ? EngineTorque(EngineRpm) * Thr : -(20.f + EngineRpm * 0.006f);
	}
	if (EngineRpm > RedLine) EngineT = FMath::Min(EngineT, 0.f);
	EngineRpm = FMath::Clamp(EngineRpm, 0.f, 7200.f);
	EngineT *= 0.35f + 0.65f * FMath::Clamp(EngineHealth, 0.f, 1.f);
	if (bAuth && bAutomatic && Gear >= 1 && ShiftTimer <= 0.f)
	{
		if (EngineRpm > 6300.f && Gear < 6) Gear++, ShiftTimer = 0.2f, SinceShift = 0;
		else if (EngineRpm < 2500.f && Gear > 1 && SinceShift > 1.f) Gear--, ShiftTimer = 0.2f, SinceShift = 0;
	}
	const float AxleT = EngineT * Ratio * 0.9f;
	// limited-slip: torque moves from the faster rear wheel to the slower one
	const float Lock = FMath::Clamp((Wheels[2].Omega - Wheels[3].Omega) * 60.f, -250.f, 250.f);
	float DriveT[4] = {0, 0, AxleT * 0.5f - Lock, AxleT * 0.5f + Lock};
	if (NDriven == 1) DriveT[2] = (DetachedWheels & 4) ? 0 : AxleT, DriveT[3] = (DetachedWheels & 8) ? 0 : AxleT;

	// ---- suspension
	FCollisionQueryParams Q(SCENE_QUERY_STAT(TTWheel), false, this);
	Q.bReturnPhysicalMaterial = true;
	const float RayLen = SuspMax + WheelR;
	for (int32 i = 0; i < 4; i++)
	{
		FWheel& W = Wheels[i];
		W.bContact = false;
		W.Load = 0;
		if (DetachedWheels & (1 << i)) continue;
		const FVector M = T.TransformPosition(W.Mount);
		FHitResult H;
		const bool bHit = GetWorld()->LineTraceSingleByChannel(H, M, M - Up * RayLen, ECC_Visibility, Q);
		W.SpringLen = bHit ? FMath::Clamp(float(H.Distance) - WheelR, 0.f, SuspMax) : SuspMax;
		const float X = (SuspMax - W.SpringLen) / 100.f;
		const float Xdot = (X - W.Compression) / Dt;
		W.Compression = X;
		if (!bHit) continue;
		W.bContact = true;
		W.ContactPoint = H.ImpactPoint;
		W.ContactNormal = H.ImpactNormal;
		W.Grip = 1.f;
		if (const UPhysicalMaterial* PM = H.PhysMaterial.Get()) W.Grip = FMath::Clamp(PM->Friction / 0.7f, 0.35f, 1.2f);
		float F = SpringK * X + (Xdot > 0 ? DampBump : DampRebound) * Xdot;
		if (X > 0.20f) F += 200000.f * (X - 0.20f);
		W.Load = F;
	}
	for (int32 Axle = 0; Axle < 2; Axle++)
	{
		FWheel& L = Wheels[Axle * 2];
		FWheel& R = Wheels[Axle * 2 + 1];
		const float Bar = (Axle == 0 ? AntiRollF : AntiRollR) * (L.Compression - R.Compression);
		if (L.bContact) L.Load += Bar;
		if (R.bContact) R.Load -= Bar;
	}

	// ---- tyres
	const float SteerRad = FMath::DegreesToRadians(SteerAngle);
	for (int32 i = 0; i < 4; i++)
	{
		FWheel& W = Wheels[i];
		if (DetachedWheels & (1 << i)) continue;
		// bent suspension: toe out, less grip; loose nuts: wobble
		const float Bend = FMath::Clamp(WheelDamage[i], 0.f, 1.f);
		const float Loose = 1.f - FMath::Clamp(WheelNutAverage(i) / NutSpec, 0.f, 1.f);
		const float Toe = FMath::DegreesToRadians(Bend * 4.f * (W.bLeft ? -1.f : 1.f) + Loose * 3.f * FMath::Sin(W.Spin));
		const float A = (W.bFront ? SteerRad : 0.f) + Toe;
		float Brake = Brk * (W.bFront ? 2400.f : 1400.f);
		if (Brk > 0.f && FMath::Abs(Speed) > 3.f && W.SlipRatio < -0.18f) Brake *= 0.3f;  // ABS
		if (!W.bFront) Brake += RepHandbrake * 3500.f;
		if (!W.bContact)
		{
			W.Omega += DriveT[i] / WheelInertia * Dt;
			const float dB = Brake / WheelInertia * Dt;
			W.Omega = FMath::Abs(W.Omega) <= dB ? 0.f : W.Omega - FMath::Sign(W.Omega) * dB;
			W.Omega *= 1.f - 0.4f * Dt;
			W.Slide = 0;
			continue;
		}
		const float Fz = FMath::Clamp(W.Load, 0.f, 15000.f);
		Chassis->AddForceAtLocation(Up * (Fz * 100.f), T.TransformPosition(W.Mount));
		const FVector N = W.ContactNormal;
		FVector WF = T.TransformVectorNoScale(FVector(FMath::Cos(A), FMath::Sin(A), 0.f));
		FVector WR = T.TransformVectorNoScale(FVector(-FMath::Sin(A), FMath::Cos(A), 0.f));
		WF = (WF - N * FVector::DotProduct(WF, N)).GetSafeNormal();
		WR = (WR - N * FVector::DotProduct(WR, N)).GetSafeNormal();
		const FVector Vc = Chassis->GetPhysicsLinearVelocityAtPoint(W.ContactPoint) / 100.0;
		const float Vx = float(FVector::DotProduct(Vc, WF)), Vy = float(FVector::DotProduct(Vc, WR));
		const float Mu = 1.05f * W.Grip * (1.f - 0.25f * Bend);
		const float Den = FMath::Max(FMath::Abs(Vx), 3.f);
		// the wheel spins much faster than the body moves: integrate it in small steps
		const int32 Sub = 8;
		const float H = Dt / Sub;
		float SumFx = 0, SumFy = 0, Smax = 0;
		for (int32 s = 0; s < Sub; s++)
		{
			const float Sr = (W.Omega * WheelR / 100.f - Vx) / Den;
			const float Sx = Sr / 0.11f, Sy = (Vy / Den) / 0.15f;
			const float S = FMath::Sqrt(Sx * Sx + Sy * Sy);
			const float Fm = Mu * Fz * SlipCurve(S);
			const float Fx = S > 1e-5f ? Fm * Sx / S : 0.f, Fy = S > 1e-5f ? -Fm * Sy / S : 0.f;
			W.Omega += (DriveT[i] - Fx * WheelR / 100.f) / WheelInertia * H;
			const float dB = Brake / WheelInertia * H;
			W.Omega = FMath::Abs(W.Omega) <= dB ? 0.f : W.Omega - FMath::Sign(W.Omega) * dB;
			SumFx += Fx, SumFy += Fy, Smax = FMath::Max(Smax, S);
			W.SlipRatio = Sr;
		}
		float Fx = SumFx / Sub, Fy = SumFy / Sub;
		if (FMath::Abs(Vx) > 0.1f) Fx -= 0.012f * Fz * FMath::Sign(Vx);  // rolling resistance
		W.Slide = Smax;
		Chassis->AddForceAtLocation((WF * Fx + WR * Fy) * 100.f, W.ContactPoint + Up * 8.0);
		// loose wheel: the nuts back off further with every metre, then the wheel leaves
		if (bAuth && Loose > 0.3f)
		{
			for (int32 k = 0; k < 5; k++)
				if (NutTorque[i * 5 + k] > 0.f) NutTorque[i * 5 + k] = FMath::Max(0.f, NutTorque[i * 5 + k] - FMath::Abs(Vx) * Dt * 0.08f);
			if (WheelNutAverage(i) < 10.f) W.LooseDistance += FMath::Abs(Vx) * Dt;
			if (W.LooseDistance > 150.f) SetWheelDetached(i, true), W.LooseDistance = 0;
		}
	}
	// ---- air
	const float V2 = float(Vel.SizeSquared());
	Chassis->AddForce(-Vel.GetSafeNormal() * (0.38f * V2 * 100.f) - Up * (0.12f * V2 * 100.f));
}

// ------------------------------------------------------------------ damage

void ATTCar::OnChassisHit(UPrimitiveComponent* HitComp, AActor* Other, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit)
{
	if (!HasAuthority() || Parts.Num() == 0) return;
	const FVector P = Hit.ImpactPoint, Nrm = Hit.ImpactNormal;  // normal points from the obstacle into us
	FVector OtherV = FVector::ZeroVector;
	if (OtherComp && OtherComp->IsSimulatingPhysics()) OtherV = OtherComp->GetPhysicsLinearVelocityAtPoint(P);
	const FVector Rel = LastVelocity - OtherV;
	const float Closing = float(FVector::DotProduct(Rel, -Nrm)) / 100.f;  // m/s into the obstacle
	const float FromImpulse = float(NormalImpulse.Size()) / MassKg / 100.f;
	const float Dv = FMath::Max(Closing, FromImpulse);
	const float Tangential = float((Rel - Nrm * FVector::DotProduct(Rel, Nrm)).Size()) / 100.f;
	const float Now = GetWorld()->GetTimeSeconds();
	if (Now - LastDentTime < 0.12f && FVector::Dist(P, LastDentPoint) < 60.f) return;
	const FTransform A = GetActorTransform();
	const FVector LP = A.InverseTransformPosition(P), LD = A.InverseTransformVectorNoScale(Nrm);
	const float Seed = FMath::FRandRange(0.f, 100.f);
	if (Dv > 2.2f)
	{
		const float Depth = FMath::Clamp((Dv - 2.2f) * 2.4f, 1.f, 38.f);
		const float Radius = FMath::Clamp(38.f + Dv * 3.2f, 40.f, 110.f);
		MulticastDent(LP, LD, Depth, Radius, Seed, 0);
		LastDentTime = Now, LastDentPoint = P;
		// wheels near the hit get bent, the engine suffers in frontal hits
		for (int32 i = 0; i < 4; i++)
		{
			const FVector WC = Wheels[i].Mount;
			if (FVector::Dist2D(FVector(LP.X, LP.Y, 0), FVector(WC.X, WC.Y, 0)) < 60.f && Dv > 5.f)
			{
				WheelDamage[i] += (Dv - 5.f) * 0.05f;
				if (WheelDamage[i] > 1.f) SetWheelDetached(i, true);
			}
		}
		if (LP.X > 140.f && LD.X < -0.4f && Dv > 4.f) EngineHealth = FMath::Max(0.f, EngineHealth - (Dv - 4.f) * 0.035f);
	}
	else if (Tangential > 3.f && Dv > 0.3f)
	{
		MulticastDent(LP, LD, FMath::Min(Tangential * 0.04f, 0.6f), 35.f, Seed, 1);
		LastDentTime = Now, LastDentPoint = P;
	}
}

void ATTCar::MulticastDent_Implementation(FVector_NetQuantize10 LocalPoint, FVector_NetQuantizeNormal LocalDir, float Depth, float Radius, float Seed, uint8 Kind)
{
	ApplyDent(LocalPoint, LocalDir, Depth, Radius, Seed, Kind);
	if (Sound && Kind == 0) Sound->Impact(FMath::Clamp(Depth / 25.f, 0.1f, 1.f));
}

void ATTCar::ApplyDent(const FVector& LocalPoint, const FVector& LocalDir, float Depth, float Radius, float Seed, uint8 Kind)
{
	if (Parts.Num() == 0) return;
	const FTransform A = GetActorTransform();
	const FVector WP = A.TransformPosition(LocalPoint), WD = A.TransformVectorNoScale(LocalDir);
	for (int32 i = 0; i < Parts.Num(); i++)
	{
		if (DetachedParts & (1 << i)) continue;
		FPart& P = Parts[i];
		const FTransform C = PartComps[i]->GetComponentTransform();
		const ttg::V Pc = ToG(C.InverseTransformPosition(WP)), Dc = ToG(C.InverseTransformVectorNoScale(WD).GetSafeNormal());
		if (Kind == 1)
		{
			P.Mesh.scratch(Pc, Radius, Depth);
			P.bDirty = true;
			continue;
		}
		const double Moved = P.Mesh.dent(Pc, Dc, Depth, Radius, 45.0, Seed);
		if (Moved <= 0) continue;
		P.bDirty = true;
		P.Health = FMath::Max(0.f, P.Health - float(Moved / (double(P.Mesh.base.size()) * 5.0)));
		if (HasAuthority() && P.bCanFall && P.Health < 0.15f && Depth > 8.f) SetPartDetached(i, true, WD * -300.f);
	}
}

void ATTCar::OnRep_Detached()
{
	for (int32 i = 0; i < NumParts; i++)
	{
		const bool bOff = (DetachedParts >> i) & 1;
		if (bOff != bool((AppliedParts >> i) & 1)) SetPartDetached(i, bOff, FVector::ZeroVector);
	}
	for (int32 i = 0; i < 4; i++)
	{
		const bool bOff = (DetachedWheels >> i) & 1;
		if (bOff != bool((AppliedWheels >> i) & 1)) SetWheelDetached(i, bOff);
	}
}

void ATTCar::SetPartDetached(int32 Part, bool bOff, const FVector& Kick)
{
	if (Part <= PartBody || Part >= NumParts) return;
	UProceduralMeshComponent* C = PartComps[Part];
	const uint8 Bit = uint8(1 << Part);
	if (bOff)
	{
		DetachedParts |= Bit, AppliedParts |= Bit;
		C->DetachFromComponent(FDetachmentTransformRules::KeepWorldTransform);
		C->SetCollisionProfileName(TEXT("PhysicsActor"));
		C->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
		C->SetSimulatePhysics(true);
		C->SetMassOverrideInKg(NAME_None, Part == PartHood ? 16.f : 9.f, true);
		C->SetPhysicsLinearVelocity(Chassis->GetPhysicsLinearVelocity() + Kick);
		C->AddAngularImpulseInDegrees(FVector(FMath::FRandRange(-200.f, 200.f), FMath::FRandRange(-200.f, 200.f), 0.f), NAME_None, true);
	}
	else
	{
		DetachedParts &= ~Bit, AppliedParts &= ~Bit;
		C->SetSimulatePhysics(false);
		C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		C->AttachToComponent(Visual, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
		C->SetRelativeTransform(FTransform::Identity);
	}
}

void ATTCar::SetWheelDetached(int32 Wheel, bool bOff)
{
	UProceduralMeshComponent* C = WheelComps[Wheel];
	const uint8 Bit = uint8(1 << Wheel);
	if (bOff)
	{
		DetachedWheels |= Bit, AppliedWheels |= Bit;
		const FVector V = Chassis->GetPhysicsLinearVelocity();
		C->DetachFromComponent(FDetachmentTransformRules::KeepWorldTransform);
		if (Wheels[Wheel].bLeft)
		{
			// mirrored mesh -> plain scale turned half a turn (physics does not like negative scale)
			const FQuat Qw = C->GetComponentQuat();
			C->SetWorldScale3D(FVector::OneVector);
			C->SetWorldRotation(Qw * FQuat(FVector::UpVector, PI));
		}
		C->SetCollisionProfileName(TEXT("PhysicsActor"));
		C->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
		C->SetSimulatePhysics(true);
		C->SetMassOverrideInKg(NAME_None, 20.f, true);
		C->SetPhysicsLinearVelocity(V);
		// keep it rolling: a wheel rolling forward spins about the car's right axis
		C->SetPhysicsAngularVelocityInRadians(Chassis->GetRightVector() * Wheels[Wheel].Omega);
		if (Sound) Sound->Impact(0.6f);
	}
	else
	{
		DetachedWheels &= ~Bit, AppliedWheels &= ~Bit;
		C->SetSimulatePhysics(false);
		C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		C->AttachToComponent(Visual, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
		C->SetRelativeScale3D(FVector(1, Wheels[Wheel].bLeft ? -1 : 1, 1));
		Wheels[Wheel].Omega = 0, Wheels[Wheel].LooseDistance = 0;
	}
}

void ATTCar::MulticastRepair_Implementation(int32 Part)
{
	for (int32 i = 0; i < Parts.Num(); i++)
		if (Part < 0 || Part == i)
		{
			Parts[i].Mesh.repair(1.0);
			Parts[i].Health = 1.f;
			Parts[i].bDirty = true;
		}
}

// ------------------------------------------------------------------ visuals

void ATTCar::UpdateVisuals(float Dt)
{
	// rebuild dented panels (at most two per frame)
	int32 Built = 0;
	for (int32 i = 0; i < Parts.Num() && Built < 2; i++)
		if (Parts[i].bDirty) BuildPartMesh(i, false), Built++;
	// wheels: suspension travel, steering, spin, wobble
	for (int32 i = 0; i < 4; i++)
	{
		FWheel& W = Wheels[i];
		if (DetachedWheels & (1 << i)) continue;
		W.Spin = FMath::Fmod(W.Spin + W.Omega * Dt, 2.f * PI);
		const float Loose = 1.f - FMath::Clamp(WheelNutAverage(i) / NutSpec, 0.f, 1.f);
		const float Bend = FMath::Clamp(WheelDamage[i], 0.f, 1.f);
		const FVector Centre = W.Mount - FVector(0, 0, W.SpringLen) + FVector(0, 0, ZC);
		const float Yaw = (W.bFront ? SteerAngle : 0.f) + Bend * 4.f * (W.bLeft ? -1.f : 1.f);
		const float Camber = Bend * 6.f * (W.bLeft ? 1.f : -1.f) + Loose * 4.f * FMath::Sin(W.Spin * 1.f);
		WheelComps[i]->SetRelativeLocationAndRotation(Centre, FRotator(-FMath::RadiansToDegrees(W.Spin), Yaw, Camber));
	}
	// lights: brake lamps, headlamps
	const bool bBraking = RepBrake > 0.05f && Gear >= 0;
	if (Mats.Num() >= 5)
	{
		Mats[3]->SetScalarParameterValue(TEXT("Glow"), bLights ? 30.f : 0.4f);
		Mats[4]->SetScalarParameterValue(TEXT("Glow"), bBraking ? 40.f : bLights ? 8.f : 0.5f);
	}
	HeadL->SetVisibility(bLights);
	HeadR->SetVisibility(bLights);
	// dashboard (no HUD: the car's own instruments)
	DashSpeed->SetText(FText::FromString(FString::Printf(TEXT("%d"), FMath::RoundToInt(FMath::Abs(Speed) * 3.6f))));
	const TCHAR* GearName[] = {TEXT("R"), TEXT("N"), TEXT("1"), TEXT("2"), TEXT("3"), TEXT("4"), TEXT("5"), TEXT("6")};
	DashInfo->SetText(FText::FromString(FString::Printf(TEXT("%s%s  %4.0f rpm%s"), GearName[FMath::Clamp(Gear + 1, 0, 7)], bAutomatic ? TEXT(" A") : TEXT(""), EngineRpm,
	                                                    EngineHealth < 0.5f ? TEXT("  ДВИГ!") : TEXT(""))));
	// sound
	float Slide = 0;
	for (int32 i = 0; i < 4; i++)
		if (Wheels[i].bContact) Slide = FMath::Max(Slide, Wheels[i].Slide);
	Sound->SetState(EngineRpm, RepThrottle, 1.f - EngineHealth, Slide - 1.2f, FMath::Abs(Speed), bEngineRunning);
	// world note faces the camera
	if (NoteTimer > 0 && (NoteTimer -= Dt) <= 0) WorldNote->SetVisibility(false);
	if (WorldNote->IsVisible())
		if (const APlayerController* PC = Cast<APlayerController>(GetController()))
			if (PC->PlayerCameraManager)
				WorldNote->SetWorldRotation((PC->PlayerCameraManager->GetCameraLocation() - WorldNote->GetComponentLocation()).Rotation());
}

void ATTCar::UpdateCamera(float Dt)
{
	if (View == ETTView::Cockpit) return;
	// chase: follow the car's heading smoothly, mouse orbits around it
	const float CarYaw = GetActorRotation().Yaw;
	CamYaw = FMath::RadiansToDegrees(FMath::Atan2(FMath::Sin(FMath::DegreesToRadians(CamYaw)), FMath::Cos(FMath::DegreesToRadians(CamYaw))));
	const float Delta = FRotator::NormalizeAxis(CarYaw - CamYaw);
	CamYaw += Delta * FMath::Min(1.f, Dt * (View == ETTView::Mechanic ? 0.f : 4.f));
	Arm->SetWorldRotation(FRotator(LookPitch, CamYaw + LookYaw, 0.f));
}

// ------------------------------------------------------------------ input

void ATTCar::SetupInput()
{
	Mapping = NewObject<UInputMappingContext>(this);
	auto Act = [&](FName Name, EInputActionValueType Type) {
		UInputAction* A = NewObject<UInputAction>(this, FName(*(FString(TEXT("IA_")) + Name.ToString())));
		A->ValueType = Type;
		Actions.Add(Name, A);
		return A;
	};
	UInputAction* Thr = Act("Throttle", EInputActionValueType::Axis1D);
	UInputAction* Brk = Act("Brake", EInputActionValueType::Axis1D);
	UInputAction* Str = Act("Steer", EInputActionValueType::Axis1D);
	UInputAction* Hb = Act("Handbrake", EInputActionValueType::Axis1D);
	UInputAction* Look = Act("Look", EInputActionValueType::Axis2D);
	Mapping->MapKey(Thr, EKeys::W);
	Mapping->MapKey(Thr, EKeys::Up);
	Mapping->MapKey(Thr, EKeys::Gamepad_RightTriggerAxis);
	Mapping->MapKey(Brk, EKeys::S);
	Mapping->MapKey(Brk, EKeys::Down);
	Mapping->MapKey(Brk, EKeys::Gamepad_LeftTriggerAxis);
	Mapping->MapKey(Str, EKeys::D);
	Mapping->MapKey(Str, EKeys::Right);
	Mapping->MapKey(Str, EKeys::Gamepad_LeftX);
	Mapping->MapKey(Str, EKeys::A).Modifiers.Add(NewObject<UInputModifierNegate>(this));
	Mapping->MapKey(Str, EKeys::Left).Modifiers.Add(NewObject<UInputModifierNegate>(this));
	Mapping->MapKey(Hb, EKeys::SpaceBar);
	Mapping->MapKey(Hb, EKeys::Gamepad_FaceButton_Bottom);
	Mapping->MapKey(Look, EKeys::Mouse2D);
	Mapping->MapKey(Look, EKeys::Gamepad_Right2D);
	const FName Names[] = {"ShiftUp", "ShiftDown", "Camera", "Mechanic", "Reset", "Use", "Loosen", "RepairPart", "RepairEngine", "Gearbox", "Lights"};
	const FKey Keys[] = {EKeys::E, EKeys::Q, EKeys::C, EKeys::Tab, EKeys::R, EKeys::LeftMouseButton, EKeys::RightMouseButton, EKeys::F, EKeys::G, EKeys::M, EKeys::L};
	for (int32 i = 0; i < int32(UE_ARRAY_COUNT(Names)); i++) Mapping->MapKey(Act(Names[i], EInputActionValueType::Boolean), Keys[i]);
}

void ATTCar::SetupPlayerInputComponent(UInputComponent* PIC)
{
	Super::SetupPlayerInputComponent(PIC);
	SetupInput();
	if (APlayerController* PC = Cast<APlayerController>(GetController()))
		if (ULocalPlayer* LP = PC->GetLocalPlayer())
			if (UEnhancedInputLocalPlayerSubsystem* Sub = LP->GetSubsystem<UEnhancedInputLocalPlayerSubsystem>()) Sub->AddMappingContext(Mapping, 0);
	UEnhancedInputComponent* E = Cast<UEnhancedInputComponent>(PIC);
	if (!E) return;
	for (const FName N : {FName("Throttle"), FName("Brake"), FName("Steer"), FName("Handbrake")})
	{
		auto Fn = N == "Throttle" ? &ATTCar::OnThrottle : N == "Brake" ? &ATTCar::OnBrake : N == "Steer" ? &ATTCar::OnSteer : &ATTCar::OnHandbrake;
		E->BindAction(Actions[N], ETriggerEvent::Triggered, this, Fn);
		E->BindAction(Actions[N], ETriggerEvent::Completed, this, Fn);
	}
	E->BindAction(Actions["Look"], ETriggerEvent::Triggered, this, &ATTCar::OnLook);
	E->BindAction(Actions["ShiftUp"], ETriggerEvent::Started, this, &ATTCar::OnShiftUp);
	E->BindAction(Actions["ShiftDown"], ETriggerEvent::Started, this, &ATTCar::OnShiftDown);
	E->BindAction(Actions["Camera"], ETriggerEvent::Started, this, &ATTCar::OnCamera);
	E->BindAction(Actions["Mechanic"], ETriggerEvent::Started, this, &ATTCar::OnMechanic);
	E->BindAction(Actions["Reset"], ETriggerEvent::Started, this, &ATTCar::OnReset);
	E->BindAction(Actions["Use"], ETriggerEvent::Started, this, &ATTCar::OnUse);
	E->BindAction(Actions["Loosen"], ETriggerEvent::Started, this, &ATTCar::OnLoosen);
	E->BindAction(Actions["RepairPart"], ETriggerEvent::Started, this, &ATTCar::OnRepairPart);
	E->BindAction(Actions["RepairEngine"], ETriggerEvent::Started, this, &ATTCar::OnRepairEngine);
	E->BindAction(Actions["Gearbox"], ETriggerEvent::Started, this, &ATTCar::OnGearbox);
	E->BindAction(Actions["Lights"], ETriggerEvent::Started, this, &ATTCar::OnLightsKey);
}

void ATTCar::OnThrottle(const FInputActionValue& V) { InThrottle = View == ETTView::Mechanic ? 0.f : V.Get<float>(); }
void ATTCar::OnBrake(const FInputActionValue& V) { InBrake = View == ETTView::Mechanic ? 0.f : V.Get<float>(); }
void ATTCar::OnSteer(const FInputActionValue& V) { InSteer = View == ETTView::Mechanic ? 0.f : V.Get<float>(); }
void ATTCar::OnHandbrake(const FInputActionValue& V) { InHandbrake = V.Get<float>(); }
void ATTCar::OnLook(const FInputActionValue& V)
{
	const FVector2D L = V.Get<FVector2D>();
	LookYaw = FRotator::NormalizeAxis(LookYaw + L.X * 1.5f);
	LookPitch = FMath::Clamp(LookPitch + L.Y * 1.5f, -75.f, 25.f);
	if (View == ETTView::Cockpit) Camera->SetRelativeRotation(FRotator(FMath::Clamp(LookPitch, -60.f, 40.f), FMath::Clamp(LookYaw, -140.f, 140.f), 0));
}
void ATTCar::OnShiftUp() { Command(CmdShiftUp, 0); }
void ATTCar::OnShiftDown() { Command(CmdShiftDown, 0); }
void ATTCar::OnGearbox() { Command(CmdGearbox, 0); }
void ATTCar::OnLightsKey() { Command(CmdLights, 0); }
void ATTCar::OnReset() { Command(CmdReset, 0); }
void ATTCar::OnCamera()
{
	if (View == ETTView::Mechanic) return;
	View = View == ETTView::Chase ? ETTView::Cockpit : ETTView::Chase;
	LookYaw = 0, LookPitch = View == ETTView::Chase ? -10.f : 0.f;
	ApplyView();
}
void ATTCar::OnMechanic()
{
	if (View != ETTView::Mechanic && FMath::Abs(Speed) > 1.f) return;  // only when parked
	View = View == ETTView::Mechanic ? ETTView::Chase : ETTView::Mechanic;
	LookYaw = 0, LookPitch = -20.f;
	ApplyView();
	if (View == ETTView::Mechanic)
		Note(TEXT("ЛКМ: затянуть гайку   ПКМ: открутить\nF: выправить / поставить деталь   G: двигатель"), GetActorLocation() + FVector(0, 0, 230), 5.f);
}

void ATTCar::ApplyView()
{
	if (View == ETTView::Cockpit)
	{
		Arm->AttachToComponent(Visual, FAttachmentTransformRules::KeepRelativeTransform);
		Arm->SetUsingAbsoluteRotation(false);
		Arm->SetRelativeLocation(FVector(12.f, -36.f, 118.f));
		Arm->SetRelativeRotation(FRotator::ZeroRotator);
		Arm->TargetArmLength = 0.f;
		Arm->SocketOffset = FVector::ZeroVector;
		Arm->bEnableCameraLag = false;
		Arm->bEnableCameraRotationLag = false;
		Arm->bDoCollisionTest = false;
		Camera->SetFieldOfView(80.f);
		Camera->SetRelativeRotation(FRotator::ZeroRotator);
	}
	else
	{
		Arm->AttachToComponent(Chassis, FAttachmentTransformRules::KeepRelativeTransform);
		Arm->SetUsingAbsoluteRotation(true);
		Arm->SetRelativeLocation(FVector(0.f, 0.f, 30.f));
		Arm->TargetArmLength = View == ETTView::Mechanic ? 460.f : 620.f;
		Arm->SocketOffset = FVector(0.f, 0.f, View == ETTView::Mechanic ? 40.f : 110.f);
		Arm->bEnableCameraLag = View == ETTView::Chase;
		Arm->CameraLagSpeed = 9.f;
		Arm->bEnableCameraRotationLag = false;
		Arm->bDoCollisionTest = true;
		Arm->ProbeChannel = ECC_Camera;
		Camera->SetRelativeRotation(FRotator::ZeroRotator);
		Camera->SetFieldOfView(View == ETTView::Mechanic ? 70.f : 78.f);
		CamYaw = GetActorRotation().Yaw;
	}
}

void ATTCar::Note(const FString& Text, const FVector& Where, float Secs)
{
	WorldNote->SetText(FText::FromString(Text));
	WorldNote->SetWorldLocation(Where);
	WorldNote->SetVisibility(true);
	NoteTimer = Secs;
}

// ------------------------------------------------------------------ workshop (first person, no menus)

int32 ATTCar::AimedWheel() const
{
	const APlayerController* PC = Cast<APlayerController>(GetController());
	if (!PC || !PC->PlayerCameraManager) return -1;
	const FVector Eye = PC->PlayerCameraManager->GetCameraLocation(), Dir = PC->PlayerCameraManager->GetCameraRotation().Vector();
	int32 Best = -1;
	double Bd = 70.0;
	for (int32 i = 0; i < 4; i++)
	{
		// a missing wheel is aimed at through its empty corner
		const FVector C = (DetachedWheels & (1 << i)) ? Chassis->GetComponentTransform().TransformPosition(Wheels[i].Mount) : WheelComps[i]->GetComponentLocation();
		const double d = FMath::PointDistToLine(C, Dir, Eye);
		if (d < Bd) Bd = d, Best = i;
	}
	return Best;
}

int32 ATTCar::AimedPart() const
{
	const APlayerController* PC = Cast<APlayerController>(GetController());
	if (!PC || !PC->PlayerCameraManager) return -1;
	const FVector Eye = PC->PlayerCameraManager->GetCameraLocation(), Dir = PC->PlayerCameraManager->GetCameraRotation().Vector();
	int32 Best = -1;
	double Bd = 110.0;
	for (int32 i = 0; i < Parts.Num(); i++)
	{
		const FVector C = (DetachedParts & (1 << i)) ? Visual->GetComponentTransform().TransformPosition(ToF(Parts[i].Mesh.centre()))
		                                              : PartComps[i]->GetComponentTransform().TransformPosition(ToF(Parts[i].Mesh.centre()));
		const double d = FMath::PointDistToLine(C, Dir, Eye) + (i == PartBody ? 40.0 : 0.0);  // panels win over the shell
		if (d < Bd) Bd = d, Best = i;
	}
	return Best;
}

void ATTCar::Command(uint8 Cmd, int32 Index)
{
	if (HasAuthority()) DoCommand(Cmd, Index);
	else ServerCommand(Cmd, Index);
}

void ATTCar::ServerCommand_Implementation(uint8 Cmd, int32 Index) { DoCommand(Cmd, Index); }

void ATTCar::DoCommand(uint8 Cmd, int32 Index)
{
	const bool bParked = FMath::Abs(Speed) < 1.f;
	switch (Cmd)
	{
	case CmdTighten:
		if (bParked && Index >= 0 && Index < 4 && !(DetachedWheels & (1 << Index)))
		{
			// next nut below spec gets one ratchet stroke; once all are at spec, keep going = overtighten
			int32 k = 0;
			while (k < 5 && (NutTorque[Index * 5 + k] < 0.f || NutTorque[Index * 5 + k] >= NutSpec)) k++;
			if (k == 5)
				for (k = 0; k < 5 && NutTorque[Index * 5 + k] < 0.f; k++) {}
			if (k < 5)
			{
				float& Nt = NutTorque[Index * 5 + k];
				Nt += 20.f;
				if (Nt > NutSpec * 1.45f) Nt = -1.f;  // thread stripped
			}
		}
		break;
	case CmdLoosen:
		if (bParked && Index >= 0 && Index < 4)
			for (int32 k = 0; k < 5; k++)
				if (NutTorque[Index * 5 + k] > 0.f) NutTorque[Index * 5 + k] = 0.f;
		break;
	case CmdRepairPart:
		if (!bParked) break;
		if (Index >= 100 && Index < 104)
		{
			// refit a wheel (nuts finger-tight: they still have to be torqued) or straighten a bent one
			const int32 W = Index - 100;
			if (DetachedWheels & (1 << W)) SetWheelDetached(W, false);
			WheelDamage[W] = 0.f;
			for (int32 k = 0; k < 5; k++) NutTorque[W * 5 + k] = 0.f;
		}
		else if (Index >= 0 && Index < NumParts)
		{
			if (DetachedParts & (1 << Index)) SetPartDetached(Index, false, FVector::ZeroVector);
			MulticastRepair(Index);
		}
		break;
	case CmdRepairEngine:
		if (bParked) EngineHealth = 1.f, bEngineRunning = true;
		break;
	case CmdReset:
	{
		const FRotator R(0.f, GetActorRotation().Yaw, 0.f);
		Chassis->SetWorldLocationAndRotation(GetActorLocation() + FVector(0, 0, 120), R, false, nullptr, ETeleportType::TeleportPhysics);
		Chassis->SetPhysicsLinearVelocity(FVector::ZeroVector);
		Chassis->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
		break;
	}
	case CmdShiftUp:
		bAutomatic = false;
		if (Gear < 6) Gear++, ShiftTimer = 0.2f;
		break;
	case CmdShiftDown:
		bAutomatic = false;
		if (Gear > -1 && (Gear > 0 || Speed < 1.f)) Gear--, ShiftTimer = 0.2f;
		break;
	case CmdGearbox:
		bAutomatic = !bAutomatic;
		if (bAutomatic && Gear == 0) Gear = 1;
		break;
	case CmdLights:
		bLights = !bLights;
		break;
	default:
		break;
	}
}

void ATTCar::OnUse()
{
	if (View != ETTView::Mechanic) return;
	const int32 W = AimedWheel();
	if (W < 0 || (DetachedWheels & (1 << W))) return;
	Command(CmdTighten, W);
	// show the wrench reading (predicted locally, the server has the real value a moment later)
	int32 k = 0;
	while (k < 5 && (NutTorque[W * 5 + k] < 0.f || NutTorque[W * 5 + k] >= NutSpec)) k++;
	FString Txt;
	for (int32 n = 0; n < 5; n++)
	{
		const float Nt = NutTorque[W * 5 + n] + (n == k ? 20.f : 0.f);
		Txt += NutTorque[W * 5 + n] < 0.f ? TEXT(" X ") : FString::Printf(TEXT(" %.0f "), FMath::Min(Nt, 999.f));
	}
	if (k < 5 && NutTorque[W * 5 + k] + 20.f >= NutSpec) Sound->Click();
	Note(TEXT("гайки, Н·м:") + Txt + (k == 5 ? TEXT("\nвсе затянуты (дальше — сорвёшь резьбу)") : TEXT("")), WheelComps[W]->GetComponentLocation() + FVector(0, 0, 70), 2.5f);
}

void ATTCar::OnLoosen()
{
	if (View != ETTView::Mechanic) return;
	const int32 W = AimedWheel();
	if (W < 0) return;
	Command(CmdLoosen, W);
	Note(TEXT("гайки откручены"), WheelComps[W]->GetComponentLocation() + FVector(0, 0, 70), 2.f);
}

void ATTCar::OnRepairPart()
{
	if (View != ETTView::Mechanic) return;
	const int32 W = AimedWheel();
	if (W >= 0 && ((DetachedWheels & (1 << W)) || WheelDamage[W] > 0.05f))
	{
		Command(CmdRepairPart, 100 + W);
		Note(TEXT("колесо на месте — затяни гайки (ЛКМ)"), Chassis->GetComponentTransform().TransformPosition(Wheels[W].Mount) + FVector(0, 0, 80), 3.f);
		return;
	}
	const int32 P = AimedPart();
	if (P < 0) return;
	const TCHAR* Names[NumParts] = {TEXT("кузов выправлен"), TEXT("капот"), TEXT("крышка багажника"), TEXT("передний бампер"), TEXT("задний бампер")};
	Command(CmdRepairPart, P);
	Note(P == PartBody ? FString(Names[P]) : FString(Names[P]) + TEXT(": как новый"), GetActorLocation() + FVector(0, 0, 200), 2.5f);
}

void ATTCar::OnRepairEngine()
{
	if (View != ETTView::Mechanic) return;
	Command(CmdRepairEngine, 0);
	Note(FString::Printf(TEXT("двигатель: было %.0f%%, теперь 100%%"), EngineHealth * 100.f), GetActorLocation() + FVector(0, 0, 200), 2.5f);
}
