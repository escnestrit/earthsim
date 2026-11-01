#include "TTCar.h"
#include "TTDriver.h"
#include "TTEngineSound.h"
#include "Camera/CameraComponent.h"
#include "Camera/PlayerCameraManager.h"
#include "Components/BoxComponent.h"
#include "Components/SpotLightComponent.h"
#include "Components/TextRenderComponent.h"
#include "Dom/JsonObject.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "Engine/LocalPlayer.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/SpringArmComponent.h"
#include "InputAction.h"
#include "InputMappingContext.h"
#include "InputModifiers.h"
#include "Kismet/GameplayStatics.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Net/UnrealNetwork.h"
#include "Particles/ParticleSystem.h"
#include "Particles/ParticleSystemComponent.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "ProceduralMeshComponent.h"
#include "Rendering/PositionVertexBuffer.h"
#include "Rendering/StaticMeshVertexBuffer.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "StaticMeshResources.h"

namespace
{
// ---- car constants (cm unless noted)
constexpr float ZC = 72.f;           // chassis origin height above the ground at ride height
constexpr float SuspMax = 22.f;      // wheel centre travel below the mount
constexpr float StaticSpring = 12.8f;// wheel centre below the mount at rest
constexpr float MassKg = 1350.f;
constexpr float SpringK = 36000.f;   // N/m
constexpr float DampBump = 3000.f, DampRebound = 4600.f;  // N s/m
constexpr float AntiRollF = 14000.f, AntiRollR = 9000.f;  // N/m
constexpr float WheelInertia = 1.3f; // kg m^2
constexpr float NutSpec = 120.f;     // N m
const float GearRatio[6] = {3.45f, 2.20f, 1.55f, 1.18f, 0.94f, 0.78f};
constexpr float FinalDrive = 3.7f, ReverseRatio = 3.3f;
constexpr float IdleRpm = 850.f, RedLine = 6900.f;
enum EProcPart : int32 { PartBody, PartHood, PartTrunk, PartBumperF, PartBumperR, NumProcParts };

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

UParticleSystem* Fx(const TCHAR* Name)
{
	return LoadObject<UParticleSystem>(nullptr, *FString::Printf(TEXT("/Game/StarterContent/Particles/%s.%s"), Name, Name));
}

bool HasAny(const FString& S, std::initializer_list<const TCHAR*> Words)
{
	for (const TCHAR* W : Words)
		if (S.Contains(W, ESearchCase::IgnoreCase)) return true;
	return false;
}
}  // namespace

// ------------------------------------------------------------------ catalog of imported cars

const TArray<FTTCarDef>& ATTCar::Catalog()
{
	static TArray<FTTCarDef> Cars;
	static bool bLoaded = false;
	if (bLoaded) return Cars;
	bLoaded = true;
	FString Text;
	if (!FFileHelper::LoadFileToString(Text, *(FPaths::ProjectContentDir() / TEXT("TT/Cars/cars.json")))) return Cars;
	TSharedPtr<FJsonObject> Root;
	if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Root) || !Root.IsValid()) return Cars;
	const TArray<TSharedPtr<FJsonValue>>* List;
	if (!Root->TryGetArrayField(TEXT("cars"), List)) return Cars;
	for (const TSharedPtr<FJsonValue>& CarV : *List)
	{
		const TSharedPtr<FJsonObject> C = CarV->AsObject();
		if (!C) continue;
		FTTCarDef Def;
		Def.Name = C->GetStringField(TEXT("name"));
		const TArray<TSharedPtr<FJsonValue>>* PartList;
		if (!C->TryGetArrayField(TEXT("parts"), PartList)) continue;
		for (const TSharedPtr<FJsonValue>& PV : *PartList)
		{
			const TSharedPtr<FJsonObject> P = PV->AsObject();
			const TSharedPtr<FJsonObject>* Vars;
			if (!P || !P->TryGetObjectField(TEXT("variants"), Vars)) continue;
			FTTCarPartDef Part;
			Part.Category = P->GetStringField(TEXT("category"));
			Part.Group = P->GetStringField(TEXT("group"));
			TArray<FString> Names;
			(*Vars)->Values.GetKeys(Names);
			// stock first (the shortest name containing "stock"), then alphabetical
			Names.Sort([](const FString& A, const FString& B) {
				const bool SA = A.Contains(TEXT("stock"), ESearchCase::IgnoreCase), SB = B.Contains(TEXT("stock"), ESearchCase::IgnoreCase);
				if (SA != SB) return SA;
				if (SA && A.Len() != B.Len()) return A.Len() < B.Len();
				return A < B;
			});
			for (const FString& N : Names)
			{
				TArray<FString> Meshes;
				for (const TSharedPtr<FJsonValue>& M : (*Vars)->GetArrayField(N)) Meshes.Add(M->AsString());
				Part.VariantNames.Add(N);
				Part.VariantMeshes.Add(Meshes);
			}
			const FString Key = Part.Category + TEXT(" ") + Part.Group;
			Part.bCanFall = HasAny(Key, {TEXT("bumper"), TEXT("bonnet"), TEXT("hood"), TEXT("trunk"), TEXT("boot"), TEXT("door"), TEXT("mirror"), TEXT("spoiler"),
			                             TEXT("skirt"), TEXT("exhaust"), TEXT("headlight"), TEXT("taillight"), TEXT("fender")}) &&
			                !HasAny(Key, {TEXT("interior"), TEXT("int")});
			if (Part.VariantNames.Num()) Def.Parts.Add(Part);
		}
		if (Def.Parts.Num()) Cars.Add(Def);
	}
	UE_LOG(LogTemp, Log, TEXT("TrueTorque: %d imported cars in the catalog"), Cars.Num());
	return Cars;
}

const FTTCarDef* ATTCar::Def() const
{
	const TArray<FTTCarDef>& C = Catalog();
	return C.IsValidIndex(CarIndex) ? &C[CarIndex] : nullptr;
}

// ------------------------------------------------------------------ construction

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

void ATTCar::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
	Super::GetLifetimeReplicatedProps(OutLifetimeProps);
	DOREPLIFETIME(ATTCar, PaintColor);
	DOREPLIFETIME(ATTCar, CarIndex);
	DOREPLIFETIME(ATTCar, RepThrottle);
	DOREPLIFETIME(ATTCar, RepBrake);
	DOREPLIFETIME(ATTCar, RepSteer);
	DOREPLIFETIME(ATTCar, RepHandbrake);
	DOREPLIFETIME(ATTCar, Gear);
	DOREPLIFETIME(ATTCar, bAutomatic);
	DOREPLIFETIME(ATTCar, bLights);
	DOREPLIFETIME(ATTCar, EngineHealth);
	DOREPLIFETIME(ATTCar, EngineTemp);
	DOREPLIFETIME(ATTCar, Coolant);
	DOREPLIFETIME(ATTCar, Oil);
	DOREPLIFETIME(ATTCar, RadiatorLeak);
	DOREPLIFETIME(ATTCar, bRadiatorWelded);
	DOREPLIFETIME(ATTCar, NutTorque);
	DOREPLIFETIME(ATTCar, WheelDamage);
	DOREPLIFETIME(ATTCar, PartVariant);
	DOREPLIFETIME(ATTCar, PartOff);
	DOREPLIFETIME(ATTCar, DetachedWheels);
}

void ATTCar::BeginPlay()
{
	Super::BeginPlay();
	// rigid body: mass, low centre of mass, a dead (non-bouncy) shell
	Chassis->SetMassOverrideInKg(NAME_None, MassKg, true);
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

	Mats.Add(MakeMat(this, TEXT("M_CarPaint"), PaintColor));
	Mats.Add(MakeMat(this, TEXT("M_Glass"), FLinearColor(0.01f, 0.012f, 0.015f)));
	Mats.Add(MakeMat(this, TEXT("M_Trim"), FLinearColor(0.02f, 0.02f, 0.02f)));
	Mats.Add(MakeMat(this, TEXT("M_Light"), FLinearColor(1.f, 0.96f, 0.88f)));
	Mats.Add(MakeMat(this, TEXT("M_Light"), FLinearColor(0.9f, 0.05f, 0.02f)));
	WheelMats.Add(MakeMat(this, TEXT("M_Tyre"), FLinearColor(0.025f, 0.025f, 0.025f)));
	WheelMats.Add(MakeMat(this, TEXT("M_Rim"), FLinearColor(0.7f, 0.7f, 0.72f)));

	if (const FTTCarDef* D = Def()) BuildImported(*D);
	if (Parts.Num() == 0) CarIndex = -1, BuildProcedural();
	PartOff.SetNumZeroed(Parts.Num());
	AppliedOff.SetNumZeroed(Parts.Num());
	if (PartVariant.Num() != Parts.Num()) PartVariant.SetNumZeroed(Parts.Num());
	AppliedVariant = PartVariant;
	SetupWheels();

	// effects (StarterContent particles; silently absent without them)
	auto MakeFx = [this](UParticleSystem* T, const FVector& At, float Scale) -> UParticleSystemComponent* {
		if (!T) return nullptr;
		UParticleSystemComponent* C = NewObject<UParticleSystemComponent>(this);
		C->bAutoActivate = false;
		C->SetTemplate(T);
		C->SetupAttachment(Visual);
		C->SetRelativeLocation(At);
		C->SetRelativeScale3D(FVector(Scale));
		C->RegisterComponent();
		return C;
	};
	const float HoodZ = 95.f;
	Steam = MakeFx(Fx(TEXT("P_Steam_Lit")), FVector(CarLength * 0.36f, 0.f, HoodZ), 0.6f);
	Smoke = MakeFx(Fx(TEXT("P_Smoke")), FVector(CarLength * 0.3f, 0.f, HoodZ), 0.5f);
	for (int32 i = 2; i < 4; i++) TyreSmoke.Add(MakeFx(Fx(TEXT("P_Smoke")), FVector(Wheels[i].Mount.X, Wheels[i].Mount.Y, 10.f), 0.35f));
	ApplyView();
	OnRep_Detached();  // late joiners: parts that fell off before we arrived
}

// ------------------------------------------------------------------ body building

void ATTCar::CreatePartComponent(int32 Part)
{
	UProceduralMeshComponent* C = NewObject<UProceduralMeshComponent>(this);
	C->SetupAttachment(Visual);
	C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	C->bUseComplexAsSimpleCollision = false;
	C->SetCastShadow(true);
	C->RegisterComponent();
	PartComps.Add(C);
}

void ATTCar::BuildProcedural()
{
	ttg::CarGeom G;
	ttg::build_car(G);
	const TCHAR* Names[NumProcParts] = {TEXT("кузов"), TEXT("капот"), TEXT("крышка багажника"), TEXT("передний бампер"), TEXT("задний бампер")};
	ttg::Mesh* Src[NumProcParts] = {&G.body, &G.hood, &G.trunk, &G.bumper_f, &G.bumper_r};
	Parts.SetNum(NumProcParts);
	for (int32 i = 0; i < NumProcParts; i++)
	{
		Parts[i].Mesh = MoveTemp(*Src[i]);
		Parts[i].Name = Names[i];
		Parts[i].bCanFall = i != PartBody;
		for (UMaterialInstanceDynamic* M : Mats) Parts[i].Materials.Add(M);
		CreatePartComponent(i);
		BuildPartMesh(i, true);
	}
	EnginePart = PartHood;
	CarLength = 452.f, CarWidth = 180.f, WheelRadius = ttg::WHEEL_R;
	ttg::build_wheel(WheelMesh);
	Wheels.SetNum(4);
	for (int32 i = 0; i < 4; i++)
		Wheels[i].Mount = FVector(i < 2 ? ttg::WHEELBASE_HALF : -ttg::WHEELBASE_HALF, (i % 2) ? ttg::TRACK_HALF : -ttg::TRACK_HALF, 0.f);
}

bool ATTCar::LoadPartMeshes(int32 Index, const TArray<FString>& Meshes, bool bApplyFrame)
{
	FPart& P = Parts[Index];
	P.Mesh = ttg::Mesh();
	P.Materials.Reset();
	for (const FString& Path : Meshes)
	{
		UStaticMesh* SM = LoadObject<UStaticMesh>(nullptr, *Path);
		if (!SM || !SM->GetRenderData() || SM->GetRenderData()->LODResources.Num() == 0) continue;
		const FStaticMeshLODResources& L = SM->GetRenderData()->LODResources[0];
		const FPositionVertexBuffer& PB = L.VertexBuffers.PositionVertexBuffer;
		const FStaticMeshVertexBuffer& VB = L.VertexBuffers.StaticMeshVertexBuffer;
		TArray<uint32> Idx;
		L.IndexBuffer.GetCopy(Idx);
		if (PB.GetNumVertices() == 0 || !PB.GetVertexData() || Idx.Num() == 0)
		{
			UE_LOG(LogTemp, Warning, TEXT("TrueTorque: %s has no CPU vertex data (enable Allow CPU Access)"), *Path);
			continue;
		}
		const int32 Base = int32(P.Mesh.base.size());
		const int32 MatBase = P.Materials.Num();
		for (uint32 v = 0; v < PB.GetNumVertices(); v++)
		{
			const FVector4f Tz = VB.VertexTangentZ(v);
			FVector Pos = FVector(PB.VertexPosition(v)), N(Tz.X, Tz.Y, Tz.Z);
			if (bApplyFrame) Pos = ImportFrame.TransformPosition(Pos), N = ImportFrame.TransformVectorNoScale(N);
			const FVector2f UV = VB.GetNumTexCoords() > 0 ? VB.GetVertexUV(v, 0) : FVector2f::ZeroVector;
			P.Mesh.add_vertex(ToG(Pos), ToG(N), UV.X, UV.Y);
		}
		for (int32 m = 0; m < SM->GetStaticMaterials().Num(); m++)
		{
			UMaterialInterface* Mi = SM->GetMaterial(m);
			if (!Mi) Mi = Mats[0];
			P.Materials.Add(Mi);
			PartMats.AddUnique(Mi);
		}
		if (SM->GetStaticMaterials().Num() == 0) P.Materials.Add(Mats[0]);
		for (const FStaticMeshSection& S : L.Sections)
		{
			const uint8 Slot = uint8(FMath::Clamp(MatBase + S.MaterialIndex, 0, 255));
			for (uint32 t = 0; t < S.NumTriangles; t++)
			{
				const uint32 i0 = S.FirstIndex + t * 3;
				P.Mesh.tris.push_back({Base + int32(Idx[i0]), Base + int32(Idx[i0 + 1]), Base + int32(Idx[i0 + 2]), Slot, true});
			}
		}
	}
	P.Mesh.build_groups();
	return P.Mesh.tris.size() > 0;
}

// Importers disagree about winding and handedness. Decide from the geometry: faces point away from
// the car's centre (outer panels), and a closed object encloses positive volume wherever the centre is.
static void OrientOutward(ttg::Mesh& M, const ttg::V& CarCentre)
{
	double Sum = 0;
	for (const ttg::Tri& T : M.tris)
	{
		const ttg::V A = M.base[T.a], B = M.base[T.b], C = M.base[T.c];
		Sum += ttg::dot(ttg::cross(C - A, B - A), (A + B + C) * (1.0 / 3) - CarCentre);
	}
	if (Sum < 0)
		for (ttg::Tri& T : M.tris) std::swap(T.b, T.c);
	// the file's normals must agree with the (now outward) faces, or the lighting comes from inside
	if (M.nrm0.size() != M.base.size()) return;
	double Agree = 0;
	for (const ttg::Tri& T : M.tris)
	{
		const ttg::V F = ttg::cross(M.base[T.c] - M.base[T.a], M.base[T.b] - M.base[T.a]);
		Agree += ttg::dot(F, M.nrm0[T.a] + M.nrm0[T.b] + M.nrm0[T.c]) >= 0 ? 1 : -1;
	}
	if (Agree < 0)
		for (ttg::V& N : M.nrm0) N = -N;
}

void ATTCar::BuildImported(const FTTCarDef& D)
{
	Parts.SetNum(D.Parts.Num());
	if (PartVariant.Num() != D.Parts.Num()) PartVariant.SetNumZeroed(D.Parts.Num());
	// 1. load every part in mesh space
	TArray<int32> Loaded;
	for (int32 i = 0; i < D.Parts.Num(); i++)
	{
		const int32 V = FMath::Clamp(int32(PartVariant[i]), 0, D.Parts[i].VariantMeshes.Num() - 1);
		Parts[i].Def = i;
		Parts[i].Name = D.Parts[i].Group;
		Parts[i].bCanFall = D.Parts[i].bCanFall;
		if (LoadPartMeshes(i, D.Parts[i].VariantMeshes[V], false)) Loaded.Add(i);
	}
	if (Loaded.Num() == 0)
	{
		Parts.Reset();
		return;
	}
	// 2. car frame: the smallest extent is up, the largest is length; roof/windows say which way is
	//    up, front bumper/headlights/bonnet say which way is forward
	auto PartCentre = [&](const TArray<int32>& Which) {
		FVector S = FVector::ZeroVector;
		for (int32 i : Which) S += ToF(Parts[i].Mesh.centre());
		return S / FMath::Max(1, Which.Num());
	};
	FBox All(ForceInit);
	for (int32 i : Loaded)
		for (const ttg::V& p : Parts[i].Mesh.base) All += ToF(p);
	// units: a car is a few metres long; files in metres (or millimetres) get rescaled to centimetres
	const double L0 = All.GetSize().GetMax();
	const double Scale = L0 < 20.0 ? 100.0 : L0 > 2000.0 ? (L0 > 20000.0 ? 0.1 : 0.01) * 1.0 : 1.0;
	if (Scale != 1.0)
	{
		for (int32 i : Loaded)
			for (ttg::V& p : Parts[i].Mesh.base) p = p * Scale;
		All = FBox(All.Min * Scale, All.Max * Scale);
	}
	const FVector Ext = All.GetSize(), Ctr = All.GetCenter();
	for (int32 i : Loaded) OrientOutward(Parts[i].Mesh, ToG(Ctr));
	int32 UpAx = 0, LenAx = 0;
	for (int32 a = 1; a < 3; a++)
	{
		if (Ext[a] < Ext[UpAx]) UpAx = a;
		if (Ext[a] > Ext[LenAx]) LenAx = a;
	}
	TArray<int32> Top, Front;
	for (int32 i : Loaded)
	{
		const FString K = D.Parts[i].Category + TEXT(" ") + D.Parts[i].Group;
		if (HasAny(K, {TEXT("roof"), TEXT("window")})) Top.Add(i);
		if (HasAny(K, {TEXT("front"), TEXT("headlight"), TEXT("bonnet"), TEXT("hood")})) Front.Add(i);
		if (HasAny(K, {TEXT("bonnet"), TEXT("hood")})) EnginePart = i;
	}
	float UpSign = 1.f;
	if (Top.Num()) UpSign = PartCentre(Top)[UpAx] < Ctr[UpAx] ? -1.f : 1.f;
	else
	{
		// no roof/window parts: a car is narrower at the top (greenhouse) than at the bottom
		const int32 LatAx = 3 - UpAx - LenAx;
		const double Lo = All.Min[UpAx], Hgt = Ext[UpAx];
		// average half-width per band (a max would pick up the mirrors)
		double WTop = 0, WBot = 0, NTop = 0, NBot = 0;
		for (int32 i : Loaded)
			for (const ttg::V& p : Parts[i].Mesh.base)
			{
				const FVector q = ToF(p);
				const double h = (q[UpAx] - Lo) / Hgt, w = FMath::Abs(q[LatAx] - Ctr[LatAx]);
				if (h > 0.75) WTop += w, NTop++;
				if (h < 0.4) WBot += w, NBot++;
			}
		WTop /= FMath::Max(1.0, NTop), WBot /= FMath::Max(1.0, NBot);
		UpSign = WTop <= WBot ? 1.f : -1.f;
		UE_LOG(LogTemp, Log, TEXT("TrueTorque: up by shape: top width %.0f, bottom width %.0f"), WTop, WBot);
	}
	const float FrontSign = Front.Num() && PartCentre(Front)[LenAx] < Ctr[LenAx] ? -1.f : 1.f;
	UE_LOG(LogTemp, Log, TEXT("TrueTorque: raw box %s, scale %.2f, up axis %d sign %.0f, length axis %d sign %.0f"), *All.ToString(), Scale, UpAx, UpSign, LenAx, FrontSign);
	FVector AxX = FVector::ZeroVector, AxZ = FVector::ZeroVector;
	AxX[LenAx] = FrontSign, AxZ[UpAx] = UpSign;
	const FVector AxY = FVector::CrossProduct(AxZ, AxX);
	FVector Origin = Ctr;
	Origin[UpAx] = UpSign > 0 ? All.Min[UpAx] : All.Max[UpAx];
	const FMatrix CarToMesh(FPlane(AxX, 0.f), FPlane(AxY, 0.f), FPlane(AxZ, 0.f), FPlane(Origin, 1.f));
	ImportFrame = FTransform(CarToMesh.Inverse());
	auto Transform = [&](const FTransform& Tf) {
		for (int32 i : Loaded)
		{
			ttg::Mesh& M = Parts[i].Mesh;
			for (size_t k = 0; k < M.base.size(); k++)
			{
				M.base[k] = ToG(Tf.TransformPosition(ToF(M.base[k])));
				if (k < M.nrm0.size()) M.nrm0[k] = ToG(Tf.TransformVectorNoScale(ToF(M.nrm0[k])));
			}
			M.build_groups();
		}
	};
	Transform(ImportFrame);
	// 3. size, wheel arches (where the lower edge of the sides rises), wheel size, ground clearance
	FBox Body(ForceInit);
	for (int32 i : Loaded)
		if (!HasAny(D.Parts[i].Group, {TEXT("mirror")}))
			for (const ttg::V& p : Parts[i].Mesh.base) Body += ToF(p);
	CarLength = Body.GetSize().X, CarWidth = Body.GetSize().Y;
	const float H = Body.GetSize().Z;
	const int32 NB = FMath::Max(8, int32(CarLength / 4.f));
	TArray<float> MinZ;
	MinZ.Init(1e9f, NB);
	for (int32 i : Loaded)
	{
		if (HasAny(D.Parts[i].Category + D.Parts[i].Group, {TEXT("interior"), TEXT("window"), TEXT("light"), TEXT("mirror"), TEXT("exhaust")})) continue;
		for (const ttg::V& p : Parts[i].Mesh.base)
			if (FMath::Abs(p.y) > CarWidth * 0.3 && p.z < H * 0.6)
			{
				const int32 b = FMath::Clamp(int32((p.x - Body.Min.X) / CarLength * NB), 0, NB - 1);
				MinZ[b] = FMath::Min(MinZ[b], float(p.z));
			}
	}
	TArray<float> Sorted;
	for (float z : MinZ)
		if (z < 1e8f) Sorted.Add(z);
	Sorted.Sort();
	const float Sill = Sorted.Num() ? Sorted[Sorted.Num() / 10] : 0.f;
	float AxleX[2] = {CarLength * 0.3f, -CarLength * 0.3f}, ArchTop[2] = {0, 0};
	for (int32 Half = 0; Half < 2; Half++)
	{
		// longest run of raised bins in the front (0) / rear (1) half
		int32 BestLen = 0, BestStart = 0, Run = 0;
		for (int32 b = 0; b < NB; b++)
		{
			const float x = Body.Min.X + (b + 0.5f) * CarLength / NB;
			const bool bIn = (Half == 0 ? x > CarLength * 0.08f : x < -CarLength * 0.08f) && MinZ[b] < 1e8f && MinZ[b] > Sill + 0.18f * H;
			Run = bIn ? Run + 1 : 0;
			if (Run > BestLen) BestLen = Run, BestStart = b - Run + 1;
		}
		if (BestLen * CarLength / NB > 30.f)
		{
			// the axle sits under the top of the arch
			int32 Apex = BestStart;
			for (int32 b = BestStart; b < BestStart + BestLen; b++)
				if (MinZ[b] > ArchTop[Half]) ArchTop[Half] = MinZ[b], Apex = b;
			int32 A0 = Apex, A1 = Apex;  // centre of the flat top (several bins at the same height)
			while (A1 + 1 < BestStart + BestLen && MinZ[A1 + 1] > ArchTop[Half] - 2.f) A1++;
			while (A0 - 1 >= BestStart && MinZ[A0 - 1] > ArchTop[Half] - 2.f) A0--;
			AxleX[Half] = Body.Min.X + ((A0 + A1) * 0.5f + 0.5f) * CarLength / NB;
		}
	}
	const float Clearance = 14.f;
	if (ArchTop[0] > 0 && ArchTop[1] > 0) WheelRadius = FMath::Clamp((0.5f * (ArchTop[0] + ArchTop[1]) - Sill + Clearance - 6.f) * 0.5f, 26.f, 40.f);
	else WheelRadius = 31.f;
	Transform(FTransform(FVector(0, 0, Clearance - Sill)));  // the sills sit `Clearance` above the ground
	// full mesh -> car transform for parts loaded later (tuning swaps): scale, frame, lift
	ImportFrame = FTransform(FQuat::Identity, FVector::ZeroVector, FVector(Scale)) * ImportFrame * FTransform(FVector(0, 0, Clearance - Sill));
	ttg::build_wheel(WheelMesh, WheelRadius, 22.0);
	Wheels.SetNum(4);
	const float Track = FMath::Max(50.f, CarWidth * 0.5f - 13.f);
	for (int32 i = 0; i < 4; i++) Wheels[i].Mount = FVector(AxleX[i < 2 ? 0 : 1], (i % 2) ? Track : -Track, 0.f);
	// 4. components
	for (int32 i = 0; i < Parts.Num(); i++)
	{
		CreatePartComponent(i);
		BuildPartMesh(i, true);
	}
	// 5. collision boxes from the real size: body below the belt, cabin above
	Chassis->SetBoxExtent(FVector(CarLength * 0.5f - 6.f, CarWidth * 0.5f - 6.f, 48.f));
	const float RoofTop = H + Clearance - Sill;
	if (RoofTop > 128.f) Cabin->SetBoxExtent(FVector(CarLength * 0.16f, CarWidth * 0.36f, (RoofTop - 118.f) * 0.5f)), Cabin->SetRelativeLocation(FVector(-CarLength * 0.05f, 0, (RoofTop + 116.f) * 0.5f - ZC));
	else Cabin->SetBoxExtent(FVector(CarLength * 0.16f, CarWidth * 0.36f, 4.f)), Cabin->SetRelativeLocation(FVector(-CarLength * 0.05f, 0, RoofTop - 6.f - ZC));
	UE_LOG(LogTemp, Log, TEXT("TrueTorque: car %s: %.0f x %.0f cm, wheels r=%.0f at x=%.0f/%.0f, %d parts"), *D.Name, CarLength, CarWidth, WheelRadius, AxleX[0], AxleX[1], Parts.Num());
}

void ATTCar::SetupWheels()
{
	const float MountZ = WheelRadius - ZC + StaticSpring;
	for (int32 i = 0; i < 4; i++)
	{
		FWheel& W = Wheels[i];
		W.bFront = i < 2;
		W.bLeft = (i % 2) == 0;
		W.Mount.Z = MountZ;
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
			for (double y : {-11.0, 11.0}) Hull.Add(FVector(WheelRadius * FMath::Cos(a), y, WheelRadius * FMath::Sin(a)));
		}
		C->ClearCollisionConvexMeshes();
		C->AddCollisionConvexMesh(Hull);
		if (W.bLeft) C->SetRelativeScale3D(FVector(1, -1, 1));
	}
	HeadL->SetRelativeLocation(FVector(CarLength * 0.47f, -CarWidth * 0.32f, 70.f));
	HeadR->SetRelativeLocation(FVector(CarLength * 0.47f, CarWidth * 0.32f, 70.f));
}

void ATTCar::BuildPartMesh(int32 Index, bool bCreate)
{
	FPart& P = Parts[Index];
	UProceduralMeshComponent* C = PartComps[Index];
	const int32 NS = FMath::Max(1, P.Materials.Num());
	TArray<TArray<FVector>> V, N;
	TArray<TArray<int32>> I;
	TArray<TArray<FVector2D>> UV;
	TArray<TArray<FLinearColor>> Col;
	TArray<TArray<FProcMeshTangent>> Tan;
	V.SetNum(NS), N.SetNum(NS), I.SetNum(NS), UV.SetNum(NS), Col.SetNum(NS), Tan.SetNum(NS);
	std::vector<ttg::V> PN;
	P.Mesh.point_normals(PN);
	const bool bOwnUV = P.Mesh.u0.size() == P.Mesh.base.size();
	for (const ttg::Tri& T : P.Mesh.tris)
	{
		const int32 S = FMath::Min(int32(T.slot), NS - 1);
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
			UV[S].Add(bOwnUV ? FVector2D(P.Mesh.u0[c], P.Mesh.v0[c]) : FVector2D(P.Mesh.base[c].x / 100.0, P.Mesh.base[c].z / 100.0));
			// R: paint gone, G: how creased the metal is (the car materials read these)
			Col[S].Add(FLinearColor(P.Mesh.scrape[c], FMath::Min(1.f, float(ttg::len(P.Mesh.off[c])) / 15.f), 0.f, 1.f));
			Tan[S].Add(FProcMeshTangent(Tx.GetSafeNormal(), false));
		}
	}
	if (bCreate) C->ClearAllMeshSections();
	for (int32 S = 0; S < NS; S++)
	{
		if (bCreate)
		{
			C->CreateMeshSection_LinearColor(S, V[S], I[S], N[S], UV[S], Col[S], Tan[S], false);
			if (P.Materials.IsValidIndex(S)) C->SetMaterial(S, P.Materials[S]);
		}
		else if (V[S].Num() > 0)
			C->UpdateMeshSection_LinearColor(S, V[S], N[S], UV[S], Col[S], Tan[S]);
	}
	if (bCreate && P.bCanFall && P.Mesh.base.size())
	{
		ttg::V Lo, Hi;
		P.Mesh.bounds(Lo, Hi);
		TArray<FVector> Hull;
		for (int32 k = 0; k < 8; k++) Hull.Add(FVector((k & 1) ? Hi.x : Lo.x, (k & 2) ? Hi.y : Lo.y, (k & 4) ? Hi.z : Lo.z));
		C->ClearCollisionConvexMeshes();
		C->AddCollisionConvexMesh(Hull);
	}
	P.bDirty = false;
}

void ATTCar::ApplyVariant(int32 Part, int32 Variant)
{
	const FTTCarDef* D = Def();
	if (!D || !Parts.IsValidIndex(Part) || !D->Parts.IsValidIndex(Parts[Part].Def)) return;
	const FTTCarPartDef& PD = D->Parts[Parts[Part].Def];
	Variant = FMath::Clamp(Variant, 0, PD.VariantMeshes.Num() - 1);
	if (IsOff(Part)) SetPartDetached(Part, false, FVector::ZeroVector);
	if (LoadPartMeshes(Part, PD.VariantMeshes[Variant], true))
	{
		OrientOutward(Parts[Part].Mesh, ttg::V(0, 0, 60));
		BuildPartMesh(Part, true);
	}
	Parts[Part].Health = 1.f, Parts[Part].Fatigue = 0.f, Parts[Part].bCracked = Parts[Part].bTaped = false;
	if (AppliedVariant.IsValidIndex(Part)) AppliedVariant[Part] = uint8(Variant);
}

void ATTCar::OnRep_Variant()
{
	for (int32 i = 0; i < PartVariant.Num() && i < AppliedVariant.Num(); i++)
		if (PartVariant[i] != AppliedVariant[i]) ApplyVariant(i, PartVariant[i]);
}

// ------------------------------------------------------------------ tick

void ATTCar::Tick(float Dt)
{
	Super::Tick(Dt);
	if (Parts.Num() == 0 || Wheels.Num() != 4) return;
	Dt = FMath::Min(Dt, 1.f / 30.f);
	if (IsLocallyControlled())
	{
		if (!HasAuthority()) ServerInput(InThrottle, InBrake, InSteer, InHandbrake);
		// drive our own copy with fresh input; the server corrects the body
		RepThrottle = InThrottle, RepBrake = InBrake, RepSteer = InSteer, RepHandbrake = InHandbrake;
	}
	UpdatePhysics(Dt);
	if (HasAuthority()) UpdateEngineHeat(Dt);
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

void ATTCar::PossessedBy(AController* NewController)
{
	Super::PossessedBy(NewController);
	RepHandbrake = 0.f;
}

void ATTCar::UnPossessed()
{
	Super::UnPossessed();
	// nobody in it: pedals released, handbrake on
	InThrottle = InBrake = InSteer = 0.f, InHandbrake = 1.f;
	RepThrottle = RepBrake = RepSteer = 0.f, RepHandbrake = 1.f;
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
		EngineRpm = FMath::FInterpTo(EngineRpm, FMath::Max(1300.f, IdleRpm + Thr * 2800.f), Dt, 6.f);
		EngineT = Thr > 0.02f ? EngineTorque(EngineRpm) * Thr : (bAutomatic && RepHandbrake < 0.5f ? 30.f : 0.f);
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
	const float Lock = FMath::Clamp((Wheels[2].Omega - Wheels[3].Omega) * 60.f, -250.f, 250.f);
	float DriveT[4] = {0, 0, AxleT * 0.5f - Lock, AxleT * 0.5f + Lock};
	if (NDriven == 1) DriveT[2] = (DetachedWheels & 4) ? 0 : AxleT, DriveT[3] = (DetachedWheels & 8) ? 0 : AxleT;

	// ---- suspension
	FCollisionQueryParams Q(SCENE_QUERY_STAT(TTWheel), false, this);
	Q.bReturnPhysicalMaterial = true;
	const float RayLen = SuspMax + WheelRadius;
	float MaxBump = 0;
	for (int32 i = 0; i < 4; i++)
	{
		FWheel& W = Wheels[i];
		W.bContact = false;
		W.Load = 0;
		if (DetachedWheels & (1 << i)) continue;
		const FVector M = T.TransformPosition(W.Mount);
		FHitResult H;
		const bool bHit = GetWorld()->LineTraceSingleByChannel(H, M, M - Up * RayLen, ECC_Visibility, Q);
		W.SpringLen = bHit ? FMath::Clamp(float(H.Distance) - WheelRadius, 0.f, SuspMax) : SuspMax;
		const float X = (SuspMax - W.SpringLen) / 100.f;
		const float Xdot = (X - W.Compression) / Dt;
		W.Compression = X;
		W.BumpSpeed = FMath::Abs(Xdot);
		MaxBump = FMath::Max(MaxBump, W.BumpSpeed);
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
	// cracked or taped panels can shake loose on a hard bump
	if (bAuth && MaxBump > 0.9f)
		for (int32 i = 0; i < Parts.Num(); i++)
			if (Parts[i].bCanFall && !IsOff(i) && (Parts[i].bCracked || Parts[i].bTaped))
			{
				const float Chance = Parts[i].bCracked ? 0.04f : 0.01f * FMath::Clamp(FMath::Abs(Speed) / 10.f, 0.f, 3.f);
				if (FMath::FRand() < Chance) SetPartDetached(i, true, -Up * 100.f);
			}

	// ---- tyres
	const float SteerRad = FMath::DegreesToRadians(SteerAngle);
	for (int32 i = 0; i < 4; i++)
	{
		FWheel& W = Wheels[i];
		if (DetachedWheels & (1 << i)) continue;
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
		const int32 Sub = 8;
		const float Hs = Dt / Sub;
		float SumFx = 0, SumFy = 0, Smax = 0;
		for (int32 s = 0; s < Sub; s++)
		{
			const float Sr = (W.Omega * WheelRadius / 100.f - Vx) / Den;
			const float Sx = Sr / 0.11f, Sy = (Vy / Den) / 0.15f;
			const float S = FMath::Sqrt(Sx * Sx + Sy * Sy);
			const float Fm = Mu * Fz * SlipCurve(S);
			const float Fx = S > 1e-5f ? Fm * Sx / S : 0.f, Fy = S > 1e-5f ? -Fm * Sy / S : 0.f;
			W.Omega += (DriveT[i] - Fx * WheelRadius / 100.f) / WheelInertia * Hs;
			const float dB = Brake / WheelInertia * Hs;
			W.Omega = FMath::Abs(W.Omega) <= dB ? 0.f : W.Omega - FMath::Sign(W.Omega) * dB;
			SumFx += Fx, SumFy += Fy, Smax = FMath::Max(Smax, S);
			W.SlipRatio = Sr;
		}
		float Fx = SumFx / Sub, Fy = SumFy / Sub;
		if (FMath::Abs(Vx) > 0.1f) Fx -= 0.012f * Fz * FMath::Sign(Vx);
		W.Slide = Smax;
		Chassis->AddForceAtLocation((WF * Fx + WR * Fy) * 100.f, W.ContactPoint + Up * 8.0);
		if (bAuth && Loose > 0.3f)
		{
			for (int32 k = 0; k < 5; k++)
				if (NutTorque[i * 5 + k] > 0.f) NutTorque[i * 5 + k] = FMath::Max(0.f, NutTorque[i * 5 + k] - FMath::Abs(Vx) * Dt * 0.08f);
			if (WheelNutAverage(i) < 10.f) W.LooseDistance += FMath::Abs(Vx) * Dt;
			if (W.LooseDistance > 150.f) SetWheelDetached(i, true), W.LooseDistance = 0;
		}
	}
	const float V2 = float(Vel.SizeSquared());
	Chassis->AddForce(-Vel.GetSafeNormal() * (0.38f * V2 * 100.f) - Up * (0.12f * V2 * 100.f));
}

// cooling: the radiator needs coolant and air; a leak empties it, low coolant overheats the engine,
// an overheated or oil-starved engine wears out and finally seizes
void ATTCar::UpdateEngineHeat(float Dt)
{
	const float Ambient = 20.f;
	if (!bRadiatorWelded) Coolant = FMath::Max(0.f, Coolant - RadiatorLeak * Dt);
	if (EngineHealth < 0.5f) Oil = FMath::Max(0.f, Oil - (0.5f - EngineHealth) * 0.002f * Dt);
	const float Load = (EngineRpm / 7000.f) * (0.25f + 0.75f * RepThrottle);
	const float Heat = bEngineRunning ? 2.f + 10.f * Load : 0.f;
	const float Open = FMath::Clamp((EngineTemp - 85.f) / 10.f, 0.f, 1.f);  // thermostat
	const float RadEff = Coolant > 0.25f ? Coolant : 0.05f;
	const float Air = 0.6f + FMath::Min(FMath::Abs(Speed), 30.f) / 15.f;
	const float Cool = (0.15f + 0.85f * Open) * RadEff * Air * (EngineTemp - Ambient) * 0.08f + (EngineTemp - Ambient) * 0.004f;
	EngineTemp = FMath::Clamp(EngineTemp + (Heat - Cool) * Dt, Ambient - 5.f, 160.f);
	if (EngineTemp > 118.f) EngineHealth = FMath::Max(0.f, EngineHealth - (EngineTemp - 118.f) * 0.0015f * Dt);
	if (Oil < 0.3f && bEngineRunning) EngineHealth = FMath::Max(0.f, EngineHealth - 0.004f * Dt * (EngineRpm / 3000.f));
	if (bRadiatorWelded && EngineTemp > 108.f) bRadiatorWelded = false;  // cold weld gives way to heat
}

// ------------------------------------------------------------------ damage

void ATTCar::OnChassisHit(UPrimitiveComponent* HitComp, AActor* Other, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit)
{
	if (!HasAuthority() || Parts.Num() == 0) return;
	const FVector P = Hit.ImpactPoint, Nrm = Hit.ImpactNormal;  // normal points from the obstacle into us
	FVector OtherV = FVector::ZeroVector;
	if (OtherComp && OtherComp->IsSimulatingPhysics()) OtherV = OtherComp->GetPhysicsLinearVelocityAtPoint(P);
	const FVector Rel = LastVelocity - OtherV;
	const float Closing = float(FVector::DotProduct(Rel, -Nrm)) / 100.f;
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
		for (int32 i = 0; i < 4; i++)
		{
			const FVector WC = Wheels[i].Mount;
			if (FVector::Dist2D(FVector(LP.X, LP.Y, 0), FVector(WC.X, WC.Y, 0)) < 60.f && Dv > 5.f)
			{
				WheelDamage[i] += (Dv - 5.f) * 0.05f;
				if (WheelDamage[i] > 1.f) SetWheelDetached(i, true);
			}
		}
		// frontal hits reach the radiator and the engine
		if (LP.X > CarLength * 0.3f && LD.X < -0.4f)
		{
			if (Dv > 3.f) RadiatorLeak += (Dv - 3.f) * 0.0006f, bRadiatorWelded = false;
			if (Dv > 4.f) EngineHealth = FMath::Max(0.f, EngineHealth - (Dv - 4.f) * 0.035f);
		}
	}
	else if (Dv > 0.8f)
	{
		// small knocks do not dent, but the metal around them tires
		for (int32 i = 0; i < Parts.Num(); i++)
			if (!IsOff(i) && FVector::Dist(PartComps[i]->Bounds.Origin, P) < 120.f)
			{
				Parts[i].Fatigue += 0.06f * Dv;
				if (Parts[i].Fatigue > 1.f && Parts[i].bCanFall) Parts[i].bCracked = true;
			}
		if (Tangential > 3.f)
		{
			MulticastDent(LP, LD, FMath::Min(Tangential * 0.04f, 0.6f), 35.f, Seed, 1);
			LastDentTime = Now, LastDentPoint = P;
		}
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
	UParticleSystem* Sparks = Fx(TEXT("P_Sparks"));
	if (Sparks && (Kind == 1 || Depth > 6.f))
		UGameplayStatics::SpawnEmitterAtLocation(GetWorld(), Sparks, GetActorTransform().TransformPosition(LocalPoint), FRotator::ZeroRotator, FVector(Kind == 1 ? 0.4f : 0.8f));
}

void ATTCar::ApplyDent(const FVector& LocalPoint, const FVector& LocalDir, float Depth, float Radius, float Seed, uint8 Kind)
{
	if (Parts.Num() == 0) return;
	const FTransform A = GetActorTransform();
	const FVector WP = A.TransformPosition(LocalPoint), WD = A.TransformVectorNoScale(LocalDir);
	for (int32 i = 0; i < Parts.Num(); i++)
	{
		if (IsOff(i)) continue;
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
		const double Size = FMath::Max(1.0, double(P.Mesh.base.size()));
		P.Health = FMath::Max(0.f, P.Health - float(Moved / (Size * 5.0)));
		P.Fatigue += float(Moved / (Size * 2.0));
		if (HasAuthority() && P.bCanFall)
		{
			if (P.Fatigue > 1.f) P.bCracked = true;
			if ((P.Health < 0.15f && Depth > 8.f) || (P.bTaped && Depth > 3.f)) SetPartDetached(i, true, WD * -300.f);
		}
	}
}

void ATTCar::OnRep_Detached()
{
	if (Parts.Num() == 0 || Wheels.Num() != 4) return;  // before BeginPlay: applied there
	for (int32 i = 0; i < Parts.Num() && i < PartOff.Num(); i++)
	{
		const bool bOff = PartOff[i] != 0;
		if (AppliedOff.IsValidIndex(i) && bOff != (AppliedOff[i] != 0)) SetPartDetached(i, bOff, FVector::ZeroVector);
	}
	for (int32 i = 0; i < 4; i++)
	{
		const bool bOff = (DetachedWheels >> i) & 1;
		if (bOff != bool((AppliedWheels >> i) & 1)) SetWheelDetached(i, bOff);
	}
}

void ATTCar::SetPartDetached(int32 Part, bool bOff, const FVector& Kick)
{
	if (!Parts.IsValidIndex(Part) || !Parts[Part].bCanFall) return;
	UProceduralMeshComponent* C = PartComps[Part];
	if (PartOff.IsValidIndex(Part)) PartOff[Part] = bOff ? 1 : 0;
	if (AppliedOff.IsValidIndex(Part)) AppliedOff[Part] = bOff ? 1 : 0;
	if (bOff)
	{
		C->DetachFromComponent(FDetachmentTransformRules::KeepWorldTransform);
		C->SetCollisionProfileName(TEXT("PhysicsActor"));
		C->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
		C->SetSimulatePhysics(true);
		C->SetMassOverrideInKg(NAME_None, Part == EnginePart ? 16.f : 8.f, true);
		C->SetPhysicsLinearVelocity(Chassis->GetPhysicsLinearVelocity() + Kick);
		C->AddAngularImpulseInDegrees(FVector(FMath::FRandRange(-200.f, 200.f), FMath::FRandRange(-200.f, 200.f), 0.f), NAME_None, true);
	}
	else
	{
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
			const FQuat Qw = C->GetComponentQuat();
			C->SetWorldScale3D(FVector::OneVector);
			C->SetWorldRotation(Qw * FQuat(FVector::UpVector, PI));
		}
		C->SetCollisionProfileName(TEXT("PhysicsActor"));
		C->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
		C->SetSimulatePhysics(true);
		C->SetMassOverrideInKg(NAME_None, 20.f, true);
		C->SetPhysicsLinearVelocity(V);
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
			Parts[i].Health = 1.f, Parts[i].Fatigue = 0.f;
			Parts[i].bCracked = Parts[i].bTaped = false;
			Parts[i].bDirty = true;
		}
}

// ------------------------------------------------------------------ visuals

void ATTCar::UpdateVisuals(float Dt)
{
	int32 Built = 0;
	for (int32 i = 0; i < Parts.Num() && Built < 2; i++)
		if (Parts[i].bDirty) BuildPartMesh(i, false), Built++;
	for (int32 i = 0; i < 4; i++)
	{
		FWheel& W = Wheels[i];
		if (DetachedWheels & (1 << i)) continue;
		W.Spin = FMath::Fmod(W.Spin + W.Omega * Dt, 2.f * PI);
		const float Loose = 1.f - FMath::Clamp(WheelNutAverage(i) / NutSpec, 0.f, 1.f);
		const float Bend = FMath::Clamp(WheelDamage[i], 0.f, 1.f);
		const FVector Centre = W.Mount - FVector(0, 0, W.SpringLen) + FVector(0, 0, ZC);
		const float Yaw = (W.bFront ? SteerAngle : 0.f) + Bend * 4.f * (W.bLeft ? -1.f : 1.f);
		const float Camber = Bend * 6.f * (W.bLeft ? 1.f : -1.f) + Loose * 4.f * FMath::Sin(W.Spin);
		WheelComps[i]->SetRelativeLocationAndRotation(Centre, FRotator(-FMath::RadiansToDegrees(W.Spin), Yaw, Camber));
	}
	const bool bBraking = RepBrake > 0.05f && Gear >= 0;
	if (Mats.Num() >= 5)
	{
		Mats[3]->SetScalarParameterValue(TEXT("Glow"), bLights ? 30.f : 0.4f);
		Mats[4]->SetScalarParameterValue(TEXT("Glow"), bBraking ? 40.f : bLights ? 8.f : 0.5f);
	}
	HeadL->SetVisibility(bLights);
	HeadR->SetVisibility(bLights);
	// effects: steam from a hot or leaking cooling system, smoke from a dying engine, tyre smoke
	auto Toggle = [](UParticleSystemComponent* C, bool bOn) {
		if (!C) return;
		if (bOn && !C->IsActive()) C->Activate(true);
		else if (!bOn && C->IsActive()) C->Deactivate();
	};
	const bool bHoodOn = EnginePart < 0 || !IsOff(EnginePart);
	Toggle(Steam, EngineTemp > 112.f || (RadiatorLeak > 0.f && !bRadiatorWelded && Coolant > 0.02f && bEngineRunning));
	Toggle(Smoke, bEngineRunning && EngineHealth < 0.4f);
	for (int32 k = 0; k < TyreSmoke.Num(); k++)
		Toggle(TyreSmoke[k], Wheels[2 + k].bContact && Wheels[2 + k].Slide > 1.6f && FMath::Abs(Wheels[2 + k].Omega * WheelRadius / 100.f) > 4.f);
	(void)bHoodOn;
	// dashboard (no HUD: the car's own instruments)
	DashSpeed->SetText(FText::FromString(FString::Printf(TEXT("%d"), FMath::RoundToInt(FMath::Abs(Speed) * 3.6f))));
	const TCHAR* GearName[] = {TEXT("R"), TEXT("N"), TEXT("1"), TEXT("2"), TEXT("3"), TEXT("4"), TEXT("5"), TEXT("6")};
	FString Warn;
	if (EngineTemp > 105.f) Warn += TEXT("  ПЕРЕГРЕВ");
	if (Oil < 0.3f) Warn += TEXT("  МАСЛО");
	if (!bEngineRunning) Warn += TEXT("  ЗАГЛОХ");
	DashInfo->SetText(FText::FromString(FString::Printf(TEXT("%s%s  %4.0f об  %3.0f°C%s"), GearName[FMath::Clamp(Gear + 1, 0, 7)], bAutomatic ? TEXT(" A") : TEXT(""),
	                                                    EngineRpm, EngineTemp, *Warn)));
	float Slide = 0;
	for (int32 i = 0; i < 4; i++)
		if (Wheels[i].bContact) Slide = FMath::Max(Slide, Wheels[i].Slide);
	Sound->SetState(EngineRpm, RepThrottle, 1.f - EngineHealth, Slide - 1.2f, FMath::Abs(Speed), bEngineRunning);
	if (NoteTimer > 0 && (NoteTimer -= Dt) <= 0) WorldNote->SetVisibility(false);
	if (WorldNote->IsVisible())
		if (const APlayerController* PC = GetWorld()->GetFirstPlayerController())
			if (PC->PlayerCameraManager)
				WorldNote->SetWorldRotation((PC->PlayerCameraManager->GetCameraLocation() - WorldNote->GetComponentLocation()).Rotation());
}

void ATTCar::UpdateCamera(float Dt)
{
	if (View == ETTView::Cockpit) return;
	const float CarYaw = GetActorRotation().Yaw;
	CamYaw = FMath::RadiansToDegrees(FMath::Atan2(FMath::Sin(FMath::DegreesToRadians(CamYaw)), FMath::Cos(FMath::DegreesToRadians(CamYaw))));
	const float Delta = FRotator::NormalizeAxis(CarYaw - CamYaw);
	CamYaw += Delta * FMath::Min(1.f, Dt * (View == ETTView::Mechanic ? 0.f : 4.f));
	Arm->SetWorldRotation(FRotator(LookPitch, CamYaw + LookYaw, 0.f));
}

// ------------------------------------------------------------------ input

void ATTCar::SetupInput()
{
	if (Mapping) return;  // re-possessed: actions already exist
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
	const FName Names[] = {"ShiftUp", "ShiftDown", "Camera", "Mechanic", "Reset", "Use", "Loosen", "Gearbox", "Lights", "Exit"};
	const FKey Keys[] = {EKeys::E, EKeys::Q, EKeys::C, EKeys::Tab, EKeys::R, EKeys::LeftMouseButton, EKeys::RightMouseButton, EKeys::M, EKeys::L, EKeys::F};
	for (int32 i = 0; i < int32(UE_ARRAY_COUNT(Names)); i++) Mapping->MapKey(Act(Names[i], EInputActionValueType::Boolean), Keys[i]);
}

void ATTCar::SetupPlayerInputComponent(UInputComponent* PIC)
{
	Super::SetupPlayerInputComponent(PIC);
	SetupInput();
	if (APlayerController* PC = Cast<APlayerController>(GetController()))
		if (ULocalPlayer* LP = PC->GetLocalPlayer())
			if (UEnhancedInputLocalPlayerSubsystem* Sub = LP->GetSubsystem<UEnhancedInputLocalPlayerSubsystem>())
			{
				Sub->ClearAllMappings();
				Sub->AddMappingContext(Mapping, 0);
			}
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
	E->BindAction(Actions["Gearbox"], ETriggerEvent::Started, this, &ATTCar::OnGearbox);
	E->BindAction(Actions["Lights"], ETriggerEvent::Started, this, &ATTCar::OnLightsKey);
	E->BindAction(Actions["Exit"], ETriggerEvent::Started, this, &ATTCar::OnExit);
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
void ATTCar::OnShiftUp() { Command(TTCmd::ShiftUp, 0); }
void ATTCar::OnShiftDown() { Command(TTCmd::ShiftDown, 0); }
void ATTCar::OnGearbox() { Command(TTCmd::Gearbox, 0); }
void ATTCar::OnLightsKey() { Command(TTCmd::Lights, 0); }
void ATTCar::OnReset() { Command(TTCmd::Reset, 0); }
void ATTCar::OnCamera()
{
	if (View == ETTView::Mechanic) return;
	View = View == ETTView::Chase ? ETTView::Cockpit : ETTView::Chase;
	LookYaw = 0, LookPitch = View == ETTView::Chase ? -10.f : 0.f;
	ApplyView();
}
void ATTCar::OnMechanic()
{
	if (View != ETTView::Mechanic && FMath::Abs(Speed) > 1.f) return;
	View = View == ETTView::Mechanic ? ETTView::Chase : ETTView::Mechanic;
	LookYaw = 0, LookPitch = -20.f;
	ApplyView();
	if (View == ETTView::Mechanic)
		Note(TEXT("ЛКМ: затянуть гайку   ПКМ: открутить\nF — выйти из машины: ремонт руками"), GetActorLocation() + FVector(0, 0, 230), 5.f);
}
void ATTCar::OnExit()
{
	if (FMath::Abs(Speed) > 3.f) return;
	if (HasAuthority()) ServerExit_Implementation();
	else ServerExit();
}

FVector ATTCar::DriverDoor() const
{
	return GetActorTransform().TransformPosition(FVector(-10.f, -(CarWidth * 0.5f + 70.f), 30.f));
}

void ATTCar::ServerExit_Implementation()
{
	AController* C = GetController();
	if (!C || FMath::Abs(Speed) > 3.f) return;
	FActorSpawnParameters P;
	P.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
	ATTDriver* D = GetWorld()->SpawnActor<ATTDriver>(ATTDriver::StaticClass(), DriverDoor(), FRotator(0.f, GetActorRotation().Yaw, 0.f), P);
	if (!D) return;
	C->UnPossess();
	C->Possess(D);
	C->SetControlRotation(FRotator(-10.f, GetActorRotation().Yaw + 90.f, 0.f));
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

bool ATTCar::CameraRay(FVector& Eye, FVector& Dir) const
{
	const APlayerController* PC = Cast<APlayerController>(GetController());
	if (!PC || !PC->PlayerCameraManager) return false;
	Eye = PC->PlayerCameraManager->GetCameraLocation(), Dir = PC->PlayerCameraManager->GetCameraRotation().Vector();
	return true;
}

int32 ATTCar::AimedWheel(const FVector& Eye, const FVector& Dir) const
{
	int32 Best = -1;
	double Bd = 60.0;
	for (int32 i = 0; i < 4 && i < Wheels.Num(); i++)
	{
		const FVector C = (DetachedWheels & (1 << i)) ? Chassis->GetComponentTransform().TransformPosition(Wheels[i].Mount) : WheelComps[i]->GetComponentLocation();
		if (FVector::DotProduct(C - Eye, Dir) < 0) continue;
		const double d = FMath::PointDistToLine(C, Dir, Eye);
		if (d < Bd) Bd = d, Best = i;
	}
	return Best;
}

int32 ATTCar::AimedPart(const FVector& Eye, const FVector& Dir) const
{
	int32 Best = -1;
	double Bd = 90.0;
	for (int32 i = 0; i < Parts.Num(); i++)
	{
		if (Parts[i].Mesh.base.empty()) continue;
		const FTransform& Tf = IsOff(i) ? Visual->GetComponentTransform() : PartComps[i]->GetComponentTransform();
		const FVector C = Tf.TransformPosition(ToF(Parts[i].Mesh.centre()));
		if (FVector::DotProduct(C - Eye, Dir) < 0) continue;
		const double d = FMath::PointDistToLine(C, Dir, Eye) + (Parts[i].bCanFall ? 0.0 : 35.0);  // panels win over the shell
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
	case TTCmd::Tighten:
		if (bParked && Index >= 0 && Index < 4 && !(DetachedWheels & (1 << Index)))
		{
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
	case TTCmd::Loosen:
		if (bParked && Index >= 0 && Index < 4)
			for (int32 k = 0; k < 5; k++)
				if (NutTorque[Index * 5 + k] > 0.f) NutTorque[Index * 5 + k] = 0.f;
		break;
	case TTCmd::RepairPart:
		if (!bParked) break;
		if (Index >= 100 && Index < 104)
		{
			// refit or straighten a wheel: new studs, nuts finger-tight (torque them!)
			const int32 W = Index - 100;
			if (DetachedWheels & (1 << W)) SetWheelDetached(W, false);
			WheelDamage[W] = 0.f;
			for (int32 k = 0; k < 5; k++) NutTorque[W * 5 + k] = 0.f;
		}
		else if (Parts.IsValidIndex(Index))
		{
			if (IsOff(Index)) SetPartDetached(Index, false, FVector::ZeroVector);
			MulticastRepair(Index);
		}
		break;
	case TTCmd::Fluids:
		if (bParked) Coolant = 1.f, Oil = 1.f;
		break;
	case TTCmd::EngineRebuild:
		if (bParked) EngineHealth = 1.f, bEngineRunning = true, RadiatorLeak = 0.f, bRadiatorWelded = false, EngineTemp = FMath::Min(EngineTemp, 60.f);
		break;
	case TTCmd::Tape:
		if (bParked && Parts.IsValidIndex(Index) && Parts[Index].bCanFall)
		{
			// quick and dirty: holds a fallen or cracked panel until the next hard knock
			if (IsOff(Index)) SetPartDetached(Index, false, FVector::ZeroVector);
			Parts[Index].bTaped = true, Parts[Index].bCracked = false;
			Parts[Index].Health = FMath::Max(Parts[Index].Health, 0.3f);
		}
		break;
	case TTCmd::ColdWeld:
		if (!bParked) break;
		if (Index < 0) bRadiatorWelded = RadiatorLeak > 0.f;
		else if (Parts.IsValidIndex(Index) && !IsOff(Index)) Parts[Index].bCracked = false, Parts[Index].Fatigue = FMath::Min(Parts[Index].Fatigue, 0.6f);
		break;
	case TTCmd::SwapVariant:
		if (bParked && Parts.IsValidIndex(Index) && PartVariant.IsValidIndex(Index))
			if (const FTTCarDef* D = Def())
				if (D->Parts.IsValidIndex(Parts[Index].Def) && D->Parts[Parts[Index].Def].VariantNames.Num() > 1)
				{
					PartVariant[Index] = uint8((PartVariant[Index] + 1) % D->Parts[Parts[Index].Def].VariantNames.Num());
					ApplyVariant(Index, PartVariant[Index]);
				}
		break;
	case TTCmd::Reset:
	{
		const FRotator R(0.f, GetActorRotation().Yaw, 0.f);
		Chassis->SetWorldLocationAndRotation(GetActorLocation() + FVector(0, 0, 120), R, false, nullptr, ETeleportType::TeleportPhysics);
		Chassis->SetPhysicsLinearVelocity(FVector::ZeroVector);
		Chassis->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
		break;
	}
	case TTCmd::ShiftUp:
		bAutomatic = false;
		if (Gear < 6) Gear++, ShiftTimer = 0.2f;
		break;
	case TTCmd::ShiftDown:
		bAutomatic = false;
		if (Gear > -1 && (Gear > 0 || Speed < 1.f)) Gear--, ShiftTimer = 0.2f;
		break;
	case TTCmd::Gearbox:
		bAutomatic = !bAutomatic;
		if (bAutomatic && Gear == 0) Gear = 1;
		break;
	case TTCmd::Lights:
		bLights = !bLights;
		break;
	default:
		break;
	}
}

// what the mechanic sees and hears (runs on the player's machine, values predicted)
void ATTCar::ShowCommand(uint8 Cmd, int32 Index)
{
	const FVector Above = GetActorLocation() + FVector(0, 0, 200);
	auto PartName = [&](int32 i) { return Parts.IsValidIndex(i) ? Parts[i].Name : FString(TEXT("?")); };
	switch (Cmd)
	{
	case TTCmd::Tighten:
	{
		if (Index < 0 || Index > 3) return;
		int32 k = 0;
		while (k < 5 && (NutTorque[Index * 5 + k] < 0.f || NutTorque[Index * 5 + k] >= NutSpec)) k++;
		FString Txt;
		for (int32 n = 0; n < 5; n++)
		{
			const float Nt = NutTorque[Index * 5 + n] + (n == k ? 20.f : 0.f);
			Txt += NutTorque[Index * 5 + n] < 0.f ? TEXT(" X ") : FString::Printf(TEXT(" %.0f "), FMath::Min(Nt, 999.f));
		}
		if (k < 5 && NutTorque[Index * 5 + k] + 20.f >= NutSpec) Sound->Click();
		Note(TEXT("гайки, Н·м:") + Txt + (k == 5 ? TEXT("\nвсе затянуты (дальше — сорвёшь резьбу)") : TEXT("")), WheelComps[Index]->GetComponentLocation() + FVector(0, 0, 70), 2.5f);
		break;
	}
	case TTCmd::Loosen: Note(TEXT("гайки откручены"), Above, 2.f); break;
	case TTCmd::RepairPart:
		Note(Index >= 100 ? FString(TEXT("колесо на месте — затяни гайки")) : PartName(Index) + TEXT(": выправлено / поставлено"), Above, 2.5f);
		break;
	case TTCmd::Fluids: Note(FString::Printf(TEXT("долито: антифриз (было %.0f%%), масло (было %.0f%%)"), Coolant * 100.f, Oil * 100.f), Above, 3.f); break;
	case TTCmd::EngineRebuild: Note(FString::Printf(TEXT("двигатель перебран (было %.0f%%), радиатор новый"), EngineHealth * 100.f), Above, 3.f); break;
	case TTCmd::Tape: Note(PartName(Index) + TEXT(": замотано изолентой — до первой серьёзной кочки"), Above, 3.f); break;
	case TTCmd::ColdWeld:
		Note(Index < 0 ? FString(RadiatorLeak > 0.f ? TEXT("радиатор заварен холодной сваркой — не перегревай") : TEXT("радиатор не течёт")) : PartName(Index) + TEXT(": трещина заварена"),
		     Above, 3.f);
		break;
	case TTCmd::SwapVariant:
		if (const FTTCarDef* D = Def())
			if (Parts.IsValidIndex(Index) && D->Parts.IsValidIndex(Parts[Index].Def))
			{
				const FTTCarPartDef& PD = D->Parts[Parts[Index].Def];
				const int32 Next = PD.VariantNames.Num() ? (PartVariant[Index] + 1) % PD.VariantNames.Num() : 0;
				Note(PD.VariantNames.Num() > 1 ? TEXT("ставлю: ") + PD.VariantNames[Next] : PartName(Index) + TEXT(": других вариантов нет"), Above, 2.5f);
			}
		break;
	default: break;
	}
}

void ATTCar::OnUse()
{
	FVector Eye, Dir;
	if (View != ETTView::Mechanic || !CameraRay(Eye, Dir)) return;
	const int32 W = AimedWheel(Eye, Dir);
	if (W < 0 || (DetachedWheels & (1 << W))) return;
	ShowCommand(TTCmd::Tighten, W);
	Command(TTCmd::Tighten, W);
}

void ATTCar::OnLoosen()
{
	FVector Eye, Dir;
	if (View != ETTView::Mechanic || !CameraRay(Eye, Dir)) return;
	const int32 W = AimedWheel(Eye, Dir);
	if (W < 0) return;
	ShowCommand(TTCmd::Loosen, W);
	Command(TTCmd::Loosen, W);
}
