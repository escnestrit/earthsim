#include "TTGameMode.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/ExponentialHeightFogComponent.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Components/PointLightComponent.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Components/SkyLightComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/VolumetricCloudComponent.h"
#include "Engine/DirectionalLight.h"
#include "Engine/ExponentialHeightFog.h"
#include "Engine/PointLight.h"
#include "Engine/PostProcessVolume.h"
#include "Engine/SkyLight.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Components/SkyAtmosphereComponent.h"
#include "SkyAtmosphere.h"
#include "VolumetricCloud.h"
#include "TTVehicle.h"

ATTGameMode::ATTGameMode()
{
	DefaultPawnClass = ATTVehicle::StaticClass();
	GameStateClass = ATTGameState::StaticClass();
	PlayerControllerClass = APlayerController::StaticClass();
}

APawn* ATTGameMode::SpawnDefaultPawnFor_Implementation(AController* NewPlayer, AActor* StartSpot)
{
	// cars line up on the start straight
	const int32 i = Spawned++;
	// grid on the bottom straight of the oval (y = -380 m), facing +X
	const FVector Loc((-20.f - (i / 2) * 9.f) * 100.f, (-380.f + (i % 2 ? 2.5f : -2.5f)) * 100.f, 60.f);
	FActorSpawnParameters P;
	P.Instigator = GetInstigator();
	P.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	ATTVehicle* V = GetWorld()->SpawnActor<ATTVehicle>(ATTVehicle::StaticClass(), Loc, FRotator::ZeroRotator, P);
	if (V)
	{
		const FLinearColor Paints[] = {{0.42f, 0.03f, 0.02f}, {0.02f, 0.08f, 0.35f}, {0.7f, 0.7f, 0.68f}, {0.02f, 0.02f, 0.02f}, {0.05f, 0.25f, 0.08f}, {0.6f, 0.4f, 0.02f}};
		V->PaintColor = Paints[i % 6];
	}
	return V;
}

// ------------------------------------------------------------------ world

UMaterialInstanceDynamic* ATTGameState::Mat(const FLinearColor& C)
{
	UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	UMaterialInstanceDynamic* M = UMaterialInstanceDynamic::Create(Base, this);
	M->SetVectorParameterValue(TEXT("Color"), C);
	Mats.Add(M);
	return M;
}

AActor* ATTGameState::Box(const FVector& Centre, const FVector& SizeM, const FRotator& Rot, const FLinearColor& Col, bool bPhysics, const TCHAR* Shape)
{
	UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, *FString::Printf(TEXT("/Engine/BasicShapes/%s.%s"), Shape, Shape));
	FActorSpawnParameters P;
	P.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	AStaticMeshActor* A = GetWorld()->SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass(), Centre * 100.0, Rot, P);
	if (!A) return nullptr;
	A->SetReplicates(false);
	UStaticMeshComponent* C = A->GetStaticMeshComponent();
	C->SetMobility(bPhysics ? EComponentMobility::Movable : EComponentMobility::Static);
	C->SetStaticMesh(Mesh);
	C->SetWorldScale3D(SizeM);
	C->SetMaterial(0, Mat(Col));
	C->SetCollisionProfileName(bPhysics ? TEXT("PhysicsActor") : TEXT("BlockAll"));
	if (bPhysics)
	{
		C->SetSimulatePhysics(true);
		C->SetMassOverrideInKg(NAME_None, 40.f, true);
	}
	return A;
}

void ATTGameState::BeginPlay()
{
	Super::BeginPlay();
	BuildWorld();
}

void ATTGameState::BuildWorld()
{
	UWorld* W = GetWorld();
	FActorSpawnParameters P;
	// ---- sky: physical atmosphere, sun, real-time sky light, volumetric clouds, height fog
	ADirectionalLight* Sun = W->SpawnActor<ADirectionalLight>(FVector(0, 0, 50000), FRotator(-38, 35, 0), P);
	if (Sun)
	{
		UDirectionalLightComponent* L = Cast<UDirectionalLightComponent>(Sun->GetLightComponent());
		L->SetMobility(EComponentMobility::Movable);
		L->SetIntensity(9.f);
		L->SetAtmosphereSunLight(true);
		L->SetCastShadows(true);
		L->SetLightSourceAngle(0.6f);
		L->bCastCloudShadows = true;
	}
	W->SpawnActor<ASkyAtmosphere>(FVector::ZeroVector, FRotator::ZeroRotator, P);
	ASkyLight* Sky = W->SpawnActor<ASkyLight>(FVector(0, 0, 1000), FRotator::ZeroRotator, P);
	if (Sky)
	{
		USkyLightComponent* S = Sky->GetLightComponent();
		S->SetMobility(EComponentMobility::Movable);
		S->bRealTimeCapture = true;
		S->SetIntensity(1.0f);
		S->RecaptureSky();
	}
	W->SpawnActor<AVolumetricCloud>(FVector::ZeroVector, FRotator::ZeroRotator, P);
	AExponentialHeightFog* Fog = W->SpawnActor<AExponentialHeightFog>(FVector(0, 0, -1000), FRotator::ZeroRotator, P);
	if (Fog)
	{
		UExponentialHeightFogComponent* F = Fog->GetComponent();
		F->SetFogDensity(0.012f);
		F->SetVolumetricFog(true);
		F->SetFogHeightFalloff(0.15f);
	}
	APostProcessVolume* PP = W->SpawnActor<APostProcessVolume>(FVector::ZeroVector, FRotator::ZeroRotator, P);
	if (PP)
	{
		PP->bUnbound = true;
		PP->Settings.bOverride_AutoExposureMinBrightness = true;
		PP->Settings.AutoExposureMinBrightness = 0.8f;
		PP->Settings.bOverride_AutoExposureMaxBrightness = true;
		PP->Settings.AutoExposureMaxBrightness = 4.f;
		PP->Settings.bOverride_BloomIntensity = true;
		PP->Settings.BloomIntensity = 0.4f;
		PP->Settings.bOverride_VignetteIntensity = true;
		PP->Settings.VignetteIntensity = 0.3f;
		PP->Settings.bOverride_SceneFringeIntensity = true;
		PP->Settings.SceneFringeIntensity = 0.3f;
	}

	const FLinearColor Asphalt(0.045f, 0.045f, 0.048f), Grass(0.06f, 0.12f, 0.03f), Concrete(0.32f, 0.31f, 0.29f), Kerb1(0.5f, 0.03f, 0.02f), Kerb2(0.75f, 0.75f, 0.72f),
	    Line(0.8f, 0.8f, 0.78f), Rust(0.25f, 0.08f, 0.03f), Wall(0.45f, 0.42f, 0.38f), Roof(0.15f, 0.16f, 0.17f);
	// ---- ground: grass field, big asphalt skid pad
	Box(FVector(0, 0, -0.52), FVector(4000, 4000, 1), FRotator::ZeroRotator, Grass, false, TEXT("Cube"));
	Box(FVector(0, 0, -0.25), FVector(300, 300, 0.5), FRotator::ZeroRotator, Asphalt);
	// ---- circuit: an oval with a chicane, track pieces whose tops are flush with the ground plane
	const int32 Seg = 96;
	const double Rx = 700, Ry = 380;
	for (int32 i = 0; i < Seg; i++)
	{
		const double a0 = 2 * PI * i / Seg, a1 = 2 * PI * (i + 1) / Seg;
		auto Pt = [&](double a) { return FVector(Rx * FMath::Cos(a), Ry * FMath::Sin(a) + (FMath::Abs(FMath::Sin(a * 2)) < 0.2 ? 15 * FMath::Sin(a * 6) : 0), 0); };
		const FVector A = Pt(a0), B = Pt(a1);
		const FVector Mid = (A + B) * 0.5;
		const FVector D = B - A;
		const double L = D.Size();
		const FRotator R(0, FMath::RadiansToDegrees(FMath::Atan2(D.Y, D.X)), 0);
		Box(FVector(Mid.X, Mid.Y, -0.25), FVector(L + 2.5, 14, 0.5), R, Asphalt);
		// kerbs on both edges, red and white
		const FVector N = FVector(-D.Y, D.X, 0).GetSafeNormal();
		for (int32 s : {-1, 1})
			Box(FVector(Mid.X, Mid.Y, 0.03) + N * (7.6 * s), FVector(L + 0.2, 1.2, 0.12), R, (i % 2) ? Kerb1 : Kerb2);
		// walls on the outside of the bends
		// tyre walls on the outside of the bends (N points into the oval)
		if (FMath::Abs(FMath::Cos(a0)) > 0.85) Box(FVector(Mid.X, Mid.Y, 0.6) - N * 14.0, FVector(L + 0.5, 0.6, 1.2), R, Concrete);
	}
	// start line on the bottom straight (cars face +X there)
	Box(FVector(0, -Ry, 0.01), FVector(0.6, 14, 0.02), FRotator::ZeroRotator, Line);
	// ---- crash zone: a concrete wall, a row of barrels, a car-sized block
	Box(FVector(220, -150, 1.5), FVector(1.0, 24, 3), FRotator::ZeroRotator, Concrete);
	for (int32 i = 0; i < 12; i++) Box(FVector(140 + (i % 4) * 1.4, 80 + (i / 4) * 1.4, 0.45), FVector(0.6, 0.6, 0.9), FRotator::ZeroRotator, Rust, true, TEXT("Cylinder"));
	// ---- ramps, kerbs and speed bumps on the pad
	Box(FVector(60, 120, 0.9), FVector(12, 6, 0.4), FRotator(10, 0, 0), Concrete);
	Box(FVector(60, 150, 1.4), FVector(14, 6, 0.4), FRotator(-8, 180, 0), Concrete);
	for (int32 i = 0; i < 6; i++) Box(FVector(-60 + i * 12, -120, 0.06), FVector(1.2, 10, 0.12), FRotator::ZeroRotator, (i % 2) ? Kerb1 : Kerb2);
	for (int32 i = 0; i < 4; i++) Box(FVector(-100, 60 + i * 20, 0.05), FVector(0.8, 10, 0.1), FRotator::ZeroRotator, Line, false, TEXT("Cylinder"));
	// ---- garage (workshop) next to the start
	const FVector G(-60, -Ry + 36, 0);
	Box(G + FVector(0, 0, -0.2), FVector(24, 18, 0.4), FRotator::ZeroRotator, Concrete);
	Box(G + FVector(0, 9, 3), FVector(24, 0.4, 6), FRotator::ZeroRotator, Wall);
	Box(G + FVector(-12, 0, 3), FVector(0.4, 18, 6), FRotator::ZeroRotator, Wall);
	Box(G + FVector(12, 4, 3), FVector(0.4, 10, 6), FRotator::ZeroRotator, Wall);
	Box(G + FVector(0, 0, 6.2), FVector(24.6, 18.6, 0.4), FRotator::ZeroRotator, Roof);
	for (int32 i = 0; i < 3; i++)
	{
		APointLight* L = W->SpawnActor<APointLight>(FVector((G.X - 7 + i * 7) * 100, G.Y * 100, 520), FRotator::ZeroRotator, P);
		if (L)
		{
			L->PointLightComponent->SetMobility(EComponentMobility::Movable);
			L->PointLightComponent->SetIntensity(8000.f);
			L->PointLightComponent->SetAttenuationRadius(1600.f);
			L->PointLightComponent->SetLightColor(FLinearColor(1.f, 0.93f, 0.82f));
		}
	}
	// ---- trees around the field (instanced: trunk + crown)
	AActor* Trees = W->SpawnActor<AActor>(AActor::StaticClass(), FVector::ZeroVector, FRotator::ZeroRotator, P);
	if (Trees)
	{
		USceneComponent* Root = NewObject<USceneComponent>(Trees);
		Trees->SetRootComponent(Root);
		Root->RegisterComponent();
		UHierarchicalInstancedStaticMeshComponent* Trunks = NewObject<UHierarchicalInstancedStaticMeshComponent>(Trees);
		UHierarchicalInstancedStaticMeshComponent* Crowns = NewObject<UHierarchicalInstancedStaticMeshComponent>(Trees);
		Trunks->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder")));
		Crowns->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cone.Cone")));
		Trunks->SetMaterial(0, Mat(FLinearColor(0.12f, 0.08f, 0.05f)));
		Crowns->SetMaterial(0, Mat(FLinearColor(0.03f, 0.1f, 0.03f)));
		for (UHierarchicalInstancedStaticMeshComponent* C : {Trunks, Crowns})
		{
			C->SetupAttachment(Root);
			C->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
			C->RegisterComponent();
		}
		FRandomStream R(7);
		for (int32 i = 0; i < 500; i++)
		{
			const double a = R.FRandRange(0, 2 * PI), d = R.FRandRange(1.25, 1.9);
			const FVector Pos(Rx * d * FMath::Cos(a), Ry * d * FMath::Sin(a), 0);
			const float h = R.FRandRange(0.8f, 1.4f);
			Trunks->AddInstance(FTransform(FRotator::ZeroRotator, (Pos + FVector(0, 0, 2.0 * h)) * 100.0, FVector(0.35, 0.35, 4.0 * h)));
			Crowns->AddInstance(FTransform(FRotator::ZeroRotator, (Pos + FVector(0, 0, 7.0 * h)) * 100.0, FVector(4.0 * h, 4.0 * h, 8.0 * h)));
		}
	}
}
