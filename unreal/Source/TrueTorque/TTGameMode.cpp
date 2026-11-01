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
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "TTCar.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Net/UnrealNetwork.h"
#include "TTHud.h"

ATTGameMode::ATTGameMode()
{
	DefaultPawnClass = ATTCar::StaticClass();
	GameStateClass = ATTGameState::StaticClass();
	PlayerControllerClass = APlayerController::StaticClass();
	HUDClass = ATTHUD::StaticClass();
	PlayerStateClass = ATTPlayerState::StaticClass();
}

ATTCar* ATTGameMode::SpawnCar(const FVector& Loc, float Yaw, int32 CarIndex, const FLinearColor& Paint)
{
	const FTransform T(FRotator(0.f, Yaw, 0.f), Loc);
	// deferred so paint and model are set before BeginPlay builds the car
	ATTCar* Car = GetWorld()->SpawnActorDeferred<ATTCar>(ATTCar::StaticClass(), T, nullptr, nullptr, ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
	if (!Car) return nullptr;
	Car->PaintColor = Paint;
	Car->CarIndex = CarIndex;
	Car->FinishSpawning(T);
	return Car;
}

static const FLinearColor Paints[] = {{0.42f, 0.03f, 0.02f}, {0.02f, 0.08f, 0.35f}, {0.7f, 0.7f, 0.68f}, {0.02f, 0.02f, 0.02f}, {0.05f, 0.25f, 0.08f}, {0.6f, 0.4f, 0.02f}};

APawn* ATTGameMode::SpawnDefaultPawnFor_Implementation(AController* NewPlayer, AActor* StartSpot)
{
	// cars line up on the start straight (bottom of the oval, y = -380 m), facing +X
	const int32 i = Spawned++;
	const int32 N = ATTCar::Catalog().Num();
	const FVector Loc((-20.f - (i / 2) * 9.f) * 100.f, (-380.f + (i % 2 ? 2.5f : -2.5f)) * 100.f, 80.f);
	return SpawnCar(Loc, 0.f, N ? i % N : -1, Paints[i % 6]);
}

void ATTGameMode::StartPlay()
{
	Super::StartPlay();
	// free cars parked by the garage: walk up (F) and take one
	const int32 N = ATTCar::Catalog().Num();
	const int32 Count = FMath::Min(N, 7) + 1;
	for (int32 k = 0; k < Count; k++)
		SpawnCar(FVector((-90.f + k * 6.f) * 100.f, (-380.f + 52.f) * 100.f, 80.f), 90.f, k < N ? k : -1, Paints[(k + 3) % 6]);
}

// ------------------------------------------------------------------ helpers

UMaterialInstanceDynamic* ATTGameState::Mat(const TCHAR* Name, const FLinearColor& C, float Rough)
{
	UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr, *FString::Printf(TEXT("/Game/TT/Materials/%s.%s"), Name, Name));
	if (!Base) Base = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	UMaterialInstanceDynamic* M = UMaterialInstanceDynamic::Create(Base, this);
	M->SetVectorParameterValue(TEXT("Color"), C);
	if (Rough >= 0.f) M->SetScalarParameterValue(TEXT("Rough"), Rough);
	Mats.Add(M);
	return M;
}

AActor* ATTGameState::Box(const FVector& Centre, const FVector& SizeM, const FRotator& Rot, UMaterialInterface* M, bool bPhysics, const TCHAR* Shape, UPhysicalMaterial* Surface)
{
	UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, *FString::Printf(TEXT("/Engine/BasicShapes/%s.%s"), Shape, Shape));
	FActorSpawnParameters P;
	P.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	AStaticMeshActor* A = GetWorld()->SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass(), Centre * 100.0, Rot, P);
	if (!A) return nullptr;
	A->SetReplicates(false);
	UStaticMeshComponent* C = A->GetStaticMeshComponent();
	C->SetMobility(EComponentMobility::Movable);
	C->SetStaticMesh(Mesh);
	C->SetWorldScale3D(SizeM);
	C->SetMaterial(0, M);
	C->SetCollisionProfileName(bPhysics ? TEXT("PhysicsActor") : TEXT("BlockAll"));
	if (Surface) C->SetPhysMaterialOverride(Surface);
	if (bPhysics)
	{
		C->SetSimulatePhysics(true);
		C->SetMassOverrideInKg(NAME_None, 40.f, true);
	}
	return A;
}

UHierarchicalInstancedStaticMeshComponent* ATTGameState::Instances(AActor* Owner, const TCHAR* Shape, UMaterialInterface* M, bool bCollide)
{
	UHierarchicalInstancedStaticMeshComponent* C = NewObject<UHierarchicalInstancedStaticMeshComponent>(Owner);
	C->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, *FString::Printf(TEXT("/Engine/BasicShapes/%s.%s"), Shape, Shape)));
	C->SetMaterial(0, M);
	C->SetupAttachment(Owner->GetRootComponent());
	C->SetCollisionEnabled(bCollide ? ECollisionEnabled::QueryAndPhysics : ECollisionEnabled::NoCollision);
	C->RegisterComponent();
	return C;
}

void ATTPlayerState::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
	Super::GetLifetimeReplicatedProps(OutLifetimeProps);
	DOREPLIFETIME(ATTPlayerState, Money);
	DOREPLIFETIME(ATTPlayerState, BestLap);
	DOREPLIFETIME(ATTPlayerState, LastLap);
}

bool ATTPlayerState::Pay(int32 Amount)
{
	if (Amount <= 0) return true;
	if (Money < Amount) return false;
	Money -= Amount;
	return true;
}

ATTGameState::ATTGameState()
{
	PrimaryActorTick.bCanEverTick = true;
}

void ATTGameState::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
	Super::GetLifetimeReplicatedProps(OutLifetimeProps);
	DOREPLIFETIME(ATTGameState, TimeOfDay);
	DOREPLIFETIME(ATTGameState, Rain);
	DOREPLIFETIME(ATTGameState, Wetness);
	DOREPLIFETIME(ATTGameState, TimeScale);
}

void ATTGameState::SkipTime(float Hours) { TimeOfDay = FMath::Fmod(TimeOfDay + Hours + 24.f, 24.f); }
void ATTGameState::ToggleRain()
{
	RainTarget = RainTarget > 0.1f ? 0.f : FMath::FRandRange(0.5f, 1.f);
	WeatherTimer = FMath::FRandRange(300.f, 600.f);
}

float ATTGameState::GripFactor(float SurfaceGrip) const
{
	// a film of water: asphalt loses about a third, grass (already slippery) a little more
	const float Loss = SurfaceGrip > 0.8f ? 0.32f : 0.4f;
	return 1.f - Loss * Wetness;
}

FString ATTGameState::WeatherText() const
{
	const int32 H = int32(TimeOfDay), M = int32((TimeOfDay - H) * 60.f);
	const TCHAR* W = Rain > 0.6f ? TEXT("ливень") : Rain > 0.15f ? TEXT("дождь") : Wetness > 0.2f ? TEXT("мокро") : TEXT("сухо");
	return FString::Printf(TEXT("%02d:%02d  %s  (дорога мокрая на %.0f%%)"), H, M, W, Wetness * 100.f);
}

void ATTGameState::Tick(float Dt)
{
	Super::Tick(Dt);
	if (HasAuthority())
	{
		TimeOfDay = FMath::Fmod(TimeOfDay + Dt * TimeScale / 3600.f, 24.f);
		// weather: every few minutes the sky decides again; rain comes and goes gradually
		if ((WeatherTimer -= Dt) <= 0.f)
		{
			RainTarget = FMath::FRand() < 0.35f ? FMath::FRandRange(0.3f, 1.f) : 0.f;
			WeatherTimer = FMath::FRandRange(240.f, 600.f);
		}
		Rain = FMath::FInterpConstantTo(Rain, RainTarget, Dt, 0.02f);
		Wetness = Rain > 0.05f ? FMath::Min(1.f, Wetness + Dt * Rain / 40.f) : FMath::Max(0.f, Wetness - Dt / 400.f);
	}
	ApplyEnvironment(Dt);
	UpdateRain(Dt);
}

void ATTGameState::ApplyEnvironment(float Dt)
{
	if (!Sun) return;
	// sun along a simple arc: up at 6:00, highest at 13:00 (~58 deg), down at 20:00
	const float Day = (TimeOfDay - 6.f) / 14.f;
	const float Elev = FMath::Sin(Day * PI) * 58.f;
	const float Az = 80.f + Day * 200.f;
	Sun->SetActorRotation(FRotator(-Elev, Az, 0.f));
	const float Light = FMath::Clamp(Elev / 12.f, 0.f, 1.f);
	const float Overcast = 1.f - 0.65f * Rain;
	Cast<UDirectionalLightComponent>(Sun->GetLightComponent())->SetIntensity((Elev > -2.f ? 10.f : 0.f) * FMath::Max(Light, 0.05f) * Overcast);
	if (Moon) Cast<UDirectionalLightComponent>(Moon->GetLightComponent())->SetIntensity((1.f - Light) * 0.12f * Overcast);
	for (UPointLightComponent* L : StreetLights) L->SetIntensity(Light < 0.25f ? 600000.f : 0.f);
	if (Fog) Fog->GetComponent()->SetFogDensity(0.008f + 0.035f * Rain);
	if (Post)
	{
		// the eye adapts to the night, but not all the way (headlights should matter)
		Post->Settings.AutoExposureMinBrightness = FMath::Lerp(0.15f, 1.f, Light);
		Post->Settings.AutoExposureMaxBrightness = FMath::Lerp(1.2f, 3.f, Light);
	}
	if (Sky && (SkyRecapture -= Dt) <= 0.f) SkyRecapture = 2.f, Sky->GetLightComponent()->SetIntensity(FMath::Lerp(0.6f, 1.f, Light) * Overcast + 0.05f);
	for (UMaterialInstanceDynamic* M : WetMats)
		if (M) M->SetScalarParameterValue(TEXT("Wet"), Wetness);
}

// rain: a few hundred streaks around the local camera, falling and wrapping (cheap, no assets)
void ATTGameState::UpdateRain(float Dt)
{
	if (!Drops || GetNetMode() == NM_DedicatedServer) return;
	const APlayerController* PC = GetWorld()->GetFirstPlayerController();
	if (!PC || !PC->PlayerCameraManager) return;
	const FVector Cam = PC->PlayerCameraManager->GetCameraLocation();
	const int32 Want = FMath::RoundToInt(Rain * 900.f);
	Drops->SetVisibility(Want > 0);
	if (Want == 0) return;
	FRandomStream R(int32(GetWorld()->GetTimeSeconds() * 10.f));
	while (DropPos.Num() < 900) DropPos.Add(FVector(R.FRandRange(-1500.f, 1500.f), R.FRandRange(-1500.f, 1500.f), R.FRandRange(-300.f, 1500.f)));
	TArray<FTransform> T;
	T.Reserve(900);
	const FVector Wind(120.f, 60.f, 0.f);
	for (int32 i = 0; i < DropPos.Num(); i++)
	{
		FVector& D = DropPos[i];
		D += (FVector(0, 0, -900.f) + Wind) * Dt;
		if (D.Z < -300.f) D.Z += 1800.f;
		if (D.X > 1500.f) D.X -= 3000.f;
		if (D.Y > 1500.f) D.Y -= 3000.f;
		const bool bOn = i < Want;
		T.Add(FTransform(FRotator(-7.f, 26.f, 0.f), Cam + D, bOn ? FVector(0.006f, 0.006f, 0.45f) : FVector::ZeroVector));
	}
	Drops->BatchUpdateInstancesTransforms(0, T, true, true, true);
}

void ATTGameState::BeginPlay()
{
	Super::BeginPlay();
	BuildWorld();
	// rain streaks (local)
	FActorSpawnParameters P;
	if (AActor* RainActor = GetWorld()->SpawnActor<AActor>(AActor::StaticClass(), FVector::ZeroVector, FRotator::ZeroRotator, P))
	{
		USceneComponent* Root = NewObject<USceneComponent>(RainActor);
		RainActor->SetRootComponent(Root);
		Root->RegisterComponent();
		Drops = NewObject<UInstancedStaticMeshComponent>(RainActor);
		Drops->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder")));
		Drops->SetMaterial(0, Mat(TEXT("M_Painted"), FLinearColor(0.55f, 0.6f, 0.66f), 0.05f));
		Drops->SetupAttachment(Root);
		Drops->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Drops->SetCastShadow(false);
		Drops->RegisterComponent();
		TArray<FTransform> T;
		T.Init(FTransform(FRotator::ZeroRotator, FVector::ZeroVector, FVector::ZeroVector), 900);
		Drops->AddInstances(T, false);
		Drops->SetVisibility(false);
	}
}

// ------------------------------------------------------------------ world

void ATTGameState::BuildWorld()
{
	UWorld* W = GetWorld();
	FActorSpawnParameters P;

	// ---- sky: physical atmosphere, sun, real-time sky light, volumetric clouds, height fog
	Sun = W->SpawnActor<ADirectionalLight>(FVector(0, 0, 50000), FRotator(-36, 35, 0), P);
	if (Sun)
	{
		UDirectionalLightComponent* L = Cast<UDirectionalLightComponent>(Sun->GetLightComponent());
		L->SetMobility(EComponentMobility::Movable);
		L->SetIntensity(10.f);
		L->SetAtmosphereSunLight(true);
		L->SetCastShadows(true);
		L->SetLightSourceAngle(0.55f);
		L->bCastCloudShadows = true;
	}
	W->SpawnActor<ASkyAtmosphere>(FVector::ZeroVector, FRotator::ZeroRotator, P);
	// moonlight: a dim, cold second light that only matters at night
	Moon = W->SpawnActor<ADirectionalLight>(FVector(0, 0, 50000), FRotator(-55, -140, 0), P);
	if (Moon)
	{
		UDirectionalLightComponent* L = Cast<UDirectionalLightComponent>(Moon->GetLightComponent());
		L->SetMobility(EComponentMobility::Movable);
		L->SetIntensity(0.f);
		L->SetLightColor(FLinearColor(0.55f, 0.65f, 1.f));
		L->SetCastShadows(true);
		L->SetAtmosphereSunLight(false);
	}
	Sky = W->SpawnActor<ASkyLight>(FVector(0, 0, 1000), FRotator::ZeroRotator, P);
	if (Sky)
	{
		USkyLightComponent* S = Sky->GetLightComponent();
		S->SetMobility(EComponentMobility::Movable);
		S->bRealTimeCapture = true;
		S->SetIntensity(1.0f);
		S->RecaptureSky();
	}
	W->SpawnActor<AVolumetricCloud>(FVector::ZeroVector, FRotator::ZeroRotator, P);
	Fog = W->SpawnActor<AExponentialHeightFog>(FVector(0, 0, -1000), FRotator::ZeroRotator, P);
	if (Fog)
	{
		UExponentialHeightFogComponent* F = Fog->GetComponent();
		F->SetFogDensity(0.008f);
		F->SetFogHeightFalloff(0.12f);
		F->SetVolumetricFog(true);
		F->SetVolumetricFogScatteringDistribution(0.4f);
	}
	Post = W->SpawnActor<APostProcessVolume>(FVector::ZeroVector, FRotator::ZeroRotator, P);
	if (Post)
	{
		FPostProcessSettings& S = Post->Settings;
		Post->bUnbound = true;
		S.bOverride_AutoExposureMinBrightness = true, S.AutoExposureMinBrightness = 1.0f;
		S.bOverride_AutoExposureMaxBrightness = true, S.AutoExposureMaxBrightness = 3.0f;
		S.bOverride_AutoExposureSpeedUp = true, S.AutoExposureSpeedUp = 2.f;
		S.bOverride_AutoExposureSpeedDown = true, S.AutoExposureSpeedDown = 1.f;
		S.bOverride_BloomIntensity = true, S.BloomIntensity = 0.45f;
		S.bOverride_VignetteIntensity = true, S.VignetteIntensity = 0.25f;
		S.bOverride_SceneFringeIntensity = true, S.SceneFringeIntensity = 0.2f;
		S.bOverride_ColorSaturation = true, S.ColorSaturation = FVector4(1.06f, 1.06f, 1.06f, 1.f);
		S.bOverride_ColorContrast = true, S.ColorContrast = FVector4(1.04f, 1.04f, 1.04f, 1.f);
		S.bOverride_FilmToe = true, S.FilmToe = 0.6f;
		S.bOverride_LumenFinalGatherQuality = true, S.LumenFinalGatherQuality = 2.f;
		S.bOverride_LumenSceneDetail = true, S.LumenSceneDetail = 2.f;
		S.bOverride_LumenReflectionQuality = true, S.LumenReflectionQuality = 2.f;  // sharper reflections on the paint
		S.bOverride_LumenSceneLightingQuality = true, S.LumenSceneLightingQuality = 2.f;
		S.bOverride_AmbientOcclusionIntensity = true, S.AmbientOcclusionIntensity = 0.6f;
		S.bOverride_LensFlareIntensity = true, S.LensFlareIntensity = 0.15f;
	}

	// ---- surfaces (grip comes from the friction: tyres divide by 0.7)
	auto Surface = [this](float Friction) {
		UPhysicalMaterial* M = NewObject<UPhysicalMaterial>(this);
		M->Friction = Friction;
		Surfaces.Add(M);
		return M;
	};
	UPhysicalMaterial* SAsphalt = Surface(0.75f);
	UPhysicalMaterial* SGrass = Surface(0.42f);
	UPhysicalMaterial* SConcrete = Surface(0.68f);

	// ---- materials
	UMaterialInterface* Asphalt = Mat(TEXT("M_Asphalt"), FLinearColor(0.8f, 0.8f, 0.82f));
	UMaterialInterface* Grass = Mat(TEXT("M_Grass"), FLinearColor(0.42f, 0.6f, 0.3f));
	UMaterialInterface* Concrete = Mat(TEXT("M_Concrete"), FLinearColor(1, 1, 1));
	UMaterialInterface* DarkConcrete = Mat(TEXT("M_Concrete"), FLinearColor(0.55f, 0.55f, 0.56f));
	UMaterialInterface* KerbRed = Mat(TEXT("M_Painted"), FLinearColor(0.55f, 0.03f, 0.02f), 0.55f);
	UMaterialInterface* KerbWhite = Mat(TEXT("M_Painted"), FLinearColor(0.8f, 0.8f, 0.78f), 0.55f);
	UMaterialInterface* LinePaint = Mat(TEXT("M_Painted"), FLinearColor(0.85f, 0.85f, 0.82f), 0.6f);
	for (UMaterialInterface* M : {Asphalt, Grass, Concrete, DarkConcrete, KerbRed, KerbWhite, LinePaint}) WetMats.Add(Cast<UMaterialInstanceDynamic>(M));
	UMaterialInterface* Armco = Mat(TEXT("M_Metal"), FLinearColor(0.75f, 0.76f, 0.78f), 0.35f);
	UMaterialInterface* Rust = Mat(TEXT("M_Metal"), FLinearColor(0.35f, 0.12f, 0.05f), 0.7f);
	UMaterialInterface* Rubber = Mat(TEXT("M_Painted"), FLinearColor(0.02f, 0.02f, 0.02f), 0.9f);
	UMaterialInterface* SeatBlue = Mat(TEXT("M_Painted"), FLinearColor(0.03f, 0.08f, 0.3f), 0.5f);
	UMaterialInterface* Roof = Mat(TEXT("M_Metal"), FLinearColor(0.3f, 0.31f, 0.33f), 0.5f);
	UMaterialInterface* Wall = Mat(TEXT("M_Painted"), FLinearColor(0.55f, 0.52f, 0.47f), 0.8f);
	UMaterialInterface* Bark = Mat(TEXT("M_Painted"), FLinearColor(0.1f, 0.07f, 0.045f), 0.9f);
	UMaterialInterface* Leaves = Mat(TEXT("M_Painted"), FLinearColor(0.03f, 0.09f, 0.025f), 0.8f);
	UMaterialInterface* LeavesB = Mat(TEXT("M_Painted"), FLinearColor(0.05f, 0.11f, 0.03f), 0.8f);
	UMaterialInterface* Ads[3] = {Mat(TEXT("M_Painted"), FLinearColor(0.02f, 0.15f, 0.6f), 0.4f), Mat(TEXT("M_Painted"), FLinearColor(0.8f, 0.55f, 0.02f), 0.4f),
	                              Mat(TEXT("M_Painted"), FLinearColor(0.7f, 0.7f, 0.7f), 0.4f)};

	// ---- ground: grass field, big asphalt skid pad
	Box(FVector(0, 0, -0.52), FVector(5000, 5000, 1), FRotator::ZeroRotator, Grass, false, TEXT("Cube"), SGrass);
	Box(FVector(0, 0, -0.25), FVector(300, 300, 0.5), FRotator::ZeroRotator, Asphalt, false, TEXT("Cube"), SAsphalt);

	AActor* Props = W->SpawnActor<AActor>(AActor::StaticClass(), FVector::ZeroVector, FRotator::ZeroRotator, P);
	USceneComponent* PropRoot = NewObject<USceneComponent>(Props);
	Props->SetRootComponent(PropRoot);
	PropRoot->RegisterComponent();
	UHierarchicalInstancedStaticMeshComponent* Tyres = Instances(Props, TEXT("Cylinder"), Rubber, true);
	UHierarchicalInstancedStaticMeshComponent* Rails = Instances(Props, TEXT("Cube"), Armco, true);
	UHierarchicalInstancedStaticMeshComponent* Posts = Instances(Props, TEXT("Cube"), Armco, false);

	// ---- circuit: an oval with a chicane; track tops flush with the ground
	const int32 Seg = 120;
	const double Rx = 700, Ry = 380;
	auto Pt = [&](double a) { return FVector(Rx * FMath::Cos(a), Ry * FMath::Sin(a) + (FMath::Abs(FMath::Sin(a * 2)) < 0.2 ? 15 * FMath::Sin(a * 6) : 0), 0); };
	for (int32 i = 0; i < Seg; i++)
	{
		const double a0 = 2 * PI * i / Seg, a1 = 2 * PI * (i + 1) / Seg;
		const FVector A = Pt(a0), B = Pt(a1), Mid = (A + B) * 0.5, D = B - A;
		const double L = D.Size();
		const FRotator R(0, FMath::RadiansToDegrees(FMath::Atan2(D.Y, D.X)), 0);
		const FVector N = FVector(-D.Y, D.X, 0).GetSafeNormal();  // points into the oval
		Box(FVector(Mid.X, Mid.Y, -0.25), FVector(L + 2.5, 14, 0.5), R, Asphalt, false, TEXT("Cube"), SAsphalt);
		for (int32 s : {-1, 1})
		{
			// kerbs (red/white), white edge lines just inside them
			Box(FVector(Mid.X, Mid.Y, 0.03) + N * (7.6 * s), FVector(L + 0.2, 1.2, 0.12), R, (i % 2) ? KerbRed : KerbWhite, false, TEXT("Cube"), SConcrete);
			Box(FVector(Mid.X, Mid.Y, 0.004) + N * (6.75 * s), FVector(L + 0.2, 0.25, 0.02), R, LinePaint);
		}
		const bool bBend = FMath::Abs(FMath::Cos(a0)) > 0.85;
		if (bBend)
		{
			// concrete wall on the outside of the bends with a row of tyre stacks in front of it
			Box(FVector(Mid.X, Mid.Y, 0.6) - N * 15.0, FVector(L + 0.5, 0.6, 1.2), R, Concrete, false, TEXT("Cube"), SConcrete);
			const FVector Dir = D.GetSafeNormal();
			for (double t = -L * 0.5; t < L * 0.5; t += 0.8)
				for (int32 h = 0; h < 3; h++)
					Tyres->AddInstance(FTransform(FRotator::ZeroRotator, (Mid + Dir * t - N * 14.2 + FVector(0, 0, 0.15 + h * 0.3)) * 100.0, FVector(0.75, 0.75, 0.3)));
		}
		else
		{
			// armco rail on posts along the straights, outside the track
			Rails->AddInstance(FTransform(R, (Mid - N * 13.0 + FVector(0, 0, 0.6)) * 100.0, FVector(L + 0.1, 0.12, 0.35)));
			Posts->AddInstance(FTransform(R, (Mid - N * 13.15 + FVector(0, 0, 0.4)) * 100.0, FVector(0.1, 0.1, 0.8)));
		}
	}
	Box(FVector(0, -Ry, 0.006), FVector(0.6, 14, 0.02), FRotator::ZeroRotator, LinePaint);  // start line
	// grid boxes
	for (int32 i = 0; i < 6; i++) Box(FVector(-20 - (i / 2) * 9, -Ry + ((i % 2) ? 2.5 : -2.5) - 0.0, 0.005) + FVector(2.4, 0, 0), FVector(0.2, 2.2, 0.02), FRotator::ZeroRotator, LinePaint);

	// ---- grandstand along the start straight (outside), with a roof
	{
		const double Y0 = -Ry - 22;
		for (int32 k = 0; k < 9; k++)
		{
			Box(FVector(0, Y0 - k * 1.6, 0.25 + k * 0.45), FVector(120, 1.6, 0.5 + k * 0.9), FRotator::ZeroRotator, DarkConcrete);
			Box(FVector(0, Y0 - k * 1.6 + 0.3, 0.55 + k * 0.9), FVector(118, 0.5, 0.35), FRotator::ZeroRotator, SeatBlue);
		}
		Box(FVector(0, Y0 - 15.5, 6), FVector(122, 0.4, 12), FRotator::ZeroRotator, Wall);
		Box(FVector(0, Y0 - 7, 12.2), FVector(124, 17, 0.4), FRotator(0, 0, -6), Roof);
		for (int32 k = 0; k < 7; k++) Box(FVector(-60 + k * 20, Y0 - 15, 6), FVector(0.5, 0.5, 12), FRotator::ZeroRotator, Roof, false, TEXT("Cylinder"));
	}
	// ---- billboards and light towers around the bends
	for (int32 k = 0; k < 8; k++)
	{
		const double a = 2 * PI * (k + 0.5) / 8;
		const FVector Out = Pt(a) * 1.0 + FVector(FMath::Cos(a), FMath::Sin(a) * 1.4, 0).GetSafeNormal() * 30.0;
		const FRotator Face(0, FMath::RadiansToDegrees(a) + 90, 0);
		Box(Out + FVector(0, 0, 4.5), FVector(12, 0.3, 3.5), Face, Ads[k % 3]);
		Box(Out + FVector(0, 0, 18), FVector(0.6, 0.6, 36), FRotator::ZeroRotator, Roof, false, TEXT("Cylinder"));
		Box(Out + FVector(0, 0, 36.5), FVector(3, 0.6, 1.5), Face, Roof);
		// floodlight on the tower, on at night
		if (APointLight* L = W->SpawnActor<APointLight>((Out + FVector(0, 0, 35.5)) * 100.0, FRotator::ZeroRotator, P))
		{
			L->PointLightComponent->SetMobility(EComponentMobility::Movable);
			L->PointLightComponent->SetIntensity(0.f);
			L->PointLightComponent->SetAttenuationRadius(14000.f);
			L->PointLightComponent->SetSourceRadius(60.f);
			L->PointLightComponent->SetLightColor(FLinearColor(1.f, 0.92f, 0.8f));
			L->PointLightComponent->SetCastShadows(false);
			StreetLights.Add(L->PointLightComponent);
		}
	}

	// ---- crash zone on the pad: wall, barrels, ramps, bumps
	Box(FVector(220, -150, 1.5), FVector(1.0, 24, 3), FRotator::ZeroRotator, Concrete, false, TEXT("Cube"), SConcrete);
	for (int32 i = 0; i < 12; i++) Box(FVector(140 + (i % 4) * 1.4, 80 + (i / 4) * 1.4, 0.45), FVector(0.6, 0.6, 0.9), FRotator::ZeroRotator, Rust, true, TEXT("Cylinder"));
	Box(FVector(60, 120, 0.9), FVector(12, 6, 0.4), FRotator(10, 0, 0), Concrete, false, TEXT("Cube"), SConcrete);
	Box(FVector(60, 150, 1.4), FVector(14, 6, 0.4), FRotator(-8, 180, 0), Concrete, false, TEXT("Cube"), SConcrete);
	for (int32 i = 0; i < 6; i++) Box(FVector(-60 + i * 12, -120, 0.06), FVector(1.2, 10, 0.12), FRotator::ZeroRotator, (i % 2) ? KerbRed : KerbWhite);
	for (int32 i = 0; i < 4; i++) Box(FVector(-100, 60 + i * 20, 0.05), FVector(0.8, 10, 0.1), FRotator::ZeroRotator, LinePaint, false, TEXT("Cylinder"));

	// ---- garage (workshop) next to the start
	const FVector G(-60, -Ry + 36, 0);
	Box(G + FVector(0, 0, -0.2), FVector(24, 18, 0.4), FRotator::ZeroRotator, Concrete, false, TEXT("Cube"), SConcrete);
	Box(G + FVector(0, 9, 3), FVector(24, 0.4, 6), FRotator::ZeroRotator, Wall);
	Box(G + FVector(-12, 0, 3), FVector(0.4, 18, 6), FRotator::ZeroRotator, Wall);
	Box(G + FVector(12, 4, 3), FVector(0.4, 10, 6), FRotator::ZeroRotator, Wall);
	Box(G + FVector(0, 0, 6.2), FVector(24.6, 18.6, 0.4), FRotator::ZeroRotator, Roof);
	for (int32 i = 0; i < 3; i++)
		if (APointLight* L = W->SpawnActor<APointLight>(FVector((G.X - 7 + i * 7) * 100, G.Y * 100, 520), FRotator::ZeroRotator, P))
		{
			L->PointLightComponent->SetMobility(EComponentMobility::Movable);
			L->PointLightComponent->SetIntensity(8000.f);
			L->PointLightComponent->SetAttenuationRadius(1600.f);
			L->PointLightComponent->SetLightColor(FLinearColor(1.f, 0.93f, 0.82f));
		}

	// ---- trees around the field: trunk + two crowns, random size and lean
	UHierarchicalInstancedStaticMeshComponent* Trunks = Instances(Props, TEXT("Cylinder"), Bark, true);
	UHierarchicalInstancedStaticMeshComponent* Crowns = Instances(Props, TEXT("Cone"), Leaves, false);
	UHierarchicalInstancedStaticMeshComponent* CrownsB = Instances(Props, TEXT("Sphere"), LeavesB, false);
	FRandomStream Rnd(7);
	for (int32 i = 0; i < 900; i++)
	{
		const double a = Rnd.FRandRange(0, 2 * PI), d = Rnd.FRandRange(1.3, 2.6);
		const FVector Pos(Rx * d * FMath::Cos(a), Ry * d * 1.3 * FMath::Sin(a), 0);
		if (FMath::Abs(Pos.X) > 2400 || FMath::Abs(Pos.Y) > 2400) continue;
		const float h = Rnd.FRandRange(0.7f, 1.5f);
		const FRotator Lean(Rnd.FRandRange(-3.f, 3.f), Rnd.FRandRange(0.f, 360.f), Rnd.FRandRange(-3.f, 3.f));
		Trunks->AddInstance(FTransform(Lean, (Pos + FVector(0, 0, 2.0 * h)) * 100.0, FVector(0.35, 0.35, 4.0 * h)));
		if (Rnd.FRand() < 0.6f)
		{
			Crowns->AddInstance(FTransform(Lean, (Pos + FVector(0, 0, 6.5 * h)) * 100.0, FVector(4.2 * h, 4.2 * h, 7.0 * h)));
			Crowns->AddInstance(FTransform(Lean, (Pos + FVector(0, 0, 9.5 * h)) * 100.0, FVector(3.0 * h, 3.0 * h, 5.0 * h)));
		}
		else
			CrownsB->AddInstance(FTransform(Lean, (Pos + FVector(0, 0, 6.5 * h)) * 100.0, FVector(5.5 * h, 5.5 * h, 5.0 * h)));
	}
	// ---- horizon: low hills and a far town in the haze
	UHierarchicalInstancedStaticMeshComponent* Hills = Instances(Props, TEXT("Sphere"), Grass, false);
	UHierarchicalInstancedStaticMeshComponent* Town = Instances(Props, TEXT("Cube"), DarkConcrete, false);
	for (int32 i = 0; i < 18; i++)
	{
		const double a = 2 * PI * i / 18 + Rnd.FRandRange(-0.1, 0.1), d = Rnd.FRandRange(2600, 3400);
		Hills->AddInstance(FTransform(FRotator(0, Rnd.FRandRange(0.f, 360.f), 0), FVector(d * FMath::Cos(a), d * FMath::Sin(a), -60) * 100.0,
		                              FVector(Rnd.FRandRange(900, 1500), Rnd.FRandRange(500, 900), Rnd.FRandRange(120, 260))));
	}
	for (int32 i = 0; i < 60; i++)
	{
		const double a = Rnd.FRandRange(-0.5, 0.5), d = Rnd.FRandRange(2200, 2500);
		const double hgt = Rnd.FRandRange(15, 90);
		Town->AddInstance(FTransform(FRotator(0, Rnd.FRandRange(0.f, 90.f), 0), FVector(d * FMath::Cos(a), d * FMath::Sin(a), hgt * 0.5) * 100.0,
		                             FVector(Rnd.FRandRange(15, 40), Rnd.FRandRange(15, 40), hgt)));
	}
}
