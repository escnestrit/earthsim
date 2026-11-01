#include "TTVehicle.h"
#include "Camera/CameraComponent.h"
#include "Camera/PlayerCameraManager.h"
#include "Components/StaticMeshComponent.h"
#include "Components/TextRenderComponent.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "Engine/LocalPlayer.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/SpringArmComponent.h"
#include "InputAction.h"
#include "InputMappingContext.h"
#include "InputModifiers.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "Kismet/GameplayStatics.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Net/UnrealNetwork.h"
#include "ProceduralMeshComponent.h"
#include "TTEngineSound.h"
#include "net.hpp"
#include "vehicle.hpp"

// tt frame: x right, y forward, z up, metres.  Unreal: X forward, Y right, Z up, centimetres.
static FORCEINLINE FVector ToUE(const tt::V3& p) { return FVector(p.y * 100.0, p.x * 100.0, p.z * 100.0); }
static FORCEINLINE FVector DirUE(const tt::V3& d) { return FVector(d.y, d.x, d.z); }
static FORCEINLINE tt::V3 ToTT(const FVector& v) { return tt::V3(v.Y / 100.0, v.X / 100.0, v.Z / 100.0); }
static FORCEINLINE tt::V3 DirTT(const FVector& v) { return tt::V3(v.Y, v.X, v.Z); }

ATTVehicle::ATTVehicle()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;
	bReplicates = true;
	SetReplicateMovement(false);
	NetUpdateFrequency = 60.f;
	bAlwaysRelevant = true;

	Body = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Body"));
	RootComponent = Body;
	Body->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Body->bUseAsyncCooking = true;
	Body->SetCastShadow(true);

	Arm = CreateDefaultSubobject<USpringArmComponent>(TEXT("Arm"));
	Arm->SetupAttachment(Body);
	Arm->TargetArmLength = 650.f;
	Arm->SocketOffset = FVector(0, 0, 160);
	Arm->bEnableCameraLag = true;
	Arm->CameraLagSpeed = 7.f;
	Arm->bEnableCameraRotationLag = true;
	Arm->CameraRotationLagSpeed = 5.f;
	Arm->bDoCollisionTest = true;
	Arm->bInheritPitch = false;
	Arm->bInheritRoll = false;
	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	Camera->SetupAttachment(Arm);
	Camera->SetFieldOfView(75.f);
	Camera->PostProcessSettings.bOverride_MotionBlurAmount = true;
	Camera->PostProcessSettings.MotionBlurAmount = 0.35f;

	Sound = CreateDefaultSubobject<UTTEngineSound>(TEXT("Sound"));
	Sound->SetupAttachment(Body);

	auto MakeText = [&](const TCHAR* Name, float Size, FColor Col) {
		UTextRenderComponent* T = CreateDefaultSubobject<UTextRenderComponent>(Name);
		T->SetupAttachment(Body);
		T->SetWorldSize(Size);
		T->SetTextRenderColor(Col);
		T->SetHorizontalAlignment(EHTA_Center);
		T->SetCastShadow(false);
		return T;
	};
	DashSpeed = MakeText(TEXT("DashSpeed"), 5.f, FColor(255, 220, 160));
	DashInfo = MakeText(TEXT("DashInfo"), 2.2f, FColor(230, 230, 230));
	WorldNote = MakeText(TEXT("WorldNote"), 9.f, FColor::White);
	WorldNote->SetVisibility(false);
	AutoPossessPlayer = EAutoReceiveInput::Disabled;
}

ATTVehicle::~ATTVehicle() = default;

void ATTVehicle::BeginPlay()
{
	Super::BeginPlay();
	Sim = std::make_unique<tt::Vehicle>();
	tt::VehicleDesc D;
	Sim->build(D);
	const FVector L = GetActorLocation();
	const double Yaw = -FMath::DegreesToRadians(GetActorRotation().Yaw);
	Sim->place(ToTT(L) + tt::V3(0, 0, 0.15), Yaw);
	Ground.SetNum(Sim->wheels.size());
	WheelSpin.SetNum(Sim->wheels.size());
	BuildMesh();
	// wheels: engine cylinders (axis along Z, 100 cm) turned to the side
	UStaticMesh* Cyl = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	for (size_t i = 0; i < Sim->wheels.size(); i++)
	{
		UStaticMeshComponent* W = NewObject<UStaticMeshComponent>(this);
		W->SetStaticMesh(Cyl);
		W->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		W->SetWorldScale3D(FVector(Sim->wheels[i].radius * 2.0, Sim->wheels[i].radius * 2.0, Sim->wheels[i].width));
		W->RegisterComponent();
		W->AttachToComponent(Body, FAttachmentTransformRules::KeepWorldTransform);
		UMaterialInstanceDynamic* M = UMaterialInstanceDynamic::Create(Base, this);
		M->SetVectorParameterValue(TEXT("Color"), FLinearColor(0.02f, 0.02f, 0.022f));
		W->SetMaterial(0, M);
		WheelMeshes.Add(W);
	}
	// dashboard in front of the driver (left seat), visible from the cockpit view
	DashSpeed->SetRelativeLocation(FVector(55, -36, 95));
	DashSpeed->SetRelativeRotation(FRotator(15, 180, 0));
	DashInfo->SetRelativeLocation(FVector(55, -36, 88));
	DashInfo->SetRelativeRotation(FRotator(15, 180, 0));
	ApplyView();
}

void ATTVehicle::EndPlay(const EEndPlayReason::Type Reason)
{
	Super::EndPlay(Reason);
}

// ------------------------------------------------------------------ mesh

void ATTVehicle::BuildMesh()
{
	UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	const FLinearColor Cols[4] = {PaintColor, FLinearColor(0.02f, 0.03f, 0.04f), FLinearColor(0.03f, 0.03f, 0.03f), FLinearColor(1.0f, 0.9f, 0.7f)};
	SectionQuads.SetNum(4);
	for (int32 m = 0; m < 4; m++)
	{
		UMaterialInstanceDynamic* M = UMaterialInstanceDynamic::Create(Base, this);
		M->SetVectorParameterValue(TEXT("Color"), Cols[m]);
		Mats.Add(M);
	}
	for (int32 i = 0; i < int32(Sim->skin.size()); i++) SectionQuads[Sim->skin[i].mat].Add(i);
	for (int32 m = 0; m < 4; m++)
	{
		TArray<FVector> V;
		TArray<int32> I;
		TArray<FVector> N;
		TArray<FVector2D> UV;
		for (int32 q : SectionQuads[m])
		{
			int32 b = V.Num();
			for (int32 k = 0; k < 4; k++) V.Add(FVector::ZeroVector), N.Add(FVector::UpVector), UV.Add(FVector2D((k == 1 || k == 2) ? 1.0 : 0.0, k >= 2 ? 1.0 : 0.0));
			// flat-shaded quad, two triangles (Unreal front faces are clockwise)
			I.Append({b, b + 2, b + 1, b, b + 3, b + 2});
		}
		Body->CreateMeshSection(m, V, I, N, UV, TArray<FColor>(), TArray<FProcMeshTangent>(), false);
		Body->SetMaterial(m, Mats[m]);
	}
	UpdateMesh();
}

void ATTVehicle::UpdateMesh()
{
	const FTransform Inv = GetActorTransform().Inverse();
	auto& Nodes = Sim->body.nodes;
	// car centre in local space: panels face away from it
	FVector Ctr = FVector::ZeroVector;
	for (const tt::Node& Nd : Nodes) Ctr += Inv.TransformPosition(ToUE(Nd.p));
	Ctr /= double(FMath::Max<int32>(1, int32(Nodes.size())));
	for (int32 m = 0; m < SectionQuads.Num(); m++)
	{
		TArray<FVector> V;
		TArray<FVector> N;
		TArray<FProcMeshTangent> T;
		V.Reserve(SectionQuads[m].Num() * 4);
		for (int32 q : SectionQuads[m])
		{
			const tt::SkinQuad& S = Sim->skin[q];
			FVector P[4] = {Inv.TransformPosition(ToUE(Nodes[S.a].p)), Inv.TransformPosition(ToUE(Nodes[S.b].p)), Inv.TransformPosition(ToUE(Nodes[S.c].p)),
			                Inv.TransformPosition(ToUE(Nodes[S.d].p))};
			// detached parts stay drawn where they fell (their nodes keep simulating)
			FVector Nrm = FVector::CrossProduct(P[2] - P[0], P[1] - P[0]).GetSafeNormal();
			if (FVector::DotProduct(Nrm, (P[0] + P[1] + P[2] + P[3]) * 0.25 - Ctr) < 0) Nrm = -Nrm;
			else Swap(P[1], P[3]);  // triangles (0,2,1),(0,3,2) must wind so the front face points along Nrm
			const FVector Tx = (P[1] - P[0] - Nrm * FVector::DotProduct(P[1] - P[0], Nrm)).GetSafeNormal();
			for (int32 k = 0; k < 4; k++) V.Add(P[k]), N.Add(Nrm), T.Add(FProcMeshTangent(Tx, false));
		}
		Body->UpdateMeshSection(m, V, N, TArray<FVector2D>(), TArray<FColor>(), T);
	}
}

void ATTVehicle::UpdateWheels(float Dt)
{
	tt::V3 o, r, f, u;
	Sim->frame(o, r, f, u);
	for (size_t i = 0; i < Sim->wheels.size(); i++)
	{
		const tt::Wheel& W = Sim->wheels[i];
		WheelSpin[i] = FMath::Fmod(WheelSpin[i] + float(W.omega) * Dt, 2.f * PI);
		const tt::V3 c = Sim->body.nodes[W.hub].p;
		// axle = car's right rotated by the steering angle; spin about the axle
		const double cs = std::cos(W.steer), sn = std::sin(W.steer);
		const tt::V3 axle = r * cs - f * sn;
		const FVector Ax = DirUE(axle);
		const FQuat Steer = FRotationMatrix::MakeFromYZ(Ax, DirUE(u)).ToQuat();     // Y along the axle
		const FQuat ToAxle = FQuat(FVector::ForwardVector, PI * 0.5f);              // cylinder Z -> Y
		const FQuat Spin = FQuat(FVector::RightVector, WheelSpin[i]);
		WheelMeshes[i]->SetWorldLocationAndRotation(ToUE(c), Steer * Spin * ToAxle);
		WheelMeshes[i]->SetVisibility(true);
	}
}

// ------------------------------------------------------------------ world contact

tt::GroundHit ATTVehicle::GroundAt(double X, double Y) const
{
	// nearest cached wheel trace (the tyres ask for the ground under themselves)
	const FGroundCache* Best = nullptr;
	double Bd = 4.0;
	for (const FGroundCache& G : Ground)
	{
		const double d = (G.X - X) * (G.X - X) + (G.Y - Y) * (G.Y - Y);
		if (d < Bd) Bd = d, Best = &G;
	}
	tt::GroundHit H;
	if (!Best || Best->H < -1e8) return H;
	// plane through the traced point
	const double dz = -((X - Best->X) * Best->Nx + (Y - Best->Y) * Best->Ny) / FMath::Max(Best->Nz, 0.2);
	H.h = Best->H + dz;
	H.n = tt::V3(Best->Nx, Best->Ny, Best->Nz);
	H.friction = Best->Friction;
	H.surface = Best->Surface;
	return H;
}

void ATTVehicle::UpdateContacts(float Dt)
{
	UWorld* W = GetWorld();
	FCollisionQueryParams Q(SCENE_QUERY_STAT(TTContacts), false, this);
	Q.bReturnPhysicalMaterial = true;
	for (TActorIterator<ATTVehicle> It(W); It; ++It) Q.AddIgnoredActor(*It);  // cars collide through the soft-body solver
	// wheels: one trace each
	for (size_t i = 0; i < Sim->wheels.size(); i++)
	{
		const tt::V3 c = Sim->body.nodes[Sim->wheels[i].hub].p;
		FHitResult Hit;
		FGroundCache& G = Ground[i];
		G.X = c.x, G.Y = c.y;
		if (W->LineTraceSingleByChannel(Hit, ToUE(c) + FVector(0, 0, 60), ToUE(c) - FVector(0, 0, 120), ECC_Visibility, Q))
		{
			const tt::V3 p = ToTT(Hit.ImpactPoint), n = DirTT(Hit.ImpactNormal);
			G.X = p.x, G.Y = p.y, G.H = p.z, G.Nx = n.x, G.Ny = n.y, G.Nz = n.z;
			G.Friction = 1.0;
			if (UPhysicalMaterial* PM = Hit.PhysMaterial.Get()) G.Friction = FMath::Clamp(double(PM->Friction) * 1.4, 0.4, 1.2);
		}
		else
			G.H = -1e9;
	}
	// body nodes: a short sweep along their motion and down, for walls, kerbs, the ground
	auto& Nodes = Sim->body.nodes;
	for (size_t i = 0; i < Nodes.size(); i++)
	{
		auto& C = Sim->body.contacts[i];
		C.clear();
		const tt::Node& N = Nodes[i];
		if (!N.collide) continue;
		const FVector P = ToUE(N.p);
		const FVector Vel = DirUE(N.v) * 100.0;
		const FVector Ahead = P + Vel * (Dt * 2.0) + Vel.GetSafeNormal() * (N.radius * 150.0);
		FHitResult Hit;
		const bool bMoving = Vel.SizeSquared() > 50.0 * 50.0;
		if (bMoving && W->LineTraceSingleByChannel(Hit, P, Ahead, ECC_Visibility, Q) && !Hit.bStartPenetrating &&
		    FVector::DotProduct(Hit.ImpactNormal, Vel.GetSafeNormal()) < -0.1)
		{
			C.push_back(tt::ContactPlane{ToTT(Hit.ImpactPoint), DirTT(Hit.ImpactNormal), 0.6, 0});
			if (UPrimitiveComponent* Comp = Hit.GetComponent())
				if (Comp->IsSimulatingPhysics() && Vel.Size() > 50.0)
					Comp->AddImpulseAtLocation(Vel.GetSafeNormal() * N.m * Vel.Size() * 0.3, Hit.ImpactPoint);
		}
		if (W->LineTraceSingleByChannel(Hit, P + FVector(0, 0, 30), P - FVector(0, 0, 40 + N.radius * 100.0), ECC_Visibility, Q) &&
		    !Hit.bStartPenetrating && Hit.ImpactNormal.Z > 0.3)
			C.push_back(tt::ContactPlane{ToTT(Hit.ImpactPoint), DirTT(Hit.ImpactNormal), 0.7, 0});
	}
}

// ------------------------------------------------------------------ tick

void ATTVehicle::Tick(float Dt)
{
	Super::Tick(Dt);
	if (!Sim) return;
	const bool bServer = HasAuthority();
	if (IsLocallyControlled())
	{
		if (!bServer) ServerInput(InThrottle, InBrake, InSteer, InHandbrake, InShift);
		else Sim->in.throttle = InThrottle, Sim->in.brake = InBrake, Sim->in.steer = InSteer, Sim->in.handbrake = InHandbrake, Sim->in.shift += InShift;
		InShift = 0;
	}
	if (bServer)
	{
		UpdateContacts(Dt);
		std::vector<tt::Vehicle*> Others;
		for (TActorIterator<ATTVehicle> It(GetWorld()); It; ++It)
			if (It->GetSim()) Others.push_back(It->GetSim());
		Sim->body.stats.impact_peak = 0;
		Sim->update(Dt, [this](double X, double Y) { return GroundAt(X, Y); }, &Others);
		// fell off the world
		if (Sim->body.nodes[Sim->ref_c].p.z < -50) OnReset();
		const double Peak = Sim->body.stats.impact_peak;
		if (Peak > 120.0 && Peak > LastPeak * 1.5) MulticastImpact(float(FMath::Clamp(Peak / 2000.0, 0.1, 1.0)));
		LastPeak = Peak;
		SendTimer -= Dt;
		if (SendTimer <= 0.f && GetNetMode() != NM_Standalone)
		{
			SendTimer = 1.f / 30.f;
			tt::Snapshot S = tt::encode(*Sim, ++Tick_);
			MulticastState(TArray<uint8>(S.bytes.data(), int32(S.bytes.size())));
		}
	}
	else if (bHavePending)
	{
		// clients: show the server's car, interpolated between snapshots
		RenderFrom.resize(Sim->body.nodes.size() * 3);
		for (size_t i = 0; i < Sim->body.nodes.size(); i++)
			RenderFrom[i * 3] = Sim->body.nodes[i].p.x, RenderFrom[i * 3 + 1] = Sim->body.nodes[i].p.y, RenderFrom[i * 3 + 2] = Sim->body.nodes[i].p.z;
		tt::Snapshot S;
		S.bytes.assign(PendingState.GetData(), PendingState.GetData() + PendingState.Num());
		tt::decode(S, *Sim);
		RenderTo.resize(RenderFrom.size());
		for (size_t i = 0; i < Sim->body.nodes.size(); i++)
			RenderTo[i * 3] = Sim->body.nodes[i].p.x, RenderTo[i * 3 + 1] = Sim->body.nodes[i].p.y, RenderTo[i * 3 + 2] = Sim->body.nodes[i].p.z;
		InterpT = 0;
		bHavePending = false;
	}
	if (!bServer && !RenderTo.empty() && InterpT < 1.f)
	{
		InterpT = FMath::Min(1.f, InterpT + Dt * 30.f);
		for (size_t i = 0; i < Sim->body.nodes.size(); i++)
			Sim->body.nodes[i].p = tt::V3(FMath::Lerp(RenderFrom[i * 3], RenderTo[i * 3], double(InterpT)), FMath::Lerp(RenderFrom[i * 3 + 1], RenderTo[i * 3 + 1], double(InterpT)),
			                              FMath::Lerp(RenderFrom[i * 3 + 2], RenderTo[i * 3 + 2], double(InterpT)));
	}
	// actor follows the chassis so the camera, sound and relevancy follow the car
	tt::V3 o, r, f, u;
	Sim->frame(o, r, f, u);
	const FRotator Rot = FRotationMatrix::MakeFromXZ(DirUE(f), DirUE(u)).Rotator();
	SetActorLocationAndRotation(ToUE(o), Rot);
	UpdateMesh();
	UpdateWheels(Dt);
	UpdateDash();
	if (View == ETTView::Chase || View == ETTView::Mechanic) Arm->SetWorldRotation(FRotator(LookPitch, Rot.Yaw + LookYaw, 0));
	// sound
	float Slip = 0;
	for (const tt::Wheel& W : Sim->wheels)
		if (W.contact) Slip = FMath::Max(Slip, float(FMath::Abs(W.slip_angle) * 3.0 + FMath::Max(0.0, FMath::Abs(W.slip_ratio) - 0.1) * 2.0));
	Sound->SetState(float(Sim->engine.rpm), float(Sim->in.throttle), float(Sim->engine.knock), Slip - 0.3f, float(FMath::Abs(Sim->speed)), Sim->engine.running);
	if (NoteTimer > 0 && (NoteTimer -= Dt) <= 0) WorldNote->SetVisibility(false);
}

void ATTVehicle::UpdateDash()
{
	const tt::Engine& E = Sim->engine;
	DashSpeed->SetText(FText::FromString(FString::Printf(TEXT("%d"), FMath::RoundToInt(FMath::Abs(Sim->speed) * 3.6))));
	const int G = Sim->gearbox.display();
	FString Gear = G < 0 ? TEXT("R") : G == 0 ? TEXT("N") : FString::FromInt(G);
	FString Lamps;
	if (E.temp > 110) Lamps += TEXT("  TEMP");
	if (E.oil < 0.25) Lamps += TEXT("  OIL");
	if (E.damage > 0.4) Lamps += TEXT("  CHECK");
	DashInfo->SetText(FText::FromString(FString::Printf(TEXT("%s   %d rpm%s"), *Gear, FMath::RoundToInt(E.rpm / 10) * 10, *Lamps)));
}

void ATTVehicle::Note(const FString& Text, const FVector& Where, float Secs)
{
	WorldNote->SetText(FText::FromString(Text));
	WorldNote->SetWorldLocation(Where);
	if (APlayerController* PC = Cast<APlayerController>(GetController()))
		WorldNote->SetWorldRotation((PC->PlayerCameraManager->GetCameraLocation() - Where).Rotation());
	WorldNote->SetVisibility(true);
	NoteTimer = Secs;
}

// ------------------------------------------------------------------ network

void ATTVehicle::ServerInput_Implementation(float Throttle, float Brake, float Steer, float Handbrake, int8 Shift)
{
	if (!Sim) return;
	Sim->in.throttle = FMath::Clamp(Throttle, 0.f, 1.f);
	Sim->in.brake = FMath::Clamp(Brake, 0.f, 1.f);
	Sim->in.steer = FMath::Clamp(Steer, -1.f, 1.f);
	Sim->in.handbrake = FMath::Clamp(Handbrake, 0.f, 1.f);
	Sim->in.shift += Shift;
}

void ATTVehicle::MulticastState_Implementation(const TArray<uint8>& Bytes)
{
	if (HasAuthority()) return;
	PendingState = Bytes;
	bHavePending = true;
}

void ATTVehicle::MulticastImpact_Implementation(float Strength)
{
	if (Sound) Sound->Impact(Strength);
}

void ATTVehicle::ServerWorkshop_Implementation(int32 Action, int32 Index, float Value) { DoWorkshop(Action, Index, Value); }

// ------------------------------------------------------------------ input

void ATTVehicle::SetupInput()
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
	{
		FEnhancedActionKeyMapping& L = Mapping->MapKey(Str, EKeys::A);
		L.Modifiers.Add(NewObject<UInputModifierNegate>(this));
		FEnhancedActionKeyMapping& L2 = Mapping->MapKey(Str, EKeys::Left);
		L2.Modifiers.Add(NewObject<UInputModifierNegate>(this));
	}
	Mapping->MapKey(Hb, EKeys::SpaceBar);
	Mapping->MapKey(Hb, EKeys::Gamepad_FaceButton_Bottom);
	Mapping->MapKey(Look, EKeys::Mouse2D);
	Mapping->MapKey(Look, EKeys::Gamepad_Right2D);
	const FName Names[] = {"ShiftUp", "ShiftDown", "Camera", "Mechanic", "Reset", "Use", "Loosen", "Replace", "Refill", "Gearbox"};
	const FKey Keys[] = {EKeys::E, EKeys::Q, EKeys::C, EKeys::Tab, EKeys::R, EKeys::LeftMouseButton, EKeys::RightMouseButton, EKeys::F, EKeys::G, EKeys::M};
	for (int32 i = 0; i < 10; i++) Mapping->MapKey(Act(Names[i], EInputActionValueType::Boolean), Keys[i]);
}

void ATTVehicle::SetupPlayerInputComponent(UInputComponent* PIC)
{
	Super::SetupPlayerInputComponent(PIC);
	SetupInput();
	if (APlayerController* PC = Cast<APlayerController>(GetController()))
		if (ULocalPlayer* LP = PC->GetLocalPlayer())
			if (UEnhancedInputLocalPlayerSubsystem* Sub = LP->GetSubsystem<UEnhancedInputLocalPlayerSubsystem>()) Sub->AddMappingContext(Mapping, 0);
	UEnhancedInputComponent* E = Cast<UEnhancedInputComponent>(PIC);
	if (!E) return;
	E->BindAction(Actions["Throttle"], ETriggerEvent::Triggered, this, &ATTVehicle::OnThrottle);
	E->BindAction(Actions["Throttle"], ETriggerEvent::Completed, this, &ATTVehicle::OnThrottle);
	E->BindAction(Actions["Brake"], ETriggerEvent::Triggered, this, &ATTVehicle::OnBrake);
	E->BindAction(Actions["Brake"], ETriggerEvent::Completed, this, &ATTVehicle::OnBrake);
	E->BindAction(Actions["Steer"], ETriggerEvent::Triggered, this, &ATTVehicle::OnSteer);
	E->BindAction(Actions["Steer"], ETriggerEvent::Completed, this, &ATTVehicle::OnSteer);
	E->BindAction(Actions["Handbrake"], ETriggerEvent::Triggered, this, &ATTVehicle::OnHandbrake);
	E->BindAction(Actions["Handbrake"], ETriggerEvent::Completed, this, &ATTVehicle::OnHandbrake);
	E->BindAction(Actions["Look"], ETriggerEvent::Triggered, this, &ATTVehicle::OnLook);
	E->BindAction(Actions["ShiftUp"], ETriggerEvent::Started, this, &ATTVehicle::OnShiftUp);
	E->BindAction(Actions["ShiftDown"], ETriggerEvent::Started, this, &ATTVehicle::OnShiftDown);
	E->BindAction(Actions["Camera"], ETriggerEvent::Started, this, &ATTVehicle::OnCamera);
	E->BindAction(Actions["Mechanic"], ETriggerEvent::Started, this, &ATTVehicle::OnMechanic);
	E->BindAction(Actions["Reset"], ETriggerEvent::Started, this, &ATTVehicle::OnReset);
	E->BindAction(Actions["Use"], ETriggerEvent::Started, this, &ATTVehicle::OnUse);
	E->BindAction(Actions["Loosen"], ETriggerEvent::Started, this, &ATTVehicle::OnLoosen);
	E->BindAction(Actions["Replace"], ETriggerEvent::Started, this, &ATTVehicle::OnReplace);
	E->BindAction(Actions["Refill"], ETriggerEvent::Started, this, &ATTVehicle::OnRefill);
	E->BindAction(Actions["Gearbox"], ETriggerEvent::Started, this, &ATTVehicle::OnGearbox);
}

void ATTVehicle::OnThrottle(const FInputActionValue& V) { InThrottle = View == ETTView::Mechanic ? 0.f : V.Get<float>(); }
void ATTVehicle::OnBrake(const FInputActionValue& V) { InBrake = V.Get<float>(); }
void ATTVehicle::OnSteer(const FInputActionValue& V) { InSteer = View == ETTView::Mechanic ? 0.f : V.Get<float>(); }
void ATTVehicle::OnHandbrake(const FInputActionValue& V) { InHandbrake = V.Get<float>(); }
void ATTVehicle::OnLook(const FInputActionValue& V)
{
	const FVector2D L = V.Get<FVector2D>();
	LookYaw = FMath::Fmod(LookYaw + L.X * 1.5f, 360.f);
	LookPitch = FMath::Clamp(LookPitch + L.Y * 1.5f, -70.f, 20.f);
	if (View == ETTView::Cockpit) Camera->SetRelativeRotation(FRotator(FMath::Clamp(LookPitch + 12.f, -60.f, 40.f), FMath::Clamp(LookYaw, -140.f, 140.f), 0));
}
void ATTVehicle::OnShiftUp()
{
	if (Sim) Sim->gearbox.automatic = false;
	InShift = 1;
}
void ATTVehicle::OnShiftDown()
{
	if (Sim) Sim->gearbox.automatic = false;
	InShift = -1;
}
void ATTVehicle::OnGearbox()
{
	if (Sim) Sim->gearbox.automatic = !Sim->gearbox.automatic;
}
void ATTVehicle::OnCamera()
{
	View = View == ETTView::Chase ? ETTView::Cockpit : ETTView::Chase;
	LookYaw = 0, LookPitch = View == ETTView::Chase ? -12 : 0;
	ApplyView();
}
void ATTVehicle::OnMechanic()
{
	if (!Sim) return;
	if (View != ETTView::Mechanic && FMath::Abs(Sim->speed) > 0.5) return;  // only when parked
	View = View == ETTView::Mechanic ? ETTView::Chase : ETTView::Mechanic;
	ApplyView();
	if (View == ETTView::Mechanic) Note(TEXT("ЛКМ: затянуть гайку  ПКМ: ослабить  F: новая деталь  G: жидкости"), GetActorLocation() + FVector(0, 0, 220));
}
void ATTVehicle::ApplyView()
{
	if (View == ETTView::Cockpit)
	{
		Arm->TargetArmLength = 0.f;
		Arm->bEnableCameraLag = false;
		Arm->bEnableCameraRotationLag = false;
		Arm->bDoCollisionTest = false;
		Arm->SetRelativeLocation(FVector(-15, -36, 112));
		Arm->SocketOffset = FVector::ZeroVector;
		Arm->bUsePawnControlRotation = false;
		Arm->SetUsingAbsoluteRotation(false);
		Arm->SetRelativeRotation(FRotator::ZeroRotator);
		Camera->SetFieldOfView(80.f);
	}
	else
	{
		Arm->SetUsingAbsoluteRotation(true);
		Arm->TargetArmLength = View == ETTView::Mechanic ? 420.f : 650.f;
		Arm->SetRelativeLocation(FVector::ZeroVector);
		Arm->SocketOffset = View == ETTView::Mechanic ? FVector(0, 0, 60) : FVector(0, 0, 160);
		Arm->bEnableCameraLag = View == ETTView::Chase;
		Arm->bEnableCameraRotationLag = View == ETTView::Chase;
		Arm->bDoCollisionTest = true;
		Camera->SetRelativeRotation(FRotator::ZeroRotator);
		Camera->SetFieldOfView(75.f);
	}
}
void ATTVehicle::OnReset()
{
	if (!HasAuthority()) return;
	tt::V3 o, r, f, u;
	Sim->frame(o, r, f, u);
	const double Yaw = std::atan2(-f.x, f.y);
	Sim->place(tt::V3(o.x, o.y, std::max(o.z, 0.0) + 1.0), Yaw);
}

// ------------------------------------------------------------------ workshop (first person, no menus)

int32 ATTVehicle::AimedWheel() const
{
	const APlayerController* PC = Cast<APlayerController>(GetController());
	if (!PC || !Sim) return -1;
	const FVector Eye = PC->PlayerCameraManager->GetCameraLocation(), Dir = PC->PlayerCameraManager->GetCameraRotation().Vector();
	int32 Best = -1;
	double Bd = 60.0;  // cm from the view ray
	for (size_t i = 0; i < Sim->wheels.size(); i++)
	{
		const FVector C = ToUE(Sim->body.nodes[Sim->wheels[i].hub].p);
		const double d = FMath::PointDistToLine(C, Dir, Eye);
		if (d < Bd) Bd = d, Best = int32(i);
	}
	return Best;
}

int32 ATTVehicle::AimedPart() const
{
	const APlayerController* PC = Cast<APlayerController>(GetController());
	if (!PC || !Sim) return -1;
	const FVector Eye = PC->PlayerCameraManager->GetCameraLocation(), Dir = PC->PlayerCameraManager->GetCameraRotation().Vector();
	int32 Best = -1;
	double Bd = 80.0;
	for (size_t p = 0; p < Sim->body.parts.size(); p++)
	{
		const tt::Part& P = Sim->body.parts[p];
		if (P.nodes.empty()) continue;
		tt::V3 c(0, 0, 0);
		for (int n : P.nodes) c += Sim->body.nodes[n].p;
		c = c / double(P.nodes.size());
		const double d = FMath::PointDistToLine(ToUE(c), Dir, Eye);
		if (d < Bd) Bd = d, Best = int32(p);
	}
	return Best;
}

void ATTVehicle::DoWorkshop(int32 Action, int32 Index, float Value)
{
	if (!Sim) return;
	if (Action == 0 && Index >= 0 && Index < int32(Sim->bolts.size())) Sim->tighten(Index, Value);
	if (Action == 1) Sim->replace_part(Index);
	if (Action == 2) Sim->refill();
}

void ATTVehicle::OnUse()
{
	if (View != ETTView::Mechanic || !Sim) return;
	const int32 Wh = AimedWheel();
	if (Wh < 0) return;
	if (Wh != WrenchWheel) WrenchWheel = Wh, WrenchNut = 0;
	// nuts of this wheel in order; each click adds 15 N m (the wrench clicks at the set torque)
	TArray<int32> Nuts;
	for (size_t b = 0; b < Sim->bolts.size(); b++)
		if (Sim->bolts[b].wheel == Wh) Nuts.Add(int32(b));
	if (Nuts.Num() == 0) return;
	const int32 B = Nuts[WrenchNut % Nuts.Num()];
	const tt::Bolt& Bo = Sim->bolts[B];
	const float NewT = float(FMath::Min(Bo.torque + 15.0, Bo.spec * 1.6));
	const bool bAtSpec = Bo.torque < Bo.spec && NewT >= Bo.spec;
	if (HasAuthority()) DoWorkshop(0, B, NewT);
	else ServerWorkshop(0, B, NewT);
	if (bAtSpec) Sound->Click(), WrenchNut++;
	const FVector At = ToUE(Sim->body.nodes[Sim->wheels[Wh].hub].p) + FVector(0, 0, 55);
	Note(FString::Printf(TEXT("гайка %d/%d: %.0f Н·м%s"), WrenchNut % Nuts.Num() + 1, Nuts.Num(), Sim->bolts[B].stripped ? 0.0 : NewT,
	                     Sim->bolts[B].stripped ? TEXT("  (сорвана резьба!)") : bAtSpec ? TEXT("  щёлк") : TEXT("")),
	     At, 2.5f);
}

void ATTVehicle::OnLoosen()
{
	if (View != ETTView::Mechanic || !Sim) return;
	const int32 Wh = AimedWheel();
	if (Wh < 0) return;
	for (size_t b = 0; b < Sim->bolts.size(); b++)
		if (Sim->bolts[b].wheel == Wh)
		{
			if (HasAuthority()) DoWorkshop(0, int32(b), 0.f);
			else ServerWorkshop(0, int32(b), 0.f);
		}
	WrenchWheel = Wh, WrenchNut = 0;
	Note(TEXT("гайки откручены"), ToUE(Sim->body.nodes[Sim->wheels[Wh].hub].p) + FVector(0, 0, 55), 2.f);
}

void ATTVehicle::OnReplace()
{
	if (View != ETTView::Mechanic || !Sim) return;
	const int32 P = AimedPart();
	if (P < 0) return;
	if (HasAuthority()) DoWorkshop(1, P, 0);
	else ServerWorkshop(1, P, 0);
	Note(FString::Printf(TEXT("новая деталь: %s"), UTF8_TO_TCHAR(Sim->body.parts[P].name.c_str())), GetActorLocation() + FVector(0, 0, 200), 2.5f);
}

void ATTVehicle::OnRefill()
{
	if (View != ETTView::Mechanic) return;
	if (HasAuthority()) DoWorkshop(2, 0, 0);
	else ServerWorkshop(2, 0, 0);
	Note(TEXT("масло и охлаждающая жидкость долиты"), GetActorLocation() + FVector(0, 0, 200), 2.f);
}
