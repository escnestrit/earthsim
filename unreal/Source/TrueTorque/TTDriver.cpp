#include "TTDriver.h"
#include "TTCar.h"
#include "Camera/CameraComponent.h"
#include "Components/CapsuleComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/TextRenderComponent.h"
#include "EngineUtils.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "Engine/LocalPlayer.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "GameFramework/PlayerController.h"
#include "InputAction.h"
#include "InputMappingContext.h"
#include "InputModifiers.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "UObject/ConstructorHelpers.h"

ATTDriver::ATTDriver()
{
	bReplicates = true;
	GetCapsuleComponent()->InitCapsuleSize(34.f, 90.f);
	bUseControllerRotationYaw = true;
	GetCharacterMovement()->MaxWalkSpeed = 450.f;
	GetCharacterMovement()->JumpZVelocity = 380.f;
	GetCharacterMovement()->bOrientRotationToMovement = false;

	Eyes = CreateDefaultSubobject<UCameraComponent>(TEXT("Eyes"));
	Eyes->SetupAttachment(GetCapsuleComponent());
	Eyes->SetRelativeLocation(FVector(10.f, 0.f, 68.f));
	Eyes->bUsePawnControlRotation = true;
	Eyes->SetFieldOfView(85.f);

	// a simple figure so other players see you (hidden from your own eyes)
	static ConstructorHelpers::FObjectFinder<UStaticMesh> Cyl(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> Sph(TEXT("/Engine/BasicShapes/Sphere.Sphere"));
	BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
	BodyMesh->SetupAttachment(GetCapsuleComponent());
	BodyMesh->SetStaticMesh(Cyl.Object);
	BodyMesh->SetRelativeScale3D(FVector(0.55f, 0.4f, 1.45f));
	BodyMesh->SetRelativeLocation(FVector(0.f, 0.f, -15.f));
	BodyMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	BodyMesh->SetOwnerNoSee(true);
	HeadMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("HeadMesh"));
	HeadMesh->SetupAttachment(GetCapsuleComponent());
	HeadMesh->SetStaticMesh(Sph.Object);
	HeadMesh->SetRelativeScale3D(FVector(0.25f));
	HeadMesh->SetRelativeLocation(FVector(0.f, 0.f, 72.f));
	HeadMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	HeadMesh->SetOwnerNoSee(true);

	Dot = CreateDefaultSubobject<UTextRenderComponent>(TEXT("Dot"));
	Dot->SetupAttachment(Eyes);
	Dot->SetRelativeLocation(FVector(40.f, 0.f, -0.5f));
	Dot->SetRelativeRotation(FRotator(0.f, 180.f, 0.f));
	Dot->SetHorizontalAlignment(EHTA_Center);
	Dot->SetVerticalAlignment(EVRTA_TextCenter);
	Dot->SetWorldSize(1.2f);
	Dot->SetText(FText::FromString(TEXT("+")));
	Dot->SetTextRenderColor(FColor(255, 255, 255, 180));
	Dot->SetOnlyOwnerSee(true);
	Dot->SetCastShadow(false);
}

void ATTDriver::SetupPlayerInputComponent(UInputComponent* PIC)
{
	Super::SetupPlayerInputComponent(PIC);
	if (!Mapping)
	{
		Mapping = NewObject<UInputMappingContext>(this);
		auto Act = [&](FName Name, EInputActionValueType Type) {
			UInputAction* A = NewObject<UInputAction>(this, FName(*(FString(TEXT("IA_")) + Name.ToString())));
			A->ValueType = Type;
			Actions.Add(Name, A);
			return A;
		};
		UInputAction* Move = Act("Move", EInputActionValueType::Axis2D);
		UInputAction* Look = Act("Look", EInputActionValueType::Axis2D);
		// WASD -> 2D: W = +Y, S = -Y, D = +X, A = -X
		Mapping->MapKey(Move, EKeys::W).Modifiers.Add(NewObject<UInputModifierSwizzleAxis>(this));
		{
			FEnhancedActionKeyMapping& S = Mapping->MapKey(Move, EKeys::S);
			S.Modifiers.Add(NewObject<UInputModifierSwizzleAxis>(this));
			S.Modifiers.Add(NewObject<UInputModifierNegate>(this));
		}
		Mapping->MapKey(Move, EKeys::D);
		Mapping->MapKey(Move, EKeys::A).Modifiers.Add(NewObject<UInputModifierNegate>(this));
		Mapping->MapKey(Move, EKeys::Gamepad_Left2D);
		Mapping->MapKey(Look, EKeys::Mouse2D);
		Mapping->MapKey(Look, EKeys::Gamepad_Right2D);
		const FName Names[] = {"Jump", "Enter", "Tighten", "Loosen", "Repair", "Tape", "Weld", "Fluids", "Engine", "Swap"};
		const FKey Keys[] = {EKeys::SpaceBar, EKeys::F, EKeys::LeftMouseButton, EKeys::RightMouseButton, EKeys::R, EKeys::T, EKeys::V, EKeys::G, EKeys::H, EKeys::Y};
		for (int32 i = 0; i < int32(UE_ARRAY_COUNT(Names)); i++) Mapping->MapKey(Act(Names[i], EInputActionValueType::Boolean), Keys[i]);
	}
	if (APlayerController* PC = Cast<APlayerController>(GetController()))
		if (ULocalPlayer* LP = PC->GetLocalPlayer())
			if (UEnhancedInputLocalPlayerSubsystem* Sub = LP->GetSubsystem<UEnhancedInputLocalPlayerSubsystem>())
			{
				Sub->ClearAllMappings();
				Sub->AddMappingContext(Mapping, 0);
			}
	UEnhancedInputComponent* E = Cast<UEnhancedInputComponent>(PIC);
	if (!E) return;
	E->BindAction(Actions["Move"], ETriggerEvent::Triggered, this, &ATTDriver::OnMove);
	E->BindAction(Actions["Look"], ETriggerEvent::Triggered, this, &ATTDriver::OnLook);
	E->BindAction(Actions["Jump"], ETriggerEvent::Started, this, &ATTDriver::OnJump);
	E->BindAction(Actions["Enter"], ETriggerEvent::Started, this, &ATTDriver::OnEnter);
	E->BindAction(Actions["Tighten"], ETriggerEvent::Started, this, &ATTDriver::OnTighten);
	E->BindAction(Actions["Loosen"], ETriggerEvent::Started, this, &ATTDriver::OnLoosen);
	E->BindAction(Actions["Repair"], ETriggerEvent::Started, this, &ATTDriver::OnRepair);
	E->BindAction(Actions["Tape"], ETriggerEvent::Started, this, &ATTDriver::OnTape);
	E->BindAction(Actions["Weld"], ETriggerEvent::Started, this, &ATTDriver::OnWeld);
	E->BindAction(Actions["Fluids"], ETriggerEvent::Started, this, &ATTDriver::OnFluids);
	E->BindAction(Actions["Engine"], ETriggerEvent::Started, this, &ATTDriver::OnEngine);
	E->BindAction(Actions["Swap"], ETriggerEvent::Started, this, &ATTDriver::OnSwap);
}

void ATTDriver::OnMove(const FInputActionValue& V)
{
	const FVector2D M = V.Get<FVector2D>();
	const FRotator Yaw(0.f, GetControlRotation().Yaw, 0.f);
	AddMovementInput(FRotationMatrix(Yaw).GetUnitAxis(EAxis::X), M.Y);
	AddMovementInput(FRotationMatrix(Yaw).GetUnitAxis(EAxis::Y), M.X);
}

void ATTDriver::OnLook(const FInputActionValue& V)
{
	const FVector2D L = V.Get<FVector2D>();
	AddControllerYawInput(L.X * 0.6f);
	AddControllerPitchInput(-L.Y * 0.6f);
}

void ATTDriver::OnJump() { Jump(); }

// the car you look at (or the nearest one within arm's reach of the view ray)
ATTCar* ATTDriver::LookedAtCar(FVector& Eye, FVector& Dir) const
{
	Eye = Eyes->GetComponentLocation(), Dir = Eyes->GetForwardVector();
	FHitResult H;
	FCollisionQueryParams Q(SCENE_QUERY_STAT(TTLook), false, this);
	if (GetWorld()->LineTraceSingleByChannel(H, Eye, Eye + Dir * 400.f, ECC_Visibility, Q))
		if (ATTCar* C = Cast<ATTCar>(H.GetActor())) return C;
	ATTCar* Best = nullptr;
	double Bd = 480.0;
	for (TActorIterator<ATTCar> It(GetWorld()); It; ++It)
	{
		const double d = FVector::Dist(It->GetActorLocation(), Eye);
		if (d < Bd) Bd = d, Best = *It;
	}
	return Best;
}

void ATTDriver::OnEnter()
{
	FVector Eye, Dir;
	ATTCar* Car = LookedAtCar(Eye, Dir);
	if (!Car || FVector::Dist(Car->GetActorLocation(), GetActorLocation()) > 450.f) return;
	ServerEnter(Car);
}

void ATTDriver::ServerEnter_Implementation(ATTCar* Car)
{
	AController* C = GetController();
	if (!C || !Car || Car->GetController() || FVector::Dist(Car->GetActorLocation(), GetActorLocation()) > 500.f) return;
	C->UnPossess();
	C->Possess(Car);
	Destroy();
}

void ATTDriver::ServerCarCommand_Implementation(ATTCar* Car, uint8 Cmd, int32 Index)
{
	if (Car && FVector::Dist(Car->GetActorLocation(), GetActorLocation()) < 650.f) Car->DoCommand(Cmd, Index);
}

void ATTDriver::Tool(uint8 Cmd)
{
	FVector Eye, Dir;
	ATTCar* Car = LookedAtCar(Eye, Dir);
	if (!Car) return;
	int32 Index = -1;
	const int32 W = Car->AimedWheel(Eye, Dir), P = Car->AimedPart(Eye, Dir);
	switch (Cmd)
	{
	case TTCmd::Tighten:
	case TTCmd::Loosen:
		Index = W;
		if (Index < 0) return;
		break;
	case TTCmd::RepairPart:
		Index = W >= 0 && Car->WheelNeedsWork(W) ? 100 + W : P;
		if (Index < 0) return;
		break;
	case TTCmd::ColdWeld:
	{
		// looking at the front of the car = the radiator
		const FVector Local = Car->GetActorTransform().InverseTransformPosition(Eye + Dir * FMath::Max(0.0, FVector::DotProduct(Car->GetActorLocation() - Eye, Dir)));
		Index = Local.X > 100.f ? -1 : P;
		break;
	}
	case TTCmd::Tape:
	case TTCmd::SwapVariant:
		Index = P;
		if (Index < 0) return;
		break;
	default:
		Index = 0;
		break;
	}
	Car->ShowCommand(Cmd, Index);
	if (HasAuthority()) Car->DoCommand(Cmd, Index);
	else ServerCarCommand(Car, Cmd, Index);
}

void ATTDriver::OnTighten() { Tool(TTCmd::Tighten); }
void ATTDriver::OnLoosen() { Tool(TTCmd::Loosen); }
void ATTDriver::OnRepair() { Tool(TTCmd::RepairPart); }
void ATTDriver::OnTape() { Tool(TTCmd::Tape); }
void ATTDriver::OnWeld() { Tool(TTCmd::ColdWeld); }
void ATTDriver::OnFluids() { Tool(TTCmd::Fluids); }
void ATTDriver::OnEngine() { Tool(TTCmd::EngineRebuild); }
void ATTDriver::OnSwap() { Tool(TTCmd::SwapVariant); }
