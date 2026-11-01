// Debug overlay (F1): what the car is doing, numbers behind the no-HUD game. Off by default.
#pragma once
#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "TTHud.generated.h"

UCLASS()
class TRUETORQUE_API ATTHUD : public AHUD
{
	GENERATED_BODY()
public:
	virtual void DrawHUD() override;
	static bool bShow;
};
