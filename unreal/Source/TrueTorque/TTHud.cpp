#include "TTHud.h"
#include "TTCar.h"
#include "TTDriver.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/Font.h"

bool ATTHUD::bShow = true;

void ATTHUD::DrawHUD()
{
	Super::DrawHUD();
	if (!Canvas) return;
	UFont* Font = GEngine->GetSmallFont();
	auto Lines = [&](const FString& Text, float X, float Y, const FLinearColor& Col) {
		TArray<FString> L;
		Text.ParseIntoArray(L, TEXT("\n"), false);
		for (const FString& S : L)
		{
			DrawText(S, FLinearColor(0, 0, 0, 0.8f), X + 1, Y + 1, Font, 1.1f);
			DrawText(S, Col, X, Y, Font, 1.1f);
			Y += 15.f;
		}
		return Y;
	};
	if (!bShow)
	{
		Lines(TEXT("F1 — подсказки и телеметрия"), 12, 10, FLinearColor(1, 1, 1, 0.35f));
		return;
	}
	float Y = 10.f;
	APawn* P = GetOwningPawn();
	if (const ATTCar* Car = Cast<ATTCar>(P))
	{
		Y = Lines(Car->DebugReport(), 12, Y, FLinearColor(0.85f, 1.f, 0.85f));
		Lines(TEXT("W/S газ-тормоз (на автомате S стоя = задний)   A/D руль   Пробел ручник   E/Q передачи   M автомат/ручная\n"
		           "C камера   L фары   R поставить на колёса   F выйти   Tab механик (ЛКМ/ПКМ гайки)   F2 линии подвески"),
		      12, Y + 8, FLinearColor(1, 0.9f, 0.6f));
	}
	else if (const ATTDriver* D = Cast<ATTDriver>(P))
	{
		// what the hands would work on: printed under the aim point
		const FString Aim = D->AimText();
		if (!Aim.IsEmpty())
		{
			TArray<FString> L;
			Aim.ParseIntoArray(L, TEXT("\n"), false);
			float Ay = Canvas->ClipY * 0.5f + 24.f;
			for (const FString& S : L)
			{
				float W = 0, H = 0;
				GetTextSize(S, W, H, Font, 1.2f);
				DrawText(S, FLinearColor(0, 0, 0, 0.8f), (Canvas->ClipX - W) * 0.5f + 1, Ay + 1, Font, 1.2f);
				DrawText(S, FLinearColor(1, 1, 0.75f), (Canvas->ClipX - W) * 0.5f, Ay, Font, 1.2f);
				Ay += 17.f;
			}
		}
		Lines(TEXT("Пешком: WASD, мышь, Пробел.  F — сесть в машину (любую свободную)\n"
		           "Смотри на деталь:  ЛКМ ключ на гайку (120 Н·м, дальше сорвёшь)   ПКМ открутить   R выправить/поставить\n"
		           "T изолента   V холодная сварка (перед машины = радиатор)   G антифриз+масло   H перебрать двигатель   Y тюнинг-вариант"),
		      12, Y, FLinearColor(1, 0.9f, 0.6f));
	}
}
