#include "TTHud.h"
#include "TTCar.h"
#include "TTDriver.h"
#include "TTGameMode.h"
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
	// always: money, clock and weather, lap times (small, top right)
	{
		FString Top;
		if (const ATTGameState* GS = GetWorld()->GetGameState<ATTGameState>()) Top += GS->WeatherText();
		if (const APlayerController* PC = GetOwningPlayerController())
			if (const ATTPlayerState* PS = PC->GetPlayerState<ATTPlayerState>())
			{
				Top += FString::Printf(TEXT("\n$%d"), PS->Money);
				if (PS->LastLap > 0.f) Top += FString::Printf(TEXT("\nкруг %d:%05.2f   лучший %d:%05.2f"), int32(PS->LastLap / 60.f), FMath::Fmod(PS->LastLap, 60.f), int32(PS->BestLap / 60.f), FMath::Fmod(PS->BestLap, 60.f));
			}
		TArray<FString> L;
		Top.ParseIntoArray(L, TEXT("\n"), false);
		float Ty = 10.f;
		for (const FString& S : L)
		{
			float W = 0, H = 0;
			GetTextSize(S, W, H, Font, 1.1f);
			DrawText(S, FLinearColor(0, 0, 0, 0.8f), Canvas->ClipX - W - 11.f, Ty + 1, Font, 1.1f);
			DrawText(S, FLinearColor(1, 1, 1, 0.85f), Canvas->ClipX - W - 12.f, Ty, Font, 1.1f);
			Ty += 15.f;
		}
	}
	if (!bShow)
	{
		Lines(TEXT("I — подсказки и телеметрия"), 12, 10, FLinearColor(1, 1, 1, 0.35f));
		return;
	}
	float Y = 10.f;
	APawn* P = GetOwningPawn();
	if (const ATTCar* Car = Cast<ATTCar>(P))
	{
		Y = Lines(Car->DebugReport(), 12, Y, FLinearColor(0.85f, 1.f, 0.85f));
		Lines(TEXT("W/S газ-тормоз (на автомате S стоя = задний)   A/D руль   Пробел ручник   E/Q передачи   M автомат/ручная\n"
		           "C камера   L фары   R эвакуатор ($50)   F выйти   Tab механик (ЛКМ/ПКМ гайки)   O линии подвески   N +3 часа   K дождь"),
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
		           "T изолента $5   V холодная сварка $25 (перед машины = радиатор)   G антифриз+масло $40   H перебрать двигатель $900   Y тюнинг $300\n"
		           "R: деталь $180, колесо $120   P — дать $100 игроку, на которого смотришь   N +3 часа   K дождь"),
		      12, Y, FLinearColor(1, 0.9f, 0.6f));
	}
}
