#include "TTEngineSound.h"

UTTEngineSound::UTTEngineSound(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
	NumChannels = 1;
	bAutoActivate = true;
}

bool UTTEngineSound::Init(int32& SampleRate)
{
	Rate = float(SampleRate);
	NumChannels = 1;
	return true;
}

void UTTEngineSound::SetState(float InRpm, float InThrottle, float InKnock, float InSquealSlip, float InSpeed, bool bInRunning)
{
	Rpm = InRpm;
	Throttle = InThrottle;
	Knock = InKnock;
	Squeal = InSquealSlip;
	Speed = InSpeed;
	bRunning = bInRunning;
}

void UTTEngineSound::Impact(float Strength) { ImpactAmp = FMath::Max(ImpactAmp.load(), FMath::Clamp(Strength, 0.f, 1.f)); }
void UTTEngineSound::Click() { ClickAmp = 1.f; }

float UTTEngineSound::Noise()
{
	Rng = Rng * 1664525u + 1013904223u;
	return float(Rng >> 8) / float(1 << 24) * 2.f - 1.f;
}

int32 UTTEngineSound::OnGenerateAudio(float* OutAudio, int32 NumSamples)
{
	const float TargetRpm = bRunning ? Rpm.load() : 0.f;
	const float TargetThr = Throttle.load();
	const float Kn = Knock.load();
	const float Sq = FMath::Clamp(Squeal.load(), 0.f, 1.f);
	const float Spd = Speed.load();
	if (ImpactAmp > 0.f) { ImpactEnv = FMath::Max(ImpactEnv, ImpactAmp.load()); ImpactAmp = 0.f; }
	if (ClickAmp > 0.f) { ClickEnv = 1.f; ClickAmp = 0.f; }
	for (int32 i = 0; i < NumSamples; i++)
	{
		CurRpm += (TargetRpm - CurRpm) * 0.0008f;
		CurThr += (TargetThr - CurThr) * 0.0005f;
		// four-stroke, four cylinders: two firings per revolution
		const float Fire = CurRpm / 60.f * 2.f;
		Phase += Fire / Rate;
		if (Phase > 1.0) Phase -= 1.0;
		const float P = float(Phase) * 2.f * PI;
		float Eng = 0.f;
		if (CurRpm > 50.f)
		{
			Eng = 0.55f * FMath::Sin(P) + 0.3f * FMath::Sin(2.f * P + 0.4f) + 0.18f * FMath::Sin(3.f * P + 1.1f) + 0.12f * FMath::Sin(0.5f * P);
			// combustion pulses: sharper with throttle
			const float Pulse = FMath::Pow(FMath::Max(0.f, FMath::Sin(P)), 6.f);
			Eng += Pulse * (0.4f + 0.8f * CurThr);
			LowNoise += (Noise() - LowNoise) * 0.05f;
			Eng += LowNoise * (0.15f + 0.5f * CurThr);
			// knock: a metallic tick on some firings
			if (Kn > 0.f && Phase < 0.02 && Noise() < Kn * 2.f - 1.f) Eng += 0.9f * Kn;
			Eng *= 0.18f + 0.22f * CurThr + CurRpm / 9000.f * 0.3f;
		}
		// tyre squeal: narrow band noise around 900 Hz
		BandNoise += (Noise() - BandNoise) * 0.3f;
		SquealPhase += (850.f + 120.f * FMath::Sin(float(SquealPhase) * 0.001f)) / Rate;
		const float SquealS = Sq * 0.25f * FMath::Sin(float(SquealPhase) * 2.f * PI) * (0.6f + 0.4f * BandNoise);
		// rolling / wind noise
		Smooth += (Noise() - Smooth) * 0.02f;
		const float Road = Smooth * FMath::Clamp(Spd / 40.f, 0.f, 1.f) * 0.25f;
		// crash and ratchet
		const float Bang = ImpactEnv * Noise() * 0.9f;
		ImpactEnv *= 0.9993f;
		const float Ratchet = ClickEnv * Noise() * 0.5f;
		ClickEnv *= 0.995f;
		OutAudio[i] = FMath::Clamp(Eng + SquealS + Road + Bang + Ratchet, -1.f, 1.f) * 0.6f;
	}
	return NumSamples;
}
