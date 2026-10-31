// Procedural car audio: engine harmonics from the firing frequency, intake/exhaust noise, knock when
// the engine is damaged, tyre squeal and crash bangs. Parameters are set from the game thread.
#pragma once
#include "CoreMinimal.h"
#include "Components/SynthComponent.h"
#include <atomic>
#include "TTEngineSound.generated.h"

UCLASS(ClassGroup = Synth, meta = (BlueprintSpawnableComponent))
class TRUETORQUE_API UTTEngineSound : public USynthComponent
{
	GENERATED_BODY()
public:
	UTTEngineSound(const FObjectInitializer& ObjectInitializer);

	// game thread -> audio thread (atomics, read once per buffer)
	void SetState(float InRpm, float InThrottle, float InKnock, float InSquealSlip, float InSpeed, bool bInRunning);
	void Impact(float Strength);  // 0..1
	void Click();                 // torque wrench ratchet

protected:
	virtual bool Init(int32& SampleRate) override;
	virtual int32 OnGenerateAudio(float* OutAudio, int32 NumSamples) override;

private:
	std::atomic<float> Rpm{850.f}, Throttle{0.f}, Knock{0.f}, Squeal{0.f}, Speed{0.f}, ImpactAmp{0.f}, ClickAmp{0.f};
	std::atomic<bool> bRunning{true};
	float Rate = 48000.f;
	double Phase = 0.0, SquealPhase = 0.0;
	float Smooth = 0.f, LowNoise = 0.f, BandNoise = 0.f, CurRpm = 850.f, CurThr = 0.f, ImpactEnv = 0.f, ClickEnv = 0.f;
	uint32 Rng = 12345;
	float Noise();
};
