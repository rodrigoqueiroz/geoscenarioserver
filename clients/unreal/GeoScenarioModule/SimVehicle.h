#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include <sys/sem.h>
#include <sys/types.h>
#include "SimVehicle.generated.h"

class AGSClientActor;

UCLASS()
class GEOSCENARIOMODULE_API ASimVehicle: public AActor
{
	GENERATED_BODY()
	
protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	UStaticMeshComponent *Mesh;

public:	
	ASimVehicle();
	virtual void Tick(float DeltaTime) override;
	// class USkeletalMeshComponent *mesh;

	void SetActive(bool active);
	bool GetActive() const;

private:
	void GetBoundingBox(FVector &outPosition, FVector &outExtent, FRotator &outOrientation);

	UFUNCTION()
	void OnOverlap(AActor *self, AActor *other);

	UFUNCTION()
	void OnHit(class UPrimitiveComponent* HitComp, class AActor* OtherActor, class UPrimitiveComponent* OtherComp, int32 otherBodyIndex, bool fromSweep, const FHitResult& Hit);

	bool isActive = false;
};
