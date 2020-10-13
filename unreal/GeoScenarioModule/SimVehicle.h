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

private:
	void GetBoundingBox(FVector &outPosition, FVector &outExtent, FRotator &outOrientation);
};
