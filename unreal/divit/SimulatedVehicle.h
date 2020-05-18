// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
// #include "SimManager.h"

#include <sys/sem.h>
#include <sys/types.h>

#include "SimulatedVehicle.generated.h"


class ASimManager;

UCLASS()
class SHMTEST_API ASimulatedVehicle : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ASimulatedVehicle();

	void Init();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	ASimManager *manager;
	int32_t id = -1;

	// ShmInfo shmInfo;
	class USkeletalMeshComponent *mesh;
	// class USkeletalMesh *MeshContainer;
	
};
