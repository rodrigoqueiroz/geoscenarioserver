// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "SimManager.h"
#include "WheeledVehicle.h"

#include "Ego.generated.h"


class UInputComponent;


UCLASS(config=Game)
class SHMTEST_API AEgo : public AWheeledVehicle
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AEgo();

	virtual void SetupPlayerInputComponent(UInputComponent* InputComponent) override;


protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	void MoveForward(float Val);
	void MoveRight(float Val);

	// struct ShmInfo shmInfo;

	class USkeletalMeshComponent *mesh;
	
};
