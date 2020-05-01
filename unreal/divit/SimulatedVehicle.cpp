// Fill out your copyright notice in the Description page of Project Settings.


#include "SimulatedVehicle.h"

#include "Components/SkeletalMeshComponent.h"
#include "UObject/ConstructorHelpers.h"
#include "Engine/SkeletalMesh.h"
#include "SimManager.h"

// #include <stdlib.h>
// #include <string.h>
// #include <stdio.h>
// #include <unistd.h>



// Random shared mem vars
// TODO: extend for multiple vehicles
// int shmid;
// key_t key = 123456;
// char *shm;

// key_t semkey = 346565;
// int semid;


// Sets default values
ASimulatedVehicle::ASimulatedVehicle()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickInterval = 0.1f;

	mesh = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("BaseMesh"));
    mesh->SetupAttachment(RootComponent);
	
	static ConstructorHelpers::FObjectFinder<USkeletalMesh> MeshContainer(TEXT("SkeletalMesh'/Game/Vehicle/Sedan/Sedan_SkelMesh.Sedan_SkelMesh'"));
	if (MeshContainer.Succeeded())
	{
		// mesh = GetMesh();
		mesh->SetSkeletalMesh(MeshContainer.Object);
	}

	// if(BaseMeshAsset.Object)
	// {
	// 	BaseMesh->SetStaticMesh(BaseMeshAsset.Object);
	// }
}


void ASimulatedVehicle::Init()
{
	UE_LOG(LogTemp, Error, TEXT("init"));

	// // get semaphore instance - these might not be getting destroyed properly
	// if ((shmInfo.sem_id = semget(shmInfo.sem_key, 1, 0666)) < 0) {
	// 	UE_LOG(LogTemp, Error, TEXT("Error getting semaphore ID\n"));
	// 	return;
	// }

	// // get shared mem instance
	// if ((shmInfo.shm_id = shmget(shmInfo.shm_key, 1024, 0666)) < 0) {
	// 	UE_LOG(LogTemp, Error, TEXT("Error getting memory ID\n"));
	// 	return;
	// }

	// // attach shared memory to this process's address space
	// shmInfo.shm = (char*)shmat(shmInfo.shm_id, NULL, 0);
	// if (shmInfo.shm == (char*)-1) {
	// 	UE_LOG(LogTemp, Error, TEXT("Error attaching shared memory\n"));
	// 	return;
	// }
}


// Called when the game starts or when spawned
void ASimulatedVehicle::BeginPlay()
{
	Super::BeginPlay();

	Init();
	
}


void ASimulatedVehicle::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);

	// shmdt(shmInfo.shm);
}


// Called every frame
void ASimulatedVehicle::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	//Update Actor
	if (id < 0) return;

	FVector location = manager->sim_vehicle_states[id].location;
	bool moved = SetActorLocation(FVector(location[0], location[1], GetActorLocation()[2]));

	//-- Shared mem controller
	// if (shmInfo.shm_id >= 0) {

	// 	if (semop(shmInfo.sem_id, &(shmInfo.p), 1) < 0) {
	// 		// UE_LOG(LogTemp, Error, TEXT("Cannot p semaphore\n"));
	// 		return;
	// 	}

	// 	// read memory as a string, convert back to floats later
	// 	// FString fs = FString(shmInfo.shm);
	// 	std::istringstream iss{shmInfo.shm};
	// 	float x, y, z, yaw;
	// 	iss >> x >> y >> z >> yaw;
	// 	if (semop(shmInfo.sem_id, &(shmInfo.v), 1) < 0) {
	// 		// UE_LOG(LogTemp, Error, TEXT("Cannot v semaphore\n"));
	// 		return;
	// 	}

	// 	// TODO: updating d pos not working
	// 	// +X in UE4 is forward (corresponds to position.z from planner)
	// 	bool moved = SetActorLocation(FVector(x, y, GetActorLocation()[2]));
	// 	if (!moved) {
	// 		UE_LOG(LogTemp, Error, TEXT("No Move!"));
	// 	}
	// 	// SetActorRotation(FRotator((180 / 3.14) * FCString::Atof(*positionString[4]), (180 / 3.14) * FCString::Atof(*positionString[5]), (180 / 3.14) * FCString::Atof(*positionString[3])));

	// 	// output what's in the memory
	// 	// UE_LOG(LogTemp, Error, TEXT("Shared memory %d: %s: %s"), shmInfo.shm_key, 10.0 * FCString::Atof(*positionString[0]), 10.0 * FCString::Atof(*positionString[1]));
	// 	UE_LOG(LogTemp, Error, TEXT("Shared memory %d: %f, %f, yaw: %f"), shmInfo.shm_key, x, y, yaw);

	// }
}

