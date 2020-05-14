// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include <sys/sem.h>

#include "SharedMemoryManager.generated.h"


struct ShmInfo
{
	ShmInfo()
	{
	}

	ShmInfo(int shared_mem_key, int semaphore_key)
		: shm_key(shared_mem_key), sem_key(semaphore_key)
	{
		
	}

	int shm_id = -1;
	key_t shm_key;
	char *shm;

	int sem_id = -1;
	key_t sem_key;

	// define operations to be performed on semaphores
	struct sembuf p = {0, -1, SEM_UNDO | IPC_NOWAIT};	// acquire
	struct sembuf v = {0, 1, SEM_UNDO};					// release
};

/**
 * To keep track of SV states being read in from GS server.
 */
struct VehicleState
{
    int id = 0;
    float x = 0;
    float y = 0;
    float z = 0;
    float yaw = 0;
    //float pitch = 0;
    //float roll = 0;
    float x_vel = 0;
    float y_vel = 0;
    float steer = 0;
    FVector location;
    FVector rotation;
};

/**
 * Used to sync frames between unreal and GS server.
 */
struct FrameStat
{
    int tick_count = 0;
    float delta_time = 0;
};


UCLASS()
class SHAREDMEMORYMODULE_API ASharedMemoryManager : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ASharedMemoryManager();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;


private:
	void AttemptConnection();
	FTimerHandle ConnectionTimerHandler;
	
	// Read and update state of all sim vehicles from shared memory.
	void ReadSVState(float deltaTime);

	void CreateEgoSharedMemory();
	void WriteEgoState();

	ShmInfo shmInfo;
	FrameStat frameStat;
    FrameStat serverFrameStat;
	bool isConnected = false;

	ShmInfo egoShmInfo;
	APawn *egoPawn = nullptr;

	// GS server SV ID -> SV state
	TMap<uint16_t, VehicleState> simVehicleStates;
	// TMap<uint16_t, ASimulatedVehicle*> sim_vehicles;
	
};
