// Fill out your copyright notice in the Description page of Project Settings.


#include "SharedMemoryManager.h"

#include "GameFramework/Pawn.h"
#include "Engine/Engine.h"
#include "EngineUtils.h"

#include <sys/types.h>
#include <sys/shm.h>
#include <sstream>
#include <string.h>


const key_t SHM_KEY = 123456;
const key_t SEM_KEY = 346565;

const key_t EGO_SHM_KEY = 333943;
const key_t EGO_SEM_KEY = 933433;


// Sets default values
ASharedMemoryManager::ASharedMemoryManager()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

    this->frameStat = FrameStat();
	this->serverFrameStat = FrameStat();
}

// Called when the game starts or when spawned
void ASharedMemoryManager::BeginPlay()
{
	Super::BeginPlay();
	
	// Setup shared memory
	this->shmInfo = ShmInfo(SHM_KEY, SEM_KEY);
	GetWorldTimerManager().SetTimer(ConnectionTimerHandler, this, &ASharedMemoryManager::AttemptConnection, 1.0f, true);
}

// Called every frame
void ASharedMemoryManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	this->frameStat.tick_count++;
    this->frameStat.delta_time = DeltaTime;

	ReadSVState(DeltaTime);

	// Find ego actor. If this plugin has spawned, ego is initialized.
	if (!this->egoPawn) {		
		this->egoPawn = GetWorld()->GetFirstPlayerController()->GetPawnOrSpectator();
		if (!this->egoPawn) {
			UE_LOG(LogTemp, Error, TEXT("No Ego found.\n"));
			return;
		} else {
			UE_LOG(LogTemp, Error, TEXT("Ego found.\n"));
			CreateEgoSharedMemory();
		}
	}

	WriteEgoState();
}

void ASharedMemoryManager::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);

	shmdt(this->shmInfo.shm);

	// destroy Ego's shared mem
	// TODO: put destroy and create functions inside the struct?
	if (semctl(egoShmInfo.sem_id, IPC_RMID, 0) < 0) {
		UE_LOG(LogTemp, Error, TEXT("Error destroying semaphore\n"));
		return;
	}

	if (shmdt(egoShmInfo.shm) < 0) {
		UE_LOG(LogTemp, Error, TEXT("Error detaching Ego shared memory\n"));
		return;
	}
	if (shmctl(egoShmInfo.shm_id, IPC_RMID, 0) < 0) {
		UE_LOG(LogTemp, Error, TEXT("Error destroying Ego shared memory\n"));
		return;
	}
}

void ASharedMemoryManager::CreateEgoSharedMemory()
{
	egoShmInfo = ShmInfo{EGO_SHM_KEY, EGO_SEM_KEY};
	
	// get semaphore instance
	if ((egoShmInfo.sem_id = semget(egoShmInfo.sem_key, 1, IPC_CREAT | 0660)) < 0) {
		UE_LOG(LogTemp, Error, TEXT("Error getting Ego semaphore ID\n"));
		return;
	}
	if (semop(egoShmInfo.sem_id, &(egoShmInfo.v), 1) < 0) {
		UE_LOG(LogTemp, Error, TEXT("Cannot v Ego semaphore\n"));
		return;
	}
	
	// get shared mem instance
	if ((egoShmInfo.shm_id = shmget(egoShmInfo.shm_key, 1024, IPC_CREAT | 0660)) < 0) {
		UE_LOG(LogTemp, Error, TEXT("Error getting Ego memory ID\n"));
		return;
	}

	// attach shared memory to this process's address space
	egoShmInfo.shm = (char*)shmat(egoShmInfo.shm_id, NULL, 0);
	if (egoShmInfo.shm == (char*)-1) {
		UE_LOG(LogTemp, Error, TEXT("Error attaching Ego shared memory\n"));
		perror("attach error: ");
		return;
	}
}

void ASharedMemoryManager::WriteEgoState()
{
	if (!this->egoPawn || this->egoShmInfo.shm_id < 0) {
		return;
	}

	// Write out Ego position (in which frame?)
	FVector frenetLocation = this->egoPawn->GetActorLocation() * 0.01f;
	frenetLocation[2] = 0.0f;
	std::stringstream oss;
	oss << frenetLocation[0] << " " << frenetLocation[1] << " " << frenetLocation[2] << '\n';
	
	// write to shm
	if (semop(this->egoShmInfo.sem_id, &(this->egoShmInfo.p), 1) < 0) {
		UE_LOG(LogTemp, Error, TEXT("Cannot p Ego semaphore\n"));
		perror("p error: ");
		return;
	}
	strcpy(this->egoShmInfo.shm, oss.str().c_str());
	if (semop(this->egoShmInfo.sem_id, &(this->egoShmInfo.v), 1) < 0) {
		UE_LOG(LogTemp, Error, TEXT("Cannot v Ego semaphore\n"));
		return;
	}

	// read it back for debug
	if (semop(this->egoShmInfo.sem_id, &(this->egoShmInfo.p), 1) < 0) {
		// UE_LOG(LogTemp, Error, TEXT("Cannot p semaphore\n"));
		return;
	}
	const char *s = this->egoShmInfo.shm;
	FString fs = s;
	UE_LOG(LogTemp, Error, TEXT("Ego Shared memory %d: %s"), this->egoShmInfo.shm_key, *fs);
	if (semop(this->egoShmInfo.sem_id, &(this->egoShmInfo.v), 1) < 0) {
		UE_LOG(LogTemp, Error, TEXT("Cannot v semaphore\n"));
		return;
	}

}

void ASharedMemoryManager::ReadSVState(float deltaTime)
{
	if (!this->isConnected || this->shmInfo.shm_id < 0) {
		return;
	}

	// SEM ACQUIRE
	if (semop(this->shmInfo.sem_id, &(this->shmInfo.p), 1) < 0) {
		UE_LOG(LogTemp, Error, TEXT("Cannot p semaphore\n"));
		return;
	}

	// SHM READ
	std::istringstream iss{this->shmInfo.shm};

	// SEM RELEASE
	if (semop(this->shmInfo.sem_id, &(this->shmInfo.v), 1) < 0) {
		UE_LOG(LogTemp, Error, TEXT("Cannot v semaphore\n"));
		return;
	}
	
	// Parse data and update SV actors
	float server_delta_time;
	int server_tick_count;
	int vid;
	iss >> server_tick_count >> server_delta_time;
	UE_LOG(LogTemp, Error, TEXT("SHM [ tick = %d server_delta_time = %.3f"), server_tick_count, server_delta_time);
	
	while (iss >> vid) {
		// TODO: a more robust way to read in from shared mem
		float x, y, z, yaw, x_vel, y_vel, steer;
		iss >> x >> y >> z >> yaw >> x_vel >> y_vel >> steer;

		VehicleState *vstate = this->simVehicleStates.Find(vid);
		if (!vstate) {
			this->simVehicleStates.Add(vid, VehicleState());
			vstate = this->simVehicleStates.Find(vid);

			// not spawning actor from here
			// ASimulatedVehicle *sv = (ASimulatedVehicle*)GetWorld()->SpawnActor(ASimulatedVehicle::StaticClass(), &location);
		}

		if (this->serverFrameStat.tick_count == server_tick_count) {
			//same tick, no new state
			//Predict new state based on Unreal tick time
			vstate->x  = vstate->x + (vstate->x_vel * deltaTime);
			vstate->y  = vstate->y + (vstate->y_vel * deltaTime);
		} else {
			vstate->id = vid;
			vstate->x = x;
			vstate->y = y;
			vstate->z = z;
			vstate->yaw = yaw;
			vstate->x_vel = x_vel;
			vstate->y_vel = y_vel;
			vstate->steer = steer;
		}

		vstate->location = FVector(vstate->x, vstate->y, vstate->z);
		// vstate->location = FMath::VInterpTo(sim_vehicles[vid]->GetActorLocation(), vstate->location, deltaTime, 1.0f);

		UE_LOG(LogTemp, Error, TEXT("Vehicle [ id=%d x=%.2f y=%.2f z=%.2f yaw=%.2f x_vel=%.2f y_vel=%.2f steer=%.2f ] tick=%d server_delta_time= %.3f"), 
											vid, x,     y,     z,     yaw,     x_vel,     y_vel,     steer,       server_tick_count, server_delta_time);
	}
	
	this->serverFrameStat.tick_count = server_tick_count;
	this->serverFrameStat.delta_time = server_delta_time;
}

void ASharedMemoryManager::AttemptConnection()
{
	// get semaphore instance
	UE_LOG(LogTemp, Error, TEXT("Connecting to Server..."));
    if ((this->shmInfo.sem_id = semget(this->shmInfo.sem_key, 1, 0666)) < 0) {
        UE_LOG(LogTemp, Error, TEXT("Error getting semaphore ID\n"));
        return;
    }

	// get shared mem instance
    if ((this->shmInfo.shm_id = shmget(this->shmInfo.shm_key, 1024, 0666)) < 0) {
        UE_LOG(LogTemp, Error, TEXT("Error getting memory ID\n"));
        return;
    }

	// attach memory to this process's address space
    this->shmInfo.shm = (char*)shmat(this->shmInfo.shm_id, NULL, 0);
    if (this->shmInfo.shm == (char*)-1) {
        UE_LOG(LogTemp, Error, TEXT("Error attaching shared memory\n"));
        return;
    }

    GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Black, "Connected to GeoScenario Server");
	UE_LOG(LogTemp, Warning, TEXT("Connected to GeoScenario Server"));
	
	//clear timer
	GetWorldTimerManager().ClearTimer(ConnectionTimerHandler);

	this->isConnected = true;
	return;
}
