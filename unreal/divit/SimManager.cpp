// Fill out your copyright notice in the Description page of Project Settings.


#include "SimManager.h"

// shared memory and semaphores
#include <sys/ipc.h>
#include <sys/shm.h>

#include "Ego.h"
#include "EngineUtils.h"

#include <sstream>


const key_t SHM_KEY = 123456;
const key_t SEM_KEY = 346565;

const key_t EGO_SHM_KEY = 333943;
const key_t EGO_SEM_KEY = 933433;


// Sets default values
ASimManager::ASimManager()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ASimManager::BeginPlay()
{
	Super::BeginPlay();

    frameStat = FrameStat();
	serverFrameStat = FrameStat();

	// Setup shared memory
	shmInfo = ShmInfo(SHM_KEY, SEM_KEY);	
	GetWorldTimerManager().SetTimer(ConnectionTimerHandler, this, &ASimManager::AttemptConnection, 1.0f, true);


	// Find ego actor
	TActorIterator<AEgo> egoItr(GetWorld());
	if (egoItr) {
		ego = *egoItr;
		if (ego) {
			UE_LOG(LogTemp, Error, TEXT("Ego found.\n"));
			CreateEgoSharedMemory();
		}

	} else {
		UE_LOG(LogTemp, Error, TEXT("No Ego found.\n"));
	}
}


// Called every frame
void ASimManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	frameStat.tick_count++ ;
    frameStat.delta_time = DeltaTime;
	//UE_LOG(LogTemp, Error, TEXT("SimVehicle Actor Tick=%d DeltaTime=%f"), frame_stat.tick_count, frame_stat.delta_time);

	if (isConnected && shmInfo.shm_id > 0) {
		// SHM ACQUIRE
		if (semop(shmInfo.sem_id, &(shmInfo.p), 1) < 0) {
			UE_LOG(LogTemp, Error, TEXT("Cannot p semaphore\n"));
			return;
		}

		// SHM READ
		std::istringstream iss{shmInfo.shm};
		float delta_time;
		int tick_count;
		int vid;

		// SHM RELEASE
		if (semop(shmInfo.sem_id, &(shmInfo.v), 1) < 0) {
			UE_LOG(LogTemp, Error, TEXT("Cannot v semaphore\n"));
			return;
		}
		
		// TODO: a more robust way to read in from shared mem
		iss >> tick_count >> delta_time;
		UE_LOG(LogTemp, Error, TEXT("SHM [ tick = %d delta_time = %.3f"), tick_count, delta_time);
		while (iss >> vid) {
			float x, y, z, yaw, x_vel, y_vel, steer;
			iss >> x >> y >> z >> yaw >> x_vel >> y_vel >> steer;

			VehicleState *vstate = sim_vehicle_states.Find(vid);
			// spawn SV actor if it doesn't exist
			if (!vstate) {
				sim_vehicle_states.Add(vid, VehicleState());
				vstate = sim_vehicle_states.Find(vid);

				FVector location = {0.0, 0.0, 110.0};
				ASimulatedVehicle *sv = (ASimulatedVehicle*)GetWorld()->SpawnActor(ASimulatedVehicle::StaticClass(), &location);
				sv->Init();
				sv->manager = this;
				sv->id = vid;
				sim_vehicles.Add(sv);
			}

			vstate->id = vid;
			vstate->x = x;
			vstate->y = y;
			vstate->z = z;
			vstate->yaw = yaw;
			vstate->x_vel = x_vel;
			vstate->y_vel = y_vel;
			vstate->steer = steer;

			vstate->location = FVector(x,y,z);

			UE_LOG(LogTemp, Error, TEXT("Vehicle [ id=%d x=%.2f y=%.2f z=%.2f yaw=%.2f x_vel=%.2f y_vel=%.2f steer=%.2f ] tick=%d delta_time= %.3f"), 
												vid, x,     y,     z,     yaw,     x_vel,     y_vel,     steer,       tick_count, delta_time);
		}
		
		serverFrameStat.tick_count = tick_count;
		serverFrameStat.delta_time = delta_time;
	}

	// Output what's in the memory
	// UE_LOG(LogTemp, Error, TEXT("Shared memory %d: %s"), shmInfo.shm_key, iss.str().c_str());

	// if (server_frame_stat.tick_count == tick_count){
	// 	//same tick, no new state
	// 	//Predict new state based on Unreal tick time
	// 	vehicle_state.x  = vehicle_state.x + (vehicle_state.x_vel * DeltaTime);
	// 	vehicle_state.y  = vehicle_state.y + (vehicle_state.y_vel * DeltaTime);
	// }
	// else {
	// 	vehicle_state.id = vid;
	// 	vehicle_state.x = x;
	// 	vehicle_state.y = y;
	// 	vehicle_state.z = z;
	// 	vehicle_state.yaw = yaw;
	// 	vehicle_state.x_vel = x_vel;
	// 	vehicle_state.y_vel = y_vel;
	// 	vehicle_state.steer = steer;
	// }


	//Update Actor
	//With Interpolation
	//vehicle_state.location = FMath::VInterpTo(GetActorLocation(),vehicle_state.location, DeltaTime, 1.0f); //Current, Target, Time since last tick, Interp Speed
	
	// bool moved = SetActorLocation(vehicle_state.location);

	// Write out Ego position
	if (ego) {
		FVector frenetLocation = ego->GetActorLocation() * 0.01f;
		frenetLocation[2] = 0.0f;
		std::stringstream oss;
		oss << frenetLocation[0] << " " << frenetLocation[1] << " " << frenetLocation[2] << '\n';
		
		// write to shm
		if (semop(egoShmInfo.sem_id, &(egoShmInfo.p), 1) < 0) {
			UE_LOG(LogTemp, Error, TEXT("Cannot p Ego semaphore\n"));
			perror("p error: ");
			return;
		}
		strcpy(egoShmInfo.shm, oss.str().c_str());
		if (semop(egoShmInfo.sem_id, &(egoShmInfo.v), 1) < 0) {
			UE_LOG(LogTemp, Error, TEXT("Cannot v Ego semaphore\n"));
			return;
		}

		// read it back
		if (semop(egoShmInfo.sem_id, &(egoShmInfo.p), 1) < 0) {
			// UE_LOG(LogTemp, Error, TEXT("Cannot p semaphore\n"));
			return;
		}
		const char *s = egoShmInfo.shm;
		FString fs = s;
		UE_LOG(LogTemp, Error, TEXT("Ego Shared memory %d: %s"), egoShmInfo.shm_key, *fs);
		if (semop(egoShmInfo.sem_id, &(egoShmInfo.v), 1) < 0) {
			UE_LOG(LogTemp, Error, TEXT("Cannot v semaphore\n"));
			return;
		}
	}

}


void ASimManager::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);

	shmdt(shmInfo.shm);

	// destroy Ego's shared mem
	if (semctl(shmInfo.sem_id, IPC_RMID, 0) < 0) {
		UE_LOG(LogTemp, Error, TEXT("Error destroying semaphore\n"));
		return;
	}

	if (shmdt(shmInfo.shm) < 0) {
		UE_LOG(LogTemp, Error, TEXT("Error detaching Ego shared memory\n"));
		return;
	}
	if (shmctl(shmInfo.shm_id, IPC_RMID, 0) < 0) {
		UE_LOG(LogTemp, Error, TEXT("Error destroying Ego shared memory\n"));
		return;
	}
}


void ASimManager::AttemptConnection()
{
	// get semaphore instance
	UE_LOG(LogTemp, Error, TEXT("Connecting to Server..."));
    if ((shmInfo.sem_id = semget(shmInfo.sem_key, 1, 0666)) < 0) {
        UE_LOG(LogTemp, Error, TEXT("Error getting semaphore ID\n"));
        return;
    }

	// get shared mem instance
    if ((shmInfo.shm_id = shmget(shmInfo.shm_key, 1024, 0666)) < 0) {
        UE_LOG(LogTemp, Error, TEXT("Error getting memory ID\n"));
        return;
    }

	// attach memory to this process's address space
    shmInfo.shm = (char*)shmat(shmInfo.shm_id, NULL, 0);
    if (shmInfo.shm == (char*)-1) {
        UE_LOG(LogTemp, Error, TEXT("Error attaching shared memory\n"));
        return;
    }

    GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Black, "Connected to GeoScenario Server");
	UE_LOG(LogTemp, Warning, TEXT("Connected to GeoScenario Server"));
	
	//clear timer
	GetWorldTimerManager().ClearTimer(ConnectionTimerHandler);

	isConnected = true;
	return;
}

void ASimManager::CreateEgoSharedMemory()
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

