//d43sharm@edu.uwaterloo.ca
//rqueiroz@gsd.uwaterloo.ca
// ---------------------------------------------
// UNREAL SIMULATED VEHICLE CLIENT
// Must be added to Unreal Project. Connects to GeoScenario Server.
// --------------------------------------------
#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include <sys/sem.h>
#include <sys/types.h>
#include "SimulatedVehicle.generated.h"

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
    struct sembuf p = {0, -1, SEM_UNDO | IPC_NOWAIT};   // acquire
    struct sembuf v = {0, 1, SEM_UNDO};                 // release
};

struct VehicleState{
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

struct FrameStat{
    int tick_count = 0;
    float delta_time = 0;
};

UCLASS()
class VEHICLEMANEUVER_API ASimulatedVehicle : public APawn
{
    GENERATED_BODY()

    FrameStat frame_stat;
    FrameStat server_frame_stat;
    VehicleState vehicle_state;
    
public: 
    // Sets default values for this actor's properties
    ASimulatedVehicle();
    //void Init();
protected:
    // Called when the game starts or when spawned
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
public: 
    // Called every frame
    virtual void Tick(float DeltaTime) override;
    ShmInfo shmInfo;
    //class USkeletalMeshComponent *mesh;
    UStaticMeshComponent *mesh;
    
};