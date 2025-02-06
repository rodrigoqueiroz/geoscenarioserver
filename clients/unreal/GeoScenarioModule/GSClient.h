#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SimVehicle.h"
#include "SimPedestrian.h"
#include "GeoScenarioModule.h"
#include "GSClient.generated.h"

struct ShmInfo
{
	ShmInfo(){}
	ShmInfo(int shared_mem_key, int semaphore_key): shm_key(shared_mem_key), sem_key(semaphore_key){}
	int shm_id = -1;
	key_t shm_key;
	char *shm;
	int sem_id = -1;
	key_t sem_key;
	struct sembuf p = {0, -1, SEM_UNDO | IPC_NOWAIT};	// acquire
	struct sembuf v = {0, 1, SEM_UNDO};					// release
};

struct VehicleState
{
    float x = 0;
    float y = 0;
    float z = 0;
    float x_vel = 0;
    float y_vel = 0;
    float yaw = 0;
    float steer = 0;
};

struct PedestrianState
{
    float x = 0;
    float y = 0;
    float z = 0;
    float x_vel = 0;
    float y_vel = 0;
    float yaw = 0;
};

struct GSVehicle
{
	int vid = -1;
	int v_type = 0;
	VehicleState vehicle_state;
	AActor* actor;
};

struct GSPedestrian
{
	int pid = -1;
	int p_type = 0;
	PedestrianState pedestrian_state;
	AActor* actor;
};

struct FrameStat
{
  float sim_time = 0;
  int tick_count = 0;
  float delta_time = 0;
};

UCLASS()
class GEOSCENARIOMODULE_API AGSClient : public AActor
{
	GENERATED_BODY()

protected:
	ShmInfo ss_shmInfo;
	ShmInfo cs_shmInfo;
	bool isConnected = false;
	FTimerHandle ConnectionTimerHandler;
	FrameStat framestat;
    FrameStat server_framestat;
	TMap<uint16_t, GSVehicle> vehicles;
	TMap<uint16_t, GSPedestrian> pedestrians;
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	AActor* FindVehicleActor(int vid);
	AActor* FindPedestrianActor(int pid);
	void CreateVehicle(int vid, int v_type, FVector &loc, FRotator &rot);
	void CreatePedestrian(int pid, int p_type, FVector &loc, FRotator &rot);
	void ReadServerState(float deltaTime);
	void UpdateRemoteVehicleStates(float deltaTime);
	void UpdateRemotePedestrianStates(float deltaTime);
	void WriteClientState(int tickCount, float deltaTime);

public:
	AGSClient();
	virtual void Tick(float DeltaTime) override;
	void AttemptConnection();
};
