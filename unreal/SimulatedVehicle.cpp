#include "SimulatedVehicle.h"
#include "Components/SkeletalMeshComponent.h"
#include "UObject/ConstructorHelpers.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sstream>

ASimulatedVehicle::ASimulatedVehicle()
{
    //PrimaryActorTick.bCanEverTick = true;
    //PrimaryActorTick.TickInterval = 0.0;
    
    //mesh = CreateDefaultSubobject<USkeletalMesh>(TEXT("BaseMesh"));
    //mesh->SetupAttachment(RootComponent);
    //static ConstructorHelpers::FObjectFinder<USkeletalMesh> MeshContainer(TEXT("SkeletalMesh'/Game/Vehicle/Sedan/Sedan_SkelMesh.Sedan_SkelMesh'"));
    //static ConstructorHelpers::FObjectFinder<USkeletalMesh> MeshContainer(TEXT("/Game/VehicleVarietyPack/Meshes/SM_SUV"));
    //if (MeshContainer.Succeeded())
    //{
     //   mesh->SetSkeletalMesh(MeshContainer.Object);
    //}

    //StaticMesh
    mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RootComponent"));
    mesh->SetNotifyRigidBodyCollision(true);
    //mesh->bGenerateOverlapEvents = true;
    RootComponent = mesh;
    FString asset_name = FString("/Game/VehicleVarietyPack/Meshes/SM_SUV.SM_SUV");
    UStaticMesh *my_static_asset = Cast<UStaticMesh>(
      StaticLoadObject(UStaticMesh::StaticClass(), NULL, *asset_name));
    mesh->SetStaticMesh(my_static_asset);

    //TODO: add keys to .ini file
    shmInfo.shm_key = 15432;
    shmInfo.sem_key = 16789;
}


void ASimulatedVehicle::BeginPlay()
{
    Super::BeginPlay();
    UE_LOG(LogTemp, Error, TEXT("Vehicle Begin Play. Connecting to Server..."));
    if ((shmInfo.sem_id = semget(shmInfo.sem_key, 1, 0666)) < 0) {
        UE_LOG(LogTemp, Error, TEXT("Error getting semaphore ID\n"));
        return;
    }
    if ((shmInfo.shm_id = shmget(shmInfo.shm_key, 1024, 0666)) < 0) {
        UE_LOG(LogTemp, Error, TEXT("Error getting memory ID\n"));
        return;
    }
    shmInfo.shm = (char*)shmat(shmInfo.shm_id, NULL, 0);
    if (shmInfo.shm == (char*)-1) {
        UE_LOG(LogTemp, Error, TEXT("Error attaching shared memory\n"));
        return;
    }
    GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Black, "Connected to GeoScenario Server");
    vehicle_state = VehicleState();
    frame_stat = FrameStat();
}



void ASimulatedVehicle::Tick(float DeltaTime) 
{
    Super::Tick(DeltaTime);
    //Tick
    frame_stat.tick_count++ ;
    frame_stat.delta_time = DeltaTime;
    
    //Performance Test:
    UE_LOG(LogTemp, Error, TEXT("SimVehicle Actor Tick=%d DeltaTime=%f"), frame_stat.tick_count, frame_stat.delta_time);
    return;
    if (shmInfo.shm_id >= 0) {
        //SHM ACQUIRE
        if (semop(shmInfo.sem_id, &(shmInfo.p), 1) < 0) {
            UE_LOG(LogTemp, Error, TEXT("Cannot p semaphore\n"));
            return;
        }

        //SHM READ
        std::istringstream iss{shmInfo.shm};
        float x, y, z, yaw, x_vel, y_vel, steer, delta_time;
        int vid,tick_count; 
        iss >> vid >> x >> y >> z >> yaw >> x_vel >> y_vel >> steer >> tick_count >> delta_time;
        UE_LOG(LogTemp, Error, TEXT("SHM [ id=%d x=%.2f y=%.2f z=%.2f yaw=%.2f x_vel=%.2f y_vel=%.2f steer=%.2f ] tick=%d delta_time= %.3f"), 
                                            vid, x,     y,     z,     yaw,     x_vel,     y_vel,     steer,       tick_count, delta_time);
        //Output what's in the memory
        // UE_LOG(LogTemp, Error, TEXT("Shared memory %d: %s"), shmInfo.shm_key, iss));

        //SHM RELEASE
        if (semop(shmInfo.sem_id, &(shmInfo.v), 1) < 0) {
            UE_LOG(LogTemp, Error, TEXT("Cannot v semaphore\n"));
            return;
        }
       
        if (server_frame_stat.tick_count == tick_count){ 
            //same tick, no new state
            //Predict new state based on Unreal tick time
            vehicle_state.x  = vehicle_state.x + (vehicle_state.x_vel * DeltaTime);
            vehicle_state.y  = vehicle_state.y + (vehicle_state.y_vel * DeltaTime);
        }
        else{
            vehicle_state.id = vid;
            vehicle_state.x = x;
            vehicle_state.y = y;
            vehicle_state.z = z;
            vehicle_state.yaw = yaw;
            vehicle_state.x_vel = x_vel;
            vehicle_state.y_vel = y_vel;
            vehicle_state.steer = steer;
        }

        server_frame_stat.tick_count = tick_count;
        server_frame_stat.delta_time = delta_time;

        //Update Actor
        vehicle_state.location = FVector(vehicle_state.x,vehicle_state.y,vehicle_state.z);
        //With Interpolation
        //vehicle_state.location = FMath::VInterpTo(GetActorLocation(),vehicle_state.location, DeltaTime, 1.0f); //Current, Target, Time since last tick, Interp Speed
        
        bool moved = SetActorLocation(vehicle_state.location);
        //SetActorRotation(FRotator((180 / 3.14) * FCString::Atof(*positionString[4]), (180 / 3.14) * FCString::Atof(*positionString[5]), (180 / 3.14) * FCString::Atof(*positionString[3])));

        if (!moved) {
            UE_LOG(LogTemp, Error, TEXT("Error moving actor!"));
        }
    }
}

void ASimulatedVehicle::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    Super::EndPlay(EndPlayReason);
    shmdt(shmInfo.shm);
}

