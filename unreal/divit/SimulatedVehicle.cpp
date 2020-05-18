// Fill out your copyright notice in the Description page of Project Settings.


#include "SimulatedVehicle.h"

#include "Components/SkeletalMeshComponent.h"
#include "UObject/ConstructorHelpers.h"
#include "Engine/SkeletalMesh.h"
#include "SimManager.h"



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
}

