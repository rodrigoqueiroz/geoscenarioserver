#include "SimVehicle.h"
#include "Components/SkeletalMeshComponent.h"
#include "UObject/ConstructorHelpers.h"
#include "Engine/SkeletalMesh.h"

ASimVehicle::ASimVehicle()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickInterval = 0.1f;
	mesh = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("BaseMesh"));
    mesh->SetupAttachment(RootComponent);
	static ConstructorHelpers::FObjectFinder<USkeletalMesh> MeshContainer(TEXT("SkeletalMesh'/Game/GeoScenarioContent/Vehicle/Sedan/Sedan_SkelMesh.Sedan_SkelMesh'"));
	if (MeshContainer.Succeeded())
	{
		mesh->SetSkeletalMesh(MeshContainer.Object);
	}
}

void ASimVehicle::BeginPlay()
{
	Super::BeginPlay();
}

void ASimVehicle::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);
}

void ASimVehicle::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

