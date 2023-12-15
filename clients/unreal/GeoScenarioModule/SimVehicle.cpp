#include "SimVehicle.h"
#include "Components/SkeletalMeshComponent.h"
#include "UObject/ConstructorHelpers.h"
#include "Engine/SkeletalMesh.h"
#include "Components/BoxComponent.h"
#include "EngineUtils.h"
#include "GeoScenarioModule.h"

ASimVehicle::ASimVehicle()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickInterval = 0.05f;
	// mesh = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("BaseMesh"));
    // mesh->SetupAttachment(RootComponent);
	// static ConstructorHelpers::FObjectFinder<USkeletalMesh> MeshContainer(TEXT("SkeletalMesh'/Game/GeoScenarioContent/Vehicle/Sedan/Sedan_SkelMesh.Sedan_SkelMesh'"));
	// if (MeshContainer.Succeeded())
	// {
	// 	mesh->SetSkeletalMesh(MeshContainer.Object);
	// }

	// set up the mesh for server vehicles
	this->Mesh =
      CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RootComponent"));
    this->Mesh->SetNotifyRigidBodyCollision(true);
    RootComponent = this->Mesh;

    // SetActorEnableCollision(true);

	FString MeshName = "/Game/Agents/Vehicles/SUV/SUVMesh.SUVMesh";
	UStaticMesh *MeshAsset = Cast<UStaticMesh>(
      StaticLoadObject(UStaticMesh::StaticClass(), NULL, *MeshName));
    Mesh->SetStaticMesh(MeshAsset);
}

void ASimVehicle::BeginPlay()
{
	Super::BeginPlay();

    // bounding box
	FVector outExt;
    FVector outPos;
    FRotator outOrien;

    // This must be done after the game has begun
    UBoxComponent *Box = NewObject<UBoxComponent>(this);

    GetBoundingBox(outPos, outExt, outOrien);
    // UE_LOG(GeoScenarioModule, Log, TEXT("The bounding box out extents: %f, %f, %f"), outExt.X, outExt.Y, outExt.Z);

	// Register the box component on the vehicles that interact with ego
    Box->RegisterComponent();
    Box->SetBoxExtent(FVector(outExt.X, outExt.Y, outExt.Z));
    Box->AttachToComponent(
      RootComponent,
      FAttachmentTransformRules(EAttachmentRule::KeepRelative, false));

    this->OnActorBeginOverlap.AddDynamic(this, &ASimVehicle::OnOverlap);

    this->isActive = true;
}

void ASimVehicle::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);
}

void ASimVehicle::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void ASimVehicle::GetBoundingBox(FVector &outPosition, FVector &outExtent, FRotator &outOrientation)
{
    // Get location and rotation of the actor in world coordinate frame to place
    // the bounding box in the correct position
    outPosition = GetActorLocation();
    outOrientation = GetActorRotation();
    outExtent = FVector(0, 0, 0);

    TArray<UStaticMeshComponent *> StaticMeshComponents;
    GetComponents<UStaticMeshComponent>(StaticMeshComponents);

    if (StaticMeshComponents.Num() >= 1) {
        auto mesh = StaticMeshComponents[0]->GetStaticMesh();
        FBoxSphereBounds AgentBounds = mesh->ExtendedBounds;

        // Combines all static mesh bounds
        for (UStaticMeshComponent *Component : StaticMeshComponents) {
            AgentBounds =
              AgentBounds + Component->GetStaticMesh()->ExtendedBounds;
        }

        outExtent = AgentBounds.BoxExtent;

        // Moves the origin position from the bottom to the center of the mesh,
        // assuming the origin is intially at the bottom of the mesh
        outPosition.Z += AgentBounds.BoxExtent.Z;
    }
}

void ASimVehicle::OnOverlap(AActor *self, AActor *other)
{
    // UE_LOG(LogTemp, Error, TEXT("OVERLAP"));

    static const FName GSTAG(TEXT("gsvehicle"));
    if (other->ActorHasTag(GSTAG)) {
        UE_LOG(GeoScenarioModule, Warning,
                TEXT("GS vehicle %s collided with %s"),
                *self->GetName(),
                *other->GetName());
        this->isActive = false;

        // Check if collided with a gs vehicle
        ASimVehicle *otherGSV = Cast<ASimVehicle>(other);
        if (otherGSV == nullptr) {
            UE_LOG(GeoScenarioModule, Warning, TEXT("Collided with ego"));
        } else {
            UE_LOG(GeoScenarioModule, Warning, TEXT("Collided with GSV"));
            otherGSV->SetActive(false);
        }
    }

    // TODO: add check for pedestrians
}

void ASimVehicle::SetActive(bool active)
{
    this->isActive = active;
}

bool ASimVehicle::GetActive() const
{
    return this->isActive;
}

void ASimVehicle::OnHit(class UPrimitiveComponent* HitComp, class AActor* OtherActor, class UPrimitiveComponent* OtherComp, int32 otherBodyIndex, bool fromSweep, const FHitResult& Hit)
{
    // UE_LOG(LogTemp, Error, TEXT("COLLISION"));
}
