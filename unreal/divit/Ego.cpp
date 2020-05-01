// Fill out your copyright notice in the Description page of Project Settings.

#include "Ego.h"

#include "../ShmTestWheelFront.h"
#include "../ShmTestWheelRear.h"
#include "Components/InputComponent.h"
#include "WheeledVehicleMovementComponent4W.h"
#include "Engine/SkeletalMesh.h"

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#include <string.h>


// Sets default values
AEgo::AEgo()
{
	PrimaryActorTick.bCanEverTick = true;
	
	// mesh = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("BaseMesh"));
    // mesh->SetupAttachment(RootComponent);
	
	static ConstructorHelpers::FObjectFinder<USkeletalMesh> MeshContainer(TEXT("SkeletalMesh'/Game/Vehicle/Sedan/Sedan_SkelMesh.Sedan_SkelMesh'"));
	GetMesh()->SetSkeletalMesh(MeshContainer.Object);
	// if (MeshContainer.Succeeded())
	// {
	// 	// mesh = GetMesh();
	// }

	// Simulation
	UWheeledVehicleMovementComponent4W* Vehicle4W = CastChecked<UWheeledVehicleMovementComponent4W>(GetVehicleMovement());

	check(Vehicle4W->WheelSetups.Num() == 4);

	Vehicle4W->WheelSetups[0].WheelClass = UShmTestWheelFront::StaticClass();
	Vehicle4W->WheelSetups[0].BoneName = FName("Wheel_Front_Left");
	Vehicle4W->WheelSetups[0].AdditionalOffset = FVector(0.f, -12.f, 0.f);

	Vehicle4W->WheelSetups[1].WheelClass = UShmTestWheelFront::StaticClass();
	Vehicle4W->WheelSetups[1].BoneName = FName("Wheel_Front_Right");
	Vehicle4W->WheelSetups[1].AdditionalOffset = FVector(0.f, 12.f, 0.f);

	Vehicle4W->WheelSetups[2].WheelClass = UShmTestWheelRear::StaticClass();
	Vehicle4W->WheelSetups[2].BoneName = FName("Wheel_Rear_Left");
	Vehicle4W->WheelSetups[2].AdditionalOffset = FVector(0.f, -12.f, 0.f);

	Vehicle4W->WheelSetups[3].WheelClass = UShmTestWheelRear::StaticClass();
	Vehicle4W->WheelSetups[3].BoneName = FName("Wheel_Rear_Right");
	Vehicle4W->WheelSetups[3].AdditionalOffset = FVector(0.f, 12.f, 0.f);

	// Shared mem (remove)
	// shmInfo = ShmInfo{333943, 933433};
}


void AEgo::SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent)
{
	UE_LOG(LogTemp, Error, TEXT("moving\n"));
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	// set up gameplay key bindings
	check(PlayerInputComponent);

	PlayerInputComponent->BindAxis("MoveForward", this, &AEgo::MoveForward);
	PlayerInputComponent->BindAxis("MoveRight", this, &AEgo::MoveRight);
	// PlayerInputComponent->BindAxis("LookUp");
	// PlayerInputComponent->BindAxis("LookRight");

	// PlayerInputComponent->BindAction("Handbrake", IE_Pressed, this, &AEgo::OnHandbrakePressed);
	// PlayerInputComponent->BindAction("Handbrake", IE_Released, this, &AEgo::OnHandbrakeReleased);
}


void AEgo::MoveForward(float Val)
{
	GetVehicleMovementComponent()->SetThrottleInput(Val);
	// UE_LOG(LogTemp, Error, TEXT("moving %f\n"), Val);
}


void AEgo::MoveRight(float Val)
{
	GetVehicleMovementComponent()->SetSteeringInput(Val);
	// UE_LOG(LogTemp, Error, TEXT("moving\n"));
}


// Called when the game starts or when spawned
void AEgo::BeginPlay()
{
	Super::BeginPlay();

}

void AEgo::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);

}

// Called every frame
void AEgo::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);


}

