#include "GSClient.h"
#include <sys/ipc.h>
#include <sys/shm.h>
#include <errno.h>
#include "EngineUtils.h"
#include "Misc/DateTime.h"
#include <sstream>
#include <string>

const key_t  SHM_KEY = 123456;
const key_t  SEM_KEY = 346565;
const key_t  CS_SHM_KEY = 333943;
const key_t  CS_SEM_KEY = 933433;
const size_t SHM_SIZE = 2048;

AGSClient::AGSClient()
{
	PrimaryActorTick.bCanEverTick = true;
}

void AGSClient::BeginPlay()
{
	Super::BeginPlay();
	UE_LOG(GeoScenarioModule, Warning, TEXT("GS Client Begin Play"));
    framestat = FrameStat();
	server_framestat = FrameStat();
	// Setup shared memory
	isConnected = false;
	ss_shmInfo = ShmInfo(SHM_KEY, SEM_KEY);	
	cs_shmInfo = ShmInfo(CS_SHM_KEY, CS_SEM_KEY);	
	GetWorldTimerManager().SetTimer(ConnectionTimerHandler, this, &AGSClient::AttemptConnection, 1.0f, true);
}

void AGSClient::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	framestat.tick_count++;
    framestat.delta_time = DeltaTime;
	//UE_LOG(GeoScenarioModule, Log, TEXT("GSClient TICK=%d DeltaTime=%f"), framestat.tick_count, framestat.delta_time);
	// Update vehicles coming from the server
	ReadServerState(DeltaTime);
	// Update states of remote vehicles
	UpdateRemoteVehicleStates(DeltaTime);
	WriteClientState(framestat.tick_count, framestat.delta_time);

	// log for experiments
	std::stringstream oss;
	
	for (auto& Elem : vehicles)
	{
		GSVehicle &gsv = Elem.Value;
		//Write out Client Vehicle states
		//todo: include full state
		if (gsv.actor != nullptr)
		{
			FVector loc = gsv.actor->GetActorLocation();
			loc[2] = 0.0f;
			oss << Elem.Key << " "
				<< gsv.vehicle_state.x << " " << gsv.vehicle_state.y << " " << gsv.vehicle_state.z << " "
				<< gsv.vehicle_state.x_vel << " " << gsv.vehicle_state.y_vel << '\n';
		}
	}

	UE_LOG(GeoScenarioModule, Log, TEXT("%s | %s"), *FString(oss.str().c_str()), *FDateTime::Now().ToString());
}

void AGSClient::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);
	shmdt(ss_shmInfo.shm);
	shmdt(cs_shmInfo.shm);
}

void AGSClient::AttemptConnection()
{
	//Server State SHM
	// get semaphore instance
	UE_LOG(GeoScenarioModule, Warning, TEXT("Connecting to GeoScenario Server ..."));
    if ((ss_shmInfo.sem_id = semget(ss_shmInfo.sem_key, 1, 0666)) < 0) {
        UE_LOG(GeoScenarioModule, Error, TEXT("Error getting SS semaphore ID. Server down?"));
		UE_LOG(GeoScenarioModule, Error, TEXT("%s"), *FString(strerror(errno)));
        return;
    }
	// get shared mem instance
    if ((ss_shmInfo.shm_id = shmget(ss_shmInfo.shm_key, SHM_SIZE, 0666)) < 0) {
        UE_LOG(GeoScenarioModule, Error, TEXT("Error getting SS memory ID"));
		UE_LOG(GeoScenarioModule, Error, TEXT("%s"), *FString(strerror(errno)));
        return;
    }
	// attach memory to this process's address space
    ss_shmInfo.shm = (char*)shmat(ss_shmInfo.shm_id, NULL, 0);
    if (ss_shmInfo.shm == (char*)-1) {
        UE_LOG(GeoScenarioModule, Error, TEXT("Error attaching SS shared memory"));
		UE_LOG(GeoScenarioModule, Error, TEXT("%s"), *FString(strerror(errno)));
        return;
    }
	
	//Client State SHM
	// get semaphore instance
    if ((cs_shmInfo.sem_id = semget(cs_shmInfo.sem_key, 1, 0666)) < 0) {
        UE_LOG(GeoScenarioModule, Error, TEXT("Error getting CS semaphore ID"));
		UE_LOG(GeoScenarioModule, Error, TEXT("%s"), *FString(strerror(errno)));
        return;
    }
	// get shared mem instance
    if ((cs_shmInfo.shm_id = shmget(cs_shmInfo.shm_key, SHM_SIZE, 0666)) < 0) {
        UE_LOG(GeoScenarioModule, Error, TEXT("Error getting CS memory ID"));
		UE_LOG(GeoScenarioModule, Error, TEXT("%s"), *FString(strerror(errno)));
        return;
    }
	// attach memory to this process's address space
    cs_shmInfo.shm = (char*)shmat(cs_shmInfo.shm_id, NULL, 0);
    if (cs_shmInfo.shm == (char*)-1) {
        UE_LOG(GeoScenarioModule, Error, TEXT("Error attaching CS shared memory"));
		UE_LOG(GeoScenarioModule, Error, TEXT("%s"), *FString(strerror(errno)));
        return;
    }
	//Connected :)
	isConnected = true;
    GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Black, "====>>>>> Connected to GeoScenario Server");
	UE_LOG(GeoScenarioModule, Warning, TEXT("====>>>>> Connected to GeoScenario Server"));
	//clear timer
	GetWorldTimerManager().ClearTimer(ConnectionTimerHandler);
	return;
}

void AGSClient::ReadServerState(float deltaTime)
{
	if (!isConnected || ss_shmInfo.shm_id < 0) { return; }

	// SS SHM ACQUIRE
	if (semop(ss_shmInfo.sem_id, &(ss_shmInfo.p), 1) < 0) {
		UE_LOG(GeoScenarioModule, Error, TEXT("Cannot p SS semaphore. Server disconnected? "));
		UE_LOG(GeoScenarioModule, Error, TEXT("%s"), *FString(strerror(errno)));
		// isConnected = false;
		return;
	}
	// SS SHM READ
	std::istringstream iss{ss_shmInfo.shm};
	FString fulliss(iss.str().c_str());
	//std::stringstream ss;
	//ss << iss.rdbuf();
	// SS SHM RELEASE
	if (semop(ss_shmInfo.sem_id, &(ss_shmInfo.v), 1) < 0) { 
		UE_LOG(GeoScenarioModule, Error, TEXT("Cannot v SS semaphore"));
		UE_LOG(GeoScenarioModule, Error, TEXT("%s"), *FString(strerror(errno)));
		// isConnected = false;
		return;
	}
	//parse frame stats
	float server_delta_time;
	int server_tick_count, nvehicles, vid;
	iss >> server_tick_count >> server_delta_time >> nvehicles;
	
	// parse vehicles
	int vehicles_read = 0;
	while (vehicles_read < nvehicles)
	{
		iss >> vid;
		vehicles_read++;

		if (vid==0) {continue;} //garbage at the end of string

		int type;
		float x, y, z, yaw, x_vel, y_vel, steer;
		iss >> type >> x >> y >> z >> yaw >> x_vel >> y_vel >> steer;
		// Unreal's y axis is inverted from GS server's.
		y *= -1;
		y_vel *= -1;
		yaw -= 90;
	    // GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Red, FString::Printf(TEXT("Yaw %f"), yaw));


		GSVehicle* gsvptr = vehicles.Find(vid);
		if (!gsvptr)
		{
			UE_LOG(GeoScenarioModule, Warning, TEXT("DEBUG Full ISS"));
			UE_LOG(GeoScenarioModule, Log, TEXT("%s"), *fulliss);
			//creates only if actor is spawned or found.
			CreateVehicle(vid, type);
			//debug
			continue;
		}

		if (type == 1)
		{
			if (server_framestat.tick_count == server_tick_count) 
			{
				//same tick, no new state
				//Predict new state based on Unreal tick time
				gsvptr->vehicle_state.x  = gsvptr->vehicle_state.x + (gsvptr->vehicle_state.x_vel * deltaTime);
				gsvptr->vehicle_state.y  = gsvptr->vehicle_state.y + (gsvptr->vehicle_state.y_vel * deltaTime);
			}
			else 
			{
				gsvptr->vehicle_state.x = x;
				gsvptr->vehicle_state.y = y;
				gsvptr->vehicle_state.z = z;
				gsvptr->vehicle_state.yaw = yaw;
				gsvptr->vehicle_state.x_vel = x_vel;
				gsvptr->vehicle_state.y_vel = y_vel;
				gsvptr->vehicle_state.steer = steer;
			}
			FVector loc = FVector(gsvptr->vehicle_state.x, gsvptr->vehicle_state.y, gsvptr->actor->GetActorLocation()[2]);
			gsvptr->actor->SetActorLocation(loc);
			gsvptr->actor->SetActorRotation(FRotator(0.0f, yaw, 0.0f));
		}
	}
	//Update Server Tick
	server_framestat.tick_count = server_tick_count;
	server_framestat.delta_time = server_delta_time;
	//UE_LOG(GeoScenarioModule, Log, TEXT("SHM [ tick = %d server_delta_time = %.3f"), server_tick_count, server_delta_time);
}

void AGSClient::UpdateRemoteVehicleStates(float deltaTime)
{
	for (auto& elem : vehicles)
	{
		GSVehicle &gsv = elem.Value;
		if (gsv.type == 0) continue;

		// Update the vehicle's vehicle_state based on its actor's location.
		// Actual movement of the vehicle is updated in another class.
		// Need to ensure remote movement is finished before reading its state
		// Maybe using a different component with a different tick group?
		FVector loc = gsv.actor->GetActorLocation();
		// update velocity
		gsv.vehicle_state.x_vel = (loc[0] - gsv.vehicle_state.x) / deltaTime;
		gsv.vehicle_state.y_vel = (loc[1] - gsv.vehicle_state.y) / deltaTime;
		// update position
		gsv.vehicle_state.x = loc[0];
		gsv.vehicle_state.y = loc[1];
		gsv.vehicle_state.z = loc[2];
	}
}

void AGSClient::CreateVehicle(int vid, int type)
{
	UE_LOG(GeoScenarioModule, Warning, TEXT("New GSVehicle vid=%d type=%d"), vid, type);
	GSVehicle gsv = GSVehicle();
	gsv.vid = vid;
	gsv.type = type;
	gsv.vehicle_state =  VehicleState();
	if (type == 1) // SDV
	{
		// spawn actor
		UE_LOG(GeoScenarioModule, Log, TEXT("Spawning Sim Vehicle"));
		FVector location = {0.0, 0.0, 0.0};
		ASimVehicle *sv = (ASimVehicle*)GetWorld()->SpawnActor(ASimVehicle::StaticClass(), &location);
		//sv->manager = this;
		//sv->id = vid;
		gsv.actor = (AActor*) sv;

		// add the tag to server vehicles
		FString GSVehicle = "gsvehicle";
		FName GSTag = FName(*GSVehicle);
		gsv.actor->Tags.Add(GSTag);

		// add the tag to publish bbox
		FString PubBbox = "Bbox:1";
		FName BboxTag = FName(*PubBbox);
		gsv.actor->Tags.Add(BboxTag);
	}
	else // EV, TV
	{
		//Find actor with tag
		UE_LOG(GeoScenarioModule, Log, TEXT("Finding Remote Vehicle"));
		gsv.actor = FindVehicleActor(vid);
		FVector loc = gsv.actor->GetActorLocation();
		gsv.vehicle_state.x = loc[0];
		gsv.vehicle_state.y = loc[1];
		gsv.vehicle_state.z = loc[2];
	}
	//check if success
	if (gsv.actor != nullptr)
	{
		vehicles.Add(vid, gsv);
	}
	else {UE_LOG(GeoScenarioModule, Error, TEXT("Error creating GSVehicle vid=%d type=%d"), vid, type);}
}


AActor* AGSClient::FindVehicleActor(int vid)
{
	static const FName GSTAG(TEXT("gsvehicle")); //faster than FString
	for(TActorIterator<AActor> Itr(GetWorld()); Itr; ++Itr)
	{
		if (Itr->ActorHasTag(GSTAG)) 
		{ 
			for (FName tag: Itr->Tags)
			{
				FString str = tag.ToString();
				if (str.Contains(":"))
				{
					FString key, value;
 					str.Split(TEXT(":"),&key,&value);
					//UE_LOG(GeoScenarioModule, Log, TEXT("KEY %s VALUE %s"), *key, *value);
					if (key == "vid")
					{
						int32 tagvid = FCString::Atoi(*value);
						if (vid == tagvid)
						{
							UE_LOG(GeoScenarioModule, Log, TEXT("GeoScenario vehicle actor FOUND. Id %d"),tagvid);
							return *Itr;
						} 
					}
				}
			}
		}
	}
	UE_LOG(GeoScenarioModule, Error, TEXT("GeoScenario vehicle actor NOT found. Id %d"),vid);
	return nullptr;
}


void AGSClient::WriteClientState(int tickCount, float deltaTime)
{
	if (!isConnected || cs_shmInfo.shm_id < 0) { return; }

	std::stringstream oss;
	oss << tickCount << " " << deltaTime << " " << vehicles.Num() << '\n';
	for (auto& Elem : vehicles)
	{
		GSVehicle &gsv = Elem.Value;
		//Write out Client Vehicle states
		//todo: include full state
		if (gsv.actor != nullptr)
		{
			FVector loc = gsv.actor->GetActorLocation();
			loc[2] = 0.0f;
			ASimVehicle *sv = Cast<ASimVehicle>(gsv.actor);
			int active = sv != nullptr ? (int)(sv->GetActive()) : 1;
			oss << Elem.Key << " "
				<< gsv.vehicle_state.x << " " << gsv.vehicle_state.y << " " << gsv.vehicle_state.z << " "
				<< gsv.vehicle_state.x_vel << " " << gsv.vehicle_state.y_vel << " "
				<< active << '\n';
		}
		else
		{
			UE_LOG(GeoScenarioModule, Error, TEXT("Cannot write Vehicle state to CS ShM. Actor is null."));
		}
	}

	// CS SHM ACQUIRE
	if (semop(cs_shmInfo.sem_id, &(cs_shmInfo.p), 1) < 0) {
		UE_LOG(GeoScenarioModule, Error, TEXT("Acquiring CS semaphore failed. Server disconnected? "));
		UE_LOG(GeoScenarioModule, Error, TEXT("%s"), *FString(strerror(errno)));
		return;
	}
	// CS SHM WRITE
	strcpy(cs_shmInfo.shm, oss.str().c_str());
	// CS SHM RELEASE
	if (semop(cs_shmInfo.sem_id, &(cs_shmInfo.v), 1) < 0) {
		UE_LOG(GeoScenarioModule, Error, TEXT("Cannot v CS semaphore"));
		UE_LOG(GeoScenarioModule, Error, TEXT("%s"), *FString(strerror(errno)));
		return;
	}
}
