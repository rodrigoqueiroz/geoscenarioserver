#include "GSClient.h"
#include <sys/ipc.h>
#include <sys/shm.h>
#include "EngineUtils.h"
#include <sstream>
#include <string>

const key_t SHM_KEY = 123456;
const key_t SEM_KEY = 346565;
const key_t CS_SHM_KEY = 333943;
const key_t CS_SEM_KEY = 933433;

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
	ReadServerState(DeltaTime);
	WriteClientState();
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
        UE_LOG(GeoScenarioModule, Error, TEXT("Error getting SS semaphore ID. Server down?\n"));
        return;
    }
	// get shared mem instance
    if ((ss_shmInfo.shm_id = shmget(ss_shmInfo.shm_key, 1024, 0666)) < 0) {
        UE_LOG(GeoScenarioModule, Error, TEXT("Error getting SS memory ID\n"));
        return;
    }
	// attach memory to this process's address space
    ss_shmInfo.shm = (char*)shmat(ss_shmInfo.shm_id, NULL, 0);
    if (ss_shmInfo.shm == (char*)-1) {
        UE_LOG(GeoScenarioModule, Error, TEXT("Error attaching SS shared memory\n"));
        return;
    }
	
	//Client State SHM
	// get semaphore instance
    if ((cs_shmInfo.sem_id = semget(cs_shmInfo.sem_key, 1, 0666)) < 0) {
        UE_LOG(GeoScenarioModule, Error, TEXT("Error getting CS semaphore ID\n"));
        return;
    }
	// get shared mem instance
    if ((cs_shmInfo.shm_id = shmget(cs_shmInfo.shm_key, 1024, 0666)) < 0) {
        UE_LOG(GeoScenarioModule, Error, TEXT("Error getting CS memory ID\n"));
        return;
    }
	// attach memory to this process's address space
    cs_shmInfo.shm = (char*)shmat(cs_shmInfo.shm_id, NULL, 0);
    if (cs_shmInfo.shm == (char*)-1) {
        UE_LOG(GeoScenarioModule, Error, TEXT("Error attaching CS shared memory\n"));
        return;
    }
	//Connected :)
	isConnected = true;
    GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Black, "====>>>>> Connected to GeoScenario Server");
	UE_LOG(GeoScenarioModule, Warning, TEXT("====>>>>> Connected to GeoScenario Server\n"));
	//clear timer
	GetWorldTimerManager().ClearTimer(ConnectionTimerHandler);
	return;
}

void AGSClient::ReadServerState(float deltaTime)
{
	if (!isConnected || ss_shmInfo.shm_id < 0) { return; }

	// SS SHM ACQUIRE
	if (semop(ss_shmInfo.sem_id, &(ss_shmInfo.p), 1) < 0) {
		UE_LOG(GeoScenarioModule, Error, TEXT("Cannot p SS semaphore. Server disconnected? \n"));
		isConnected = false;
		return;
	}
	// SS SHM READ
	std::istringstream iss{ss_shmInfo.shm};
	FString fulliss(iss.str().c_str());
	//std::stringstream ss;
	//ss << iss.rdbuf();
	// SS SHM RELEASE
	if (semop(ss_shmInfo.sem_id, &(ss_shmInfo.v), 1) < 0) { 
		UE_LOG(GeoScenarioModule, Error, TEXT("Cannot v SS semaphore\n"));
		isConnected = false;
		return;
	}
	//parse frame stats
	float server_delta_time;
	int server_tick_count;
	int vid;
	iss >> server_tick_count >> server_delta_time;
	server_framestat.tick_count = server_tick_count;
	server_framestat.delta_time = server_delta_time;
	//UE_LOG(GeoScenarioModule, Log, TEXT("SHM [ tick = %d server_delta_time = %.3f"), server_tick_count, server_delta_time);
	
	// parse vehicles
	while (iss >> vid)
	{
		if (vid==0) {continue;} //garbage at the end of string

		int remote;
		float x, y, z, yaw, x_vel, y_vel, steer;
		iss >> remote >> x >> y >> z >> yaw >> x_vel >> y_vel >> steer;
		//
		GSVehicle* gsvptr = vehicles.Find(vid);
		if (!gsvptr)
		{
			//creates only if actor is spawned or found.
			CreateVehicle(vid, remote); 
			//debug
			UE_LOG(GeoScenarioModule, Warning, TEXT("Full ISS"));
			UE_LOG(GeoScenarioModule, Log, TEXT("%s"), *fulliss);
			continue;
		}
		//
		if (gsvptr && remote == 0)
		{
			if (server_framestat.tick_count == server_tick_count) 
			{
				//same tick, no new state
				//Predict new state based on Unreal tick time
				gsvptr->vehicle_state.x  = gsvptr->vehicle_state.x + (gsvptr->vehicle_state.x_vel * deltaTime);
				gsvptr->vehicle_state.y  = gsvptr->vehicle_state.y + (gsvptr->vehicle_state.y_vel * deltaTime);
			} else 
			{
				gsvptr->vehicle_state.x = x;
				gsvptr->vehicle_state.y = y;
				gsvptr->vehicle_state.z = z;
				gsvptr->vehicle_state.yaw = yaw;
				gsvptr->vehicle_state.x_vel = x_vel;
				gsvptr->vehicle_state.y_vel = y_vel;
				gsvptr->vehicle_state.steer = steer;
			}
			FVector loc = FVector(gsvptr->vehicle_state.x, gsvptr->vehicle_state.y, GetActorLocation()[2]);
			gsvptr->actor->SetActorLocation(loc);
		}
	}
}

void AGSClient::CreateVehicle(int vid, int remote)
{
	UE_LOG(GeoScenarioModule, Warning, TEXT("New GSVehicle vid=%d remote=%d"), vid, remote);
	GSVehicle gsv = GSVehicle();
	gsv.vid = vid;
	gsv.remote = remote;
	if (remote == 1) 
	{
		//Find actor with tag
		UE_LOG(GeoScenarioModule, Log, TEXT("Finding Remote Vehicle\n"));
		gsv.actor = FindVehicleActor(vid);
	}
	else 
	{	
		// spawn actor
		UE_LOG(GeoScenarioModule, Log, TEXT("Spawning Sim Vehicle\n"));
		FVector location = {0.0, 0.0, 0.0};
		ASimVehicle *sv = (ASimVehicle*)GetWorld()->SpawnActor(ASimVehicle::StaticClass(), &location);
		//sv->manager = this;
		//sv->id = vid;
		gsv.actor = (AActor*) sv;
	}
	//check if success
	if (gsv.actor != nullptr)
	{
		vehicles.Add(vid, gsv);
	}
	else {UE_LOG(GeoScenarioModule, Error, TEXT("Error creating GSVehicle vid=%d remote=%d"), vid, remote);}
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


void AGSClient::WriteClientState()
{
	if (!isConnected || cs_shmInfo.shm_id < 0) { return; }

	std::stringstream oss;
	for (auto& Elem : vehicles)
	{
		//Write out Client Vehicle states
		//todo: include full state
		if (Elem.Value.actor != nullptr)
		{
			FVector loc = Elem.Value.actor->GetActorLocation();
			loc[2] = 0.0f;
			oss << Elem.Key << " " << loc[0] << " " << loc[1] << " " << loc[2] << '\n';
		}
		else{ UE_LOG(GeoScenarioModule, Error, TEXT("Cannot write Vehicle state to CS ShM. Actor is null."));}
	}

	// CS SHM ACQUIRE
	if (semop(cs_shmInfo.sem_id, &(cs_shmInfo.p), 1) < 0) {
		UE_LOG(GeoScenarioModule, Error, TEXT("Cannot p CS semaphore. Server disconnected? \n"));
		perror("p error: ");
		return;
	}
	// CS SHM WRITE
	strcpy(cs_shmInfo.shm, oss.str().c_str());
	// CS SHM RELEASE
	if (semop(cs_shmInfo.sem_id, &(cs_shmInfo.v), 1) < 0) {
		UE_LOG(GeoScenarioModule, Error, TEXT("Cannot v CS semaphore\n"));
		return;
	}
}

