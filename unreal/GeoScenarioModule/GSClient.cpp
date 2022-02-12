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
	// Update states of remote vehicles and pedestrians
	UpdateRemoteVehicleStates(DeltaTime);
	UpdateRemotePedestrianStates(DeltaTime);
	WriteClientState(framestat.tick_count, framestat.delta_time);

	// log for experiments
	// std::stringstream oss;
	//
	// for (auto& Elem : vehicles)
	// {
	// 	GSVehicle &gsv = Elem.Value;
	// 	//Write out Client Vehicle states
	// 	//todo: include full state
	// 	if (gsv.actor != nullptr)
	// 	{
	// 		FVector loc = gsv.actor->GetActorLocation();
	// 		loc[2] = 0.0f;
	// 		oss << Elem.Key << " "
	// 			<< gsv.vehicle_state.x << " " << gsv.vehicle_state.y << " " << gsv.vehicle_state.z << " "
	// 			<< gsv.vehicle_state.x_vel << " " << gsv.vehicle_state.y_vel << '\n';
	// 	}
	// }
	// UE_LOG(GeoScenarioModule, Log, TEXT("%s | %s"), *FString(oss.str().c_str()), *FDateTime::Now().ToString());
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
        UE_LOG(GeoScenarioModule, Error,
               TEXT("Error getting CS memory ID: %s "), *FString(strerror(errno)));
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
  //GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Black, "====>>>>> Connected to GeoScenario Server");
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
	int server_tick_count, nvehicles, npedestrians, vid, pid;
	iss >> server_tick_count >> server_delta_time >> nvehicles;

	// parse vehicles
	int vehicles_read = 0;
	while (vehicles_read < nvehicles)
	{
		iss >> vid;
		if (vid==0) {continue;} //garbage at the end of string
		vehicles_read++;

		int v_type;
		float x, y, z, x_vel, y_vel, yaw, steer;
		iss >> v_type >> x >> y >> z >> x_vel >> y_vel >> yaw >> steer;
		// Unreal's y axis is inverted from GS server's.
		y *= -1;
		y_vel *= -1;
		yaw -= 90;
	    // GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Red, FString::Printf(TEXT("Yaw %f"), yaw));

		FVector loc = {x, y, z};
		FRotator rot = {0.0f, yaw, 0.0f};

		GSVehicle* gsvptr = vehicles.Find(vid);
		if (!gsvptr)
		{
			// UE_LOG(GeoScenarioModule, Warning, TEXT("DEBUG Full ISS"));
			// UE_LOG(GeoScenarioModule, Log, TEXT("%s"), *fulliss);
			//creates only if actor is spawned or found.
			CreateVehicle(vid, v_type, loc, rot);

			//debug
			continue;
		}

		if (v_type == 1)
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
				gsvptr->vehicle_state.x = loc.X;
				gsvptr->vehicle_state.y = loc.Y;
				gsvptr->vehicle_state.z = loc.Z;
				gsvptr->vehicle_state.x_vel = x_vel;
				gsvptr->vehicle_state.y_vel = y_vel;
				gsvptr->vehicle_state.yaw = rot.Yaw;
				gsvptr->vehicle_state.steer = steer;
			}
			gsvptr->actor->SetActorLocation(loc);
			gsvptr->actor->SetActorRotation(rot);
		}
	}
	//Update Server Tick
	server_framestat.tick_count = server_tick_count;
	server_framestat.delta_time = server_delta_time;
	//UE_LOG(GeoScenarioModule, Log, TEXT("SHM [ tick = %d server_delta_time = %.3f"), server_tick_count, server_delta_time);
}

// parse pedestrians
int pedestrians_read = 0;
while (pedestrians_read < npedestrians)
{
	iss >> pid;
	if (pid==0) {continue;} //garbage at the end of string
	pedestrians_read++;

	int v_type;
	float x, y, z, x_vel, y_vel, yaw;
	iss >> p_type >> x >> y >> z >> x_vel >> y_vel >> yaw;
	// Unreal's y axis is inverted from GS server's.
	y *= -1;
	y_vel *= -1;
	yaw -= 90;
		// GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Red, FString::Printf(TEXT("Yaw %f"), yaw));

	FVector loc = {x, y, z};
	FRotator rot = {0.0f, yaw, 0.0f};

	GSPedestrian* gspptr = pedestrians.Find(pid);
	if (!gspptr)
	{
		// UE_LOG(GeoScenarioModule, Warning, TEXT("DEBUG Full ISS"));
		// UE_LOG(GeoScenarioModule, Log, TEXT("%s"), *fulliss);
		//creates only if actor is spawned or found.
		CreatePedestrian(pid, p_type, loc, rot);

		//debug
		continue;
	}

	if (p_type == 4)
	{
		if (server_framestat.tick_count == server_tick_count)
		{
			//same tick, no new state
			//Predict new state based on Unreal tick time
			gspptr->pedestrian_state.x  = gspptr->pedestrian_state.x + (gspptr->pedestrian_state.x_vel * deltaTime);
			gspptr->pedestrian_state.y  = gspptr->pedestrian_state.y + (gspptr->pedestrian_state.y_vel * deltaTime);
		}
		else
		{
			gspptr->pedestrian_state.x = loc.X;
			gspptr->pedestrian_state.y = loc.Y;
			gspptr->pedestrian_state.z = loc.Z;
			gspptr->pedestrian_state.x_vel = x_vel;
			gspptr->pedestrian_state.y_vel = y_vel;
			gspptr->pedestrian_state.yaw = rot.Yaw;
		}
		gspptr->actor->SetActorLocation(loc);
		gspptr->actor->SetActorRotation(rot);
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
		if (gsv.v_type != 2) continue;

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
		// update yaw
		FRotator rot = gsv.actor->GetActorRotation();
		gsv.vehicle_state.yaw = rot.Yaw;
	}
}

void AGSClient::UpdateRemotePedestrianStates(float deltaTime)
{
	for (auto& elem : pedestrians)
	{
		GSPedestrian &gsp = elem.Value;
		if (gsp.p_type != 2) continue;

		// Update the pedestrian's pedestrian_state based on its actor's location.
		// Actual movement of the pedestrian is updated in another class.
		// Need to ensure remote movement is finished before reading its state
		// Maybe using a different component with a different tick group?
		FVector loc = gsp.actor->GetActorLocation();
		// update velocity
		gsp.pedestrian_state.x_vel = (loc[0] - gsp.pedestrian_state.x) / deltaTime;
		gsp.pedestrian_state.y_vel = (loc[1] - gsp.pedestrian_state.y) / deltaTime;
		// update position
		gsp.pedestrian_state.x = loc[0];
		gsp.pedestrian_state.y = loc[1];
		gsp.pedestrian_state.z = loc[2];
		// update yaw
		FRotator rot = gsp.actor->GetActorRotation();
		gsp.pedestrian_state.yaw = rot.Yaw;
	}
}

void AGSClient::CreateVehicle(int vid, int v_type, FVector &loc, FRotator &rot)
{
	UE_LOG(GeoScenarioModule, Warning, TEXT("New GSVehicle vid=%d v_type=%d"), vid, v_type);
	GSVehicle gsv = GSVehicle();
	gsv.vid = vid;
	gsv.v_type = v_type;
	gsv.vehicle_state =  VehicleState();
	if (v_type == 1 || v_type == 3) // SDV or TV
	{
		// spawn actor
		UE_LOG(GeoScenarioModule, Log, TEXT("Spawning Sim Vehicle"));

		ASimVehicle *sv = (ASimVehicle*)GetWorld()->SpawnActor(ASimVehicle::StaticClass(), &loc, &rot);
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
	else if (v_type == 2) // EV
	{
		//Find actor with tag
		UE_LOG(GeoScenarioModule, Log, TEXT("Finding Remote Vehicle"));
		gsv.actor = FindVehicleActor(vid);
		loc = gsv.actor->GetActorLocation();
		rot = gsv.actor->GetActorRotation();
	}

	//check if success
	if (gsv.actor != nullptr)
	{
		gsv.vehicle_state.x = loc.X;
		gsv.vehicle_state.y = loc.Y;
		gsv.vehicle_state.z = loc.Z;
		gsv.vehicle_state.yaw = rot.Yaw;

		vehicles.Add(vid, gsv);
	}
	else {UE_LOG(GeoScenarioModule, Error, TEXT("Error creating GSVehicle vid=%d v_type=%d"), vid, v_type);}
}

void AGSClient::CreatePedestrian(int pid, int p_type, FVector &loc, FRotator &rot)
{
	UE_LOG(GeoScenarioModule, Warning, TEXT("New GSPedestrian pid=%d p_type=%d"), pid, p_type);
	GSPedestrian gsp = GSPedestrian();
	gsp.pid = pid;
	gsp.p_type = p_type;
	gsp.pedestrian_state =  PedestrianState();
	if (p_type == 4 || p_type == 1) // SP or TP
	{
		// spawn actor
		UE_LOG(GeoScenarioModule, Log, TEXT("Spawning Sim Pedestrian"));

		ASimPedestrian *sp = (ASimPedestrian*)GetWorld()->SpawnActor(ASimPedestrian::StaticClass(), &loc, &rot);
		//sp->manager = this;
		//sp->id = pid;
		gsp.actor = (AActor*) sp;

		// add the tag to server pedestrians
		FString GSPedestrian = "gspedestrian";
		FName GSTag = FName(*GSPedestrian);
		gsp.actor->Tags.Add(GSTag);

		// add the tag to publish bbox
		FString PubBbox = "Bbox:1";
		FName BboxTag = FName(*PubBbox);
		gsp.actor->Tags.Add(BboxTag);
	}

	//check if success
	if (gsp.actor != nullptr)
	{
		gsp.pedestrian_state.x = loc.X;
		gsp.pedestrian_state.y = loc.Y;
		gsp.pedestrian_state.z = loc.Z;
		gsp.pedestrian_state.yaw = rot.Yaw;

		pedestrians.Add(vid, gsp);
	}
	else {UE_LOG(GeoScenarioModule, Error, TEXT("Error creating GSPedestrian pid=%d p_type=%d"), pid, p_type);}
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

AActor* AGSClient::FindPedestrianActor(int pid)
{
	static const FName GSTAG(TEXT("gspedestrian")); //faster than FString
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
					if (key == "pid")
					{
						int32 tagpid = FCString::Atoi(*value);
						if (pid == tagpid)
						{
							UE_LOG(GeoScenarioModule, Log, TEXT("GeoScenario pedestrian actor FOUND. Id %d"), tagpid);
							return *Itr;
						}
					}
				}
			}
		}
	}
	UE_LOG(GeoScenarioModule, Error, TEXT("GeoScenario pedestrian actor NOT found. Id %d"), pid);
	return nullptr;
}


void AGSClient::WriteClientState(int tickCount, float deltaTime)
{
	if (!isConnected || cs_shmInfo.shm_id < 0) { return; }

	std::stringstream oss;
	// output the correct number of pedestrians
	oss << tickCount << " " << deltaTime << " " << vehicles.Num() << " " << pedestrians.Num() << '\n';
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
				<< gsv.vehicle_state.x_vel << " " << gsv.vehicle_state.y_vel << " " /* TODO: gsv.vehicle_state.yaw << " " */
				<< active << '\n';
		}
		else
		{
			UE_LOG(GeoScenarioModule, Error, TEXT("Cannot write Vehicle state to CS ShM. Actor is null."));
		}
	}
	/* repeat the loop for pedestrians */
	for (auto& Elem : pedestrians)
	{
		GSPedestrian &gsp = Elem.Value;
		//Write out Client Pedestrian states
		//todo: include full state
		if (gsp.actor != nullptr)
		{
			FVector loc = gsp.actor->GetActorLocation();
			loc[2] = 0.0f;
			ASimPedestrian *sp = Cast<ASimPedestrian>(gsp.actor);
			int active = sp != nullptr ? (int)(sp->GetActive()) : 1;
			oss << Elem.Key << " "
				<< gsp.pedestrian_state.x << " " << gsp.pedestrian_state.y << " " << gsp.pedestrian_state.z << " "
				<< gsp.pedestrian_state.x_vel << " " << gsp.pedestrian_state.y_vel << " " /* TODO: gsp.pedestrian_state.yaw << " " */
				<< active << '\n';
		}
		else
		{
			UE_LOG(GeoScenarioModule, Error, TEXT("Cannot write Pedestrian state to CS ShM. Actor is null."));
		}
	}
  // UE_LOG(GeoScenarioModule, Log,
  //        TEXT("WriteClientState: %s | %s"),
  //             *FString(oss.str().c_str()),
  //             *FDateTime::Now().ToString());

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
