
#include "GeoScenarioModule.h"

DEFINE_LOG_CATEGORY(GeoScenarioModule);
 
#define LOCTEXT_NAMESPACE "FGeoScenarioModule"
 
void FGeoScenarioModule::StartupModule()
{
	UE_LOG(GeoScenarioModule, Warning, TEXT("GeoScenarioModule has started!"));
}
 
void FGeoScenarioModule::ShutdownModule()
{
	UE_LOG(GeoScenarioModule, Warning, TEXT("GeoScenarioModule has shut down"));
}
 
#undef LOCTEXT_NAMESPACE
 
IMPLEMENT_MODULE(FGeoScenarioModule,GeoScenarioModule);